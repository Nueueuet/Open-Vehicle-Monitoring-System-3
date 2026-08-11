/*
;    Project:       Open Vehicle Monitor System
;	 Subproject:    Integrate VW e-Golf
;
;    Changes:
;    February 7 2026: Initial Implementation
;
;    (C) 2026  Erick Fuentes <fuentes.erick@gmail.com>
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

// #include <stdio.h>
#include "vehicle_vwegolf.h"

#undef TAG
#define TAG "v-vwegolf"

OvmsVehicleVWeGolf::OvmsVehicleVWeGolf() {
    ESP_LOGI(TAG, "Start vehicle module: VW e-Golf");

    // init configs:
    MyConfig.RegisterParam("xvg", "VW e-Golf", true, true);

    // Regenerative-braking strength (numeric, cheap to transmit). Decoded from the
    // gear-selector frame 0x187 in IncomingFrameCan2. -1 until first seen in D/B.
    m_recup_level = MyMetrics.InitInt("xvg.v.recup", SM_STALE_MIN, -1);

    // Zieltemperatur der Vorklimatisierung aus dem BAP-Profil. Nicht zu
    // verwechseln mit v.e.cabinsetpoint -- das kommt aus Frame 0x594 und
    // traegt denselben Wert in anderer Kodierung.
    m_cc_temp = MyMetrics.InitFloat("xvg.v.cc.temp", SM_STALE_MIN, 0, Celcius);

    // KCAN (CAN3) carries comfort, body, and clima frames via the J533 gateway.
    // FCAN (CAN2) is the powertrain bus (BMS, motor controller, VIN).
    // CAN1 (OBD) is diagnostic-only and inaccessible while the car is asleep.
    //
    // FCAN is listen-only: we read gear and VIN but never transmit on this bus.
    // Active mode would require the ESP32 CAN controller to ACK every received
    // frame; its ACK timing on a bus already managed by native ECUs produces
    // spurious ECC TX-direction errors (ecc != 0 → CAN_logerror every ~200 ms)
    // even though rxerr/txerr stay at zero. Listen-only eliminates this entirely.
    RegisterCanBus(2, CAN_MODE_LISTEN, CAN_SPEED_500KBPS);  // FCAN — powertrain (read-only)
    RegisterCanBus(3, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);  // KCAN — comfort / clima

    OvmsCommand* cmd_vweg = MyCommandApp.RegisterCommand("xvg", "VW e-Golf controls");
    cmd_vweg->RegisterCommand("offline", "Stop sending OCU keepalive", [this](...) {
        m_is_control_active = false;
        m_control_hold = 0;
        m_climate_start_requested = false;
        m_climate_stop_requested = false;
        ESP_LOGI(TAG, "OCU keepalive stopped");
    });
    cmd_vweg->RegisterCommand("fold_mirrors", "Fold mirrors in",
                              [this](...) { CommandMirrorFoldIn(); });
    cmd_vweg->RegisterCommand("profile", "Read charge/climate profile from the car",
                              [this](...) { QueueBapProfile(); });
}

OvmsVehicleVWeGolf::~OvmsVehicleVWeGolf() {
    MyCommandApp.UnregisterCommand("xvg");
    ESP_LOGI(TAG, "Stop vehicle module: VW e-Golf");
}

class OvmsVehicleVWeGolfInit {
 public:
    OvmsVehicleVWeGolfInit();
} MyOvmsVehicleVWeGolfInit __attribute__((init_priority(9000)));

OvmsVehicleVWeGolfInit::OvmsVehicleVWeGolfInit() {
    ESP_LOGI(TAG, "Registering Vehicle: VW e-Golf (9000)");
    MyVehicleFactory.RegisterVehicle<OvmsVehicleVWeGolf>("VWEG", "VW e-Golf");
}

void OvmsVehicleVWeGolf::IncomingFrameCan2(CAN_frame_t* p_frame) {
    switch (p_frame->MsgID) {
        case 0x187: {
            const uint8_t gear_nibble = p_frame->data.u8[2] & 0x0F;
            ESP_LOGV(TAG, "0x187 gear nibble=%d", gear_nibble);
            if (gear_nibble == 2) {
                // Park
                StandardMetrics.ms_v_env_gear->SetValue(0);
            } else if (gear_nibble == 3) {
                // Reverse
                StandardMetrics.ms_v_env_gear->SetValue(-1);
            } else if (gear_nibble == 4) {
                // Neutral
                StandardMetrics.ms_v_env_gear->SetValue(0);
            } else if (gear_nibble == 5) {
                // Drive
                StandardMetrics.ms_v_env_gear->SetValue(1);
            } else if (gear_nibble == 6) {
                // B mode
                StandardMetrics.ms_v_env_gear->SetValue(1);
            }

            // Regenerative-braking (recuperation) strength. The e-Golf has five
            // regen levels: D0 (coast, no regen), D1, D2, D3, and B (max). D0..D3
            // are selected with the paddles while in gear D; B is its own gear.
            // Exposed as a 0..4 strength (least->most) on xvg.v.recup.
            //
            // State is in this frame's d[1] high nibble; the top bit is always set
            // in operation, so mask it off (rc = low 3 bits). In gear D:
            //   0 = D0 coast (just shifted into D, no stage selected)
            //   1 = D1, 2 = D2, 3 = D3  (paddle regen stages)
            //   5 = D0 with recuperation switched off by the driver (also coast)
            // In gear B rc reads 0, but the gear itself means max regen.
            const uint8_t rc = (p_frame->data.u8[1] >> 4) & 0x7;
            int recup = -1;                                 // N/A unless in D or B
            if (gear_nibble == 6) {
                recup = 4;                                  // B — max regen
            } else if (gear_nibble == 5) {
                recup = (rc >= 1 && rc <= 3) ? rc : 0;      // D1/D2/D3, else D0 (coast)
            }
            m_recup_level->SetValue(recup);
            ESP_LOGV(TAG, "0x187 gear=%u rc=%u recup=%d", gear_nibble, rc, recup);
            break;
        }
        case 0x6B4: {
            // This message contains the VIN in 3 parts, with the first byte identifying the frame.
            // We only set the VIN after all three parts have been received. Once the VIN has been
            // set, we ignore future VIN frames.
            uint8_t frame_idx = p_frame->data.u8[0];
            ESP_LOGV(TAG, "0x6B4 frame_idx=%d parts=0x%02x", frame_idx, m_vin_parts_received);
            if (m_vin_parts_received == 0x07) {
                // We've already received three VIN frames and set the VIN in the metrics.
                break;
            } else if (frame_idx == 0) {
                m_vin_buf[0] = p_frame->data.u8[5];
                m_vin_buf[1] = p_frame->data.u8[6];
                m_vin_buf[2] = p_frame->data.u8[7];
                m_vin_parts_received |= 0x01;
            } else if (frame_idx == 1) {
                memcpy(&m_vin_buf[3], &p_frame->data.u8[1], 7);
                m_vin_parts_received |= 0x02;
            } else if (frame_idx == 2) {
                memcpy(&m_vin_buf[10], &p_frame->data.u8[1], 7);
                m_vin_parts_received |= 0x04;
            }

            if (m_vin_parts_received == 0x07) {
                // Set the VIN now that we've received all three parts.
                m_vin_buf[17] = '\0';
                StandardMetrics.ms_v_vin->SetValue(m_vin_buf);
            }
            break;
        }
    }
    // J533 bridges KCAN traffic onto CAN2; forward every frame so the KCAN
    // decoder in IncomingFrameCan3 can process it regardless of which bus it arrives on.
    IncomingFrameCan3(p_frame);
}

void OvmsVehicleVWeGolf::IncomingFrameCan3(CAN_frame_t* p_frame) {
    m_last_message_received = 0;

    // BAP-Anwendungsschicht erkennen. Nach dem Wecken sendet der Bus zunaechst
    // nur Statusframes; die 0x1733xxxx-Familie kommt erst rund 30 s spaeter.
    // Erst dann nimmt die Klima-ECU Kommandos an.
    if ((p_frame->MsgID & 0xFFFF0000UL) == 0x17330000UL) m_bap_ready = true;
    // Kanal 0x25 = BatteryControl (Klima/Laden). Meldet sich die ECU selbst,
    // ist sie ansprechbar und wir muessen die Einschwingzeit nicht abwarten.
    if (p_frame->MsgID == 0x17332510UL) {
        m_clima_bap_seen = true;

        // BAP-Langnachricht zusammensetzen. Kopf: 80/90/a0 <len> <func> ...,
        // Fortsetzungen c0/d0/e0 aufwaerts. Wir verfolgen nur den Strang, der
        // die Profilliste traegt; die anderen laufen parallel und werden
        // ignoriert.
        const uint8_t* b = p_frame->data.u8;
        uint8_t dlc = p_frame->FIR.B.DLC;
        // Doppelte Zustellung abfangen: IncomingFrameCan2 reicht die vom J533
        // gespiegelten Frames hier ebenfalls herein, jeder BAP-Frame kommt also
        // zweimal an. Fuer die zustandslosen Dekoder ist das folgenlos, den
        // Zusammensetzer der Langnachricht bringt es aus dem Tritt: Der zweite
        // c0 passt nicht mehr zum erwarteten c1, der Strang bricht ab.
        // Innerhalb einer Langnachricht zaehlt der Marker hoch -- zwei gleiche
        // Frames hintereinander sind daher immer eine Wiederholung.
        if (dlc == m_prof_prev_dlc && dlc <= 8
            && memcmp(b, m_prof_prev, dlc) == 0) goto bap_done;
        m_prof_prev_dlc = dlc;
        for (uint8_t i = 0; i < dlc && i < 8; i++) m_prof_prev[i] = b[i];
        if (dlc >= 4 && (b[0] == 0x80 || b[0] == 0x90 || b[0] == 0xA0)) {
            uint16_t func = ((uint16_t)b[2] << 8) | b[3];
            ESP_LOGI(TAG, "BAP head: %02x len=%u func=%04x", b[0], b[1], func);
            if ((func & VWEGOLF_BAP_FUNC_MASK) == VWEGOLF_BAP_PROP_PROFILE
                && b[1] <= VWEGOLF_PROF_MAXLEN) {
                m_prof_len = b[1];
                m_prof_pos = 0;
                m_prof_active = true;
                m_prof_gap = false;
                // Fortsetzungsmarker gehoert zum selben Strang: 80->c0, 90->d0, a0->e0
                m_prof_cont = (b[0] == 0x80) ? 0xC0 : ((b[0] == 0x90) ? 0xD0 : 0xE0);
                for (uint8_t i = 4; i < dlc && m_prof_pos < m_prof_len; i++)
                    m_prof_buf[m_prof_pos++] = b[i];
                if (m_prof_pos >= m_prof_len) { m_prof_active = false; ParseBapProfile(); }
            }
        } else if (m_prof_active && dlc >= 2
                   && (b[0] & 0xF0) == (m_prof_cont & 0xF0)) {
            if (b[0] != m_prof_cont) {
                // Frame verloren. Luecke stopfen, damit die folgenden Bytes an
                // der richtigen Stelle landen, und das Profil als unbrauchbar
                // fuer das Zurueckschreiben markieren.
                int8_t skip = (int8_t)(b[0] - m_prof_cont);
                if (skip < 1 || skip > 4) {
                    ESP_LOGW(TAG, "BAP stream lost (want %02x got %02x), abort",
                             m_prof_cont, b[0]);
                    m_prof_active = false;
                    m_prof_len = 0;
                    goto bap_done;
                }
                ESP_LOGW(TAG, "BAP gap: %d frame(s) missing before %02x", skip, b[0]);
                m_prof_gap = true;
                for (int8_t s = 0; s < skip; s++)
                    for (uint8_t k = 0; k < 7 && m_prof_pos < m_prof_len; k++)
                        m_prof_buf[m_prof_pos++] = 0xFF;
                m_prof_cont = b[0];
            }
            for (uint8_t i = 1; i < dlc && m_prof_pos < m_prof_len; i++)
                m_prof_buf[m_prof_pos++] = b[i];
            m_prof_cont++;
            if (m_prof_pos >= m_prof_len) { m_prof_active = false; ParseBapProfile(); }
        }
        bap_done: ;
    }

    uint8_t* d = p_frame->data.u8;

    uint8_t tmp_u8 = 0;
    uint16_t tmp_u16 = 0;
    uint32_t tmp_u32 = 0;
    // int8_t tmp_i8 = 0;
    // int16_t tmp_i16 = 0;
    // int32_t tmp_i32 = 0;
    float tmp_f32 = 0.0F;

    // //TODO Debug only doesn't work ECU timeout
    // vTaskDelay(pdMS_TO_TICKS(500)); //500ms wait.. hopefully my log isn't get messed up
    // //TODO end doesn't work ECU timeout

    switch (p_frame->MsgID) {
        // TODO: Need to move to verify
        case 0xFD:  // Vehicle speed from ESP module. 16-bit LE in d[4:5], factor 0.01 km/h.
        {
            tmp_u16 = ((uint16_t)(d[4]) >> 0) | ((uint16_t)(d[5]) << 8);
            tmp_f32 = ((float)tmp_u16) * 0.01F;
            StandardMetrics.ms_v_pos_speed->SetValue(tmp_f32);
            ESP_LOGV(TAG, "0x00FD speed=%.2f km/h", tmp_f32);
            break;
        }
        case 0x131:  // State of charge. d[3] * 0.5%. 0xFE = "not ready" sentinel (127%).
        {
            if (d[3] == 0xFE) break;
            tmp_f32 = ((float)d[3]) * 0.5F;
            StandardMetrics.ms_v_bat_soc->SetValue(tmp_f32);
            ESP_LOGV(TAG, "0x0131 soc=%.1f%%", tmp_f32);
            break;
        }
        case 0x191:  // BMS current, voltage, power.
        {
            // Startup sentinel: d[2]=0xFF decodes to I=2047 A and V=1023.5 V. Discard it.
            if (d[2] == 0xFF) break;

            // Current: 12-bit, factor 1 A, offset -2047 A.
            tmp_u16 = ((uint16_t)(d[1] & 0xf0) >> 4) | ((uint16_t)(d[2]) << 4);
            tmp_f32 = ((float)tmp_u16) * 1.0F - 2047.0F;
            StandardMetrics.ms_v_bat_current->SetValue(tmp_f32);

            // Voltage: 12-bit, factor 0.25 V.
            tmp_u16 = ((uint16_t)(d[3])) | ((uint16_t)(d[4] & 0xf) << 8);
            tmp_f32 = ((float)tmp_u16) * 0.25F;
            StandardMetrics.ms_v_bat_voltage->SetValue(tmp_f32);

            // Power: negative = charging, positive = driving.
            tmp_f32 = -1.0F * (StandardMetrics.ms_v_bat_voltage->AsFloat() *
                               StandardMetrics.ms_v_bat_current->AsFloat()) / 1000.0F;
            StandardMetrics.ms_v_bat_power->SetValue(tmp_f32);
            ESP_LOGV(TAG, "0x0191 I=%.1fA V=%.2fV", StandardMetrics.ms_v_bat_current->AsFloat(),
                     StandardMetrics.ms_v_bat_voltage->AsFloat());
            break;
        }
        case 0x2AF:  // Trip energy counters. 15-bit, factor 10 Ws → kWh.
        {
            // Regen energy: d[4] + d[5] bits [6:0]. Max raw 32767 * 10 = 327670 Ws.
            tmp_f32 = (float)(d[4] | ((uint16_t)(d[5] & 0x7f) << 8)) * 10.0F / 3600000.0F;
            StandardMetrics.ms_v_bat_energy_recd->SetValue(tmp_f32);

            // Consumed energy: d[6] + d[7] bits [6:0].
            tmp_f32 = (float)(d[6] | ((uint16_t)(d[7] & 0x7f) << 8)) * 10.0F / 3600000.0F;
            StandardMetrics.ms_v_bat_energy_used->SetValue(tmp_f32);
            ESP_LOGV(TAG, "0x02AF recd=%.4f used=%.4f kWh",
                     StandardMetrics.ms_v_bat_energy_recd->AsFloat(),
                     StandardMetrics.ms_v_bat_energy_used->AsFloat());
            break;
        }
        // case 0x3D6: //Ladezustand
        //   {
        //     ESP_LOGD(TAG, "chargeState: %x", ((d[6] & 0x30)>>4));
        //     if(((d[6] & 0x30)>>4) == 0x0)
        //     {
        //       StdMetrics.ms_v_charge_state->SetValue("stopped");
        //       ESP_LOGV(TAG, "charging stopped");
        //     }
        //     if(((d[6] & 0x30)>>4) == 0x1)
        //     {
        //       StdMetrics.ms_v_charge_state->SetValue("charging");
        //       ESP_LOGV(TAG, "charging");
        //     }
        //     if(((d[6] & 0x30)>>4) == 0x2)
        //     {
        //       StdMetrics.ms_v_charge_state->SetValue("init");
        //       ESP_LOGV(TAG, "charging init");
        //     }
        //     if(((d[6] & 0x30)>>4) == 0x3)
        //     {
        //       StdMetrics.ms_v_charge_state->SetValue("error");
        //       ESP_LOGV(TAG, "charging error");
        //     }

        //   break;
        //   }
        // Working, but sign bit missing
        case 0x486:  // GPS position. Lat: bits 0-26 (factor 1e-6°), Lon: bits 27-54.
        {
            // Sign bits: bit 55 (d[6] MSB) = Southern hemisphere, bit 56 (d[7] bit 0) = Western.
            // Confirmed consistent with known N/E location. S/W hemisphere still needs a capture.
            // Sentinel frames (all 0xFF) decode to lat=134°/lon=268° — filter by range.
            tmp_u32 = ((uint32_t)(d[0])) | ((uint32_t)(d[1]) << 8) |
                      ((uint32_t)(d[2]) << 16) | ((uint32_t)(d[3] & 0x7) << 24);
            float lat = ((float)tmp_u32) * 0.000001F;
            if ((d[6] >> 7) & 1) lat = -lat;  // Southern hemisphere

            tmp_u32 = ((uint32_t)(d[3] & 0xf8) >> 3) | ((uint32_t)(d[4]) << 5) |
                      ((uint32_t)(d[5]) << 13) | ((uint32_t)(d[6] & 0x7f) << 21);
            float lon = ((float)tmp_u32) * 0.000001F;
            if ((d[7] >> 0) & 1) lon = -lon;  // Western hemisphere

            bool valid = (lat > -91.0F && lat < 91.0F && lon > -181.0F && lon < 181.0F);
            StandardMetrics.ms_v_pos_gpslock->SetValue(valid);
            if (valid) {
                StandardMetrics.ms_v_pos_latitude->SetValue(lat);
                StandardMetrics.ms_v_pos_longitude->SetValue(lon);
            }
            ESP_LOGV(TAG, "0x0486 lat=%.6f lon=%.6f valid=%d", lat, lon, valid);
            break;
        }
        case 0x386:  // Drive mode (Charisma / Fahrprofilauswahl active profile).
        {
            // d[5] = active drive profile: 0x02 = Normal, 0x05 = Eco, 0x08 = Eco+
            // (matches the MIB CharismaProfiles enum auto_normal=2/efficiency=5/range=8).
            // Mapped to ms_v_env_drivemode as 0 = Normal, 1 = Eco, 2 = Eco+.
            switch (d[5]) {
                case 0x02:
                    StandardMetrics.ms_v_env_drivemode->SetValue(0);
                    break;
                case 0x05:
                    StandardMetrics.ms_v_env_drivemode->SetValue(1);
                    break;
                case 0x08:
                    StandardMetrics.ms_v_env_drivemode->SetValue(2);
                    break;
                default:
                    // Unknown profile value — leave the last known drive mode.
                    break;
            }
            ESP_LOGV(TAG, "0x0386 drivemode raw=0x%02x", d[5]);
            break;
        }
        case 0x583:  // ZV_02: central locking and door open states.
        {
            // d[2] bit 1: locked externally. d[3] bits 4:0: trunk, rr, rl, fr, fl (1=open).
            StdMetrics.ms_v_env_locked->SetValue((d[2] & 0x2) >> 1);
            StdMetrics.ms_v_door_fl->SetValue((d[3] & 0x1) >> 0);
            StdMetrics.ms_v_door_fr->SetValue((d[3] & 0x2) >> 1);
            StdMetrics.ms_v_door_rl->SetValue((d[3] & 0x4) >> 2);
            StdMetrics.ms_v_door_rr->SetValue((d[3] & 0x8) >> 3);
            StdMetrics.ms_v_door_trunk->SetValue((d[3] & 0x10) >> 4);
            ESP_LOGV(TAG, "0x0583 locked=%u fl=%u fr=%u rl=%u rr=%u trunk=%u",
                     (d[2] & 0x2) >> 1, d[3] & 0x1, (d[3] & 0x2) >> 1,
                     (d[3] & 0x4) >> 2, (d[3] & 0x8) >> 3, (d[3] & 0x10) >> 4);
            break;
        }
        case 0x594:  // HV charge management
        {
            // 0x594 => AC/DC charging, is climate timer active or not, plug connected (secured or
            // not), programmed cabin temp, charging active, time until HV battery full in 5min
            // steps
            tmp_u16 = ((uint16_t)(d[1] & 0xf0) >> 4) | ((uint16_t)(d[2] & 0x1f) << 4) |
                      0;  // Faktor 5 Offset 0, Minimum 0, Maximum 2545 [5min] Initial 2550
            tmp_u16 = (uint16_t)tmp_u16;
            tmp_u16 = (((int16_t)tmp_u16) * 5);
            StdMetrics.ms_v_charge_duration_full->SetValue(
                tmp_u16,
                Minutes);  // working          // Estimated time remaing for full charge [min]

            tmp_u8 = ((uint8_t)(d[2] & 0x60) >> 5) |
                     0;  // Faktor 1 Offset 0, Minimum 0, Maximum 3 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;
            if (tmp_u8 == 0x1) {
                StdMetrics.ms_v_charge_timermode->SetValue(true);  // True if timer enabled
            } else {
                StdMetrics.ms_v_charge_timermode->SetValue(false);  // false if timer disabled
            }

            {
                bool was_charging = StdMetrics.ms_v_charge_inprogress->AsBool();
                bool is_charging = (d[3] & 0x20) != 0;  // bit 5 of d[3]
                StdMetrics.ms_v_charge_inprogress->SetValue(is_charging);
                StdMetrics.ms_v_charge_state->SetValue(is_charging ? "charging" : "stopped");
                if (is_charging) {
                    StdMetrics.ms_v_charge_voltage->SetValue(
                        StandardMetrics.ms_v_bat_voltage->AsFloat());
                }
                if (is_charging != was_charging) {
                    if (is_charging) NotifyChargeStart();
                    else NotifyChargeStopped();
                }
            }

            tmp_u16 = ((uint16_t)(d[3] & 0xc0) >> 6) | ((uint16_t)(d[4] & 0x7f) << 2) |
                      0;  // Faktor 50 Offset 0, Minimum 0, Maximum 25450 [W] Initial 25500
            tmp_u16 = (uint16_t)tmp_u16;
            tmp_u16 = (((int16_t)tmp_u16) * 50);  // maximum charging power

            tmp_u8 = ((uint8_t)(d[4] & 0x80) >> 7) |
                     0;  // Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;
            if (tmp_u8 == 0x1) {
                // parking climate control timer set
            } else {
                // parking climate control timer not set
            }

            tmp_u8 = ((uint8_t)(d[5] & 0x1) << 0) |
                     0;  // Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;
            if (tmp_u8 == 0x1) {
                // error charging plug
            } else {
                // no error charging plug
            }

            tmp_u8 = ((uint8_t)(d[5] & 0x2) >> 1) |
                     0;  // Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;
            if (tmp_u8 == 0x1) {
                // error charging plug lock
            } else {
                // no error charging plug lock
            }

            tmp_u8 = ((uint8_t)(d[5] & 0xc) >> 2) |
                     0;  // Faktor 1 Offset 0, Minimum 0, Maximum 3 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;
            // Charge port open = cable physically present (ChargeType != 0).
            // The framework's status display gates on ms_v_door_chargeport — without it,
            // the "Not charging" fallback always shows regardless of charge_inprogress.
            switch (tmp_u8) {
                case 0x0: {
                    // No connector — do not overwrite last known type with "undefined".
                    // NOTE: CCS DC charging also reads 0 here; the CCS indicator is
                    // elsewhere in the frame and not yet identified.
                    StdMetrics.ms_v_door_chargeport->SetValue(false);
                    break;
                }
                case 0x1: {
                    StdMetrics.ms_v_charge_type->SetValue("type2");
                    StdMetrics.ms_v_door_chargeport->SetValue(true);
                    break;
                }
                case 0x2: {
                    StdMetrics.ms_v_charge_type->SetValue("ccs");
                    StdMetrics.ms_v_door_chargeport->SetValue(true);
                    break;
                }
                case 0x3: {
                    // Cable connected, charge complete or not needed (e.g. 100% SoC).
                    StdMetrics.ms_v_door_chargeport->SetValue(true);
                    break;
                }
                default:
                    break;
            }

            tmp_u8 = ((uint8_t)(d[5] & 0x10) >> 4) |
                     0;  // Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;
            if (tmp_u8 == 0x1) {
                // vehicle connected to power grid
            } else {
                // vehicle not connected to power grid
            }

            tmp_u8 = ((uint8_t)(d[5] & 0x60) >> 5) |
                     0;  // Faktor 1 Offset 0, Minimum 0, Maximum 3 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;
            switch (tmp_u8) {
                case 0x0: {
                    // no HV request
                    break;
                }
                case 0x1: {
                    // charging request
                    break;
                }
                case 0x2: {
                    // conditioning request
                    break;
                }
                case 0x3: {
                    // climatisation request
                    break;
                }
            }

            // tmp_u8 =
            // ((uint8_t) (d[5] & 0x80) >> 7) |
            // 0; // Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // tmp_u8 = (uint8_t) tmp_u8;
            // if(tmp_u8 == 0x1)
            // {
            //   // no conditioning cabin request
            // }
            // else
            // {
            //   //conditioning cabin request
            // }

            tmp_u8 = ((uint8_t)(d[6] & 0xc) >> 2) |
                     0;  // Faktor 1 Offset 0, Minimum 0, Maximum 3 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;
            switch (tmp_u8) {
                case 0x0: {
                    // no conditioning request
                    break;
                }
                case 0x1: {
                    // instant conditioning request
                    break;
                }
                case 0x2: {
                    // timed conditioning request
                    break;
                }
                case 0x3: {
                    // error conditioning request
                    break;
                }
            }

            tmp_u8 = ((uint8_t)(d[6] & 0x30) >> 4) |
                     0;  // Faktor 1 Offset 0, Minimum 0, Maximum 3 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;
            switch (tmp_u8) {
                case 0x0: {
                    // plug status init
                    break;
                }
                case 0x1: {
                    // no plug detected
                    break;
                }
                case 0x2: {
                    // plug detected but not locked
                    break;
                }
                case 0x3: {
                    // eplug detected and locked
                    break;
                }
            }
            // StdMetrics.ms_v_charge_state;                  // charging, topoff, done, prepare,
            // timerwait, heating, stopped StdMetrics.ms_v_charge_substate;               //
            // scheduledstop, scheduledstart, onrequest, timerwait, powerwait, stopped, interrupted

            tmp_u8 = ((uint8_t)(d[7] & 0x1F) << 0) |
                     0;  // Faktor 0.5 Offset 15.5, Minimum 15.5, Maximum 29.5 [°C] Initial 30.5
            tmp_u8 = (uint8_t)tmp_u8;
            tmp_f32 = ((float)tmp_u8) * 0.5F + 15.5F;
            StandardMetrics.ms_v_env_cabinsetpoint->SetValue(
                tmp_f32);  // working            // Cabin setpoint temperature [°C]

            tmp_u8 = ((uint8_t)(d[7] & 0x20) >> 5) |
                     0;                /// Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;  // 0x0 no rear window heating 0x1 rear window heating
            // if(tmp_u8 == 0x1)
            // {
            //   //
            // }
            // else
            // {
            //   //
            // }

            tmp_u8 = ((uint8_t)(d[7] & 0x40) >> 6) |
                     0;                /// Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;  // 0x0 no hv bat conditioning 0x1 hv bat conditioning
            // if(tmp_u8 == 0x1)
            // {
            //   //
            // }
            // else
            // {
            //   //
            // }

            tmp_u8 = ((uint8_t)(d[7] & 0x80) >> 7) |
                     0;                /// Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            tmp_u8 = (uint8_t)tmp_u8;  // 0x0 no front window heating 0x1 front window heating
            // if(tmp_u8 == 0x1)
            // {
            //   //
            // }
            // else
            // {
            //   //
            // }

            ESP_LOGV(TAG, "0x0594 charging=%d timer=%d type=%s setpoint=%.1f°C",
                     StdMetrics.ms_v_charge_inprogress->AsBool(),
                     StdMetrics.ms_v_charge_timermode->AsBool(),
                     StdMetrics.ms_v_charge_type->AsString().c_str(),
                     StdMetrics.ms_v_env_cabinsetpoint->AsFloat());

            break;
        }
        case 0x59E:  // BMS battery pack temperature. Factor 0.5°C, offset -40°C.
        {
            // 0xFE/0xFF are startup sentinels (decode to 87/87.5°C). Discard them.
            if (d[2] >= 0xFE) break;
            tmp_f32 = ((float)d[2]) * 0.5F - 40.0F;
            StandardMetrics.ms_v_bat_temp->SetValue(tmp_f32);
            ESP_LOGV(TAG, "0x059E bat_temp=%.1f°C", tmp_f32);
            break;
        }
        case 0x5CA:  // HV battery energy content. 11-bit, factor 50 Wh → kWh.
        {
            // Near-max raw value (upper 7 bits of d[2] all set) is a startup sentinel
            // that decodes to ~102 kWh — well above the physical 35.8 kWh capacity.
            if ((d[2] & 0x7F) == 0x7F) break;
            tmp_u16 = ((uint16_t)(d[1] & 0xf0) >> 4) | ((uint16_t)(d[2] & 0x7f) << 4);
            tmp_f32 = ((float)tmp_u16) * 50.0F / 1000.0F;
            StandardMetrics.ms_v_bat_capacity->SetValue(tmp_f32);
            ESP_LOGV(TAG, "0x05CA bat_capacity=%.1f kWh", tmp_f32);
            break;
        }
        case 0x5EA:  // Clima ECU status: cabin temperature and remote mode.
        {
            // HVAC on = climate is actively conditioning: d[3] bit 3 (0x08).
            // NOTE: d[3] bits 6-7 mark a remote-climate *session* but read the same
            // (=2) whether merely armed (d3=0x80) or actually running (d3=0x88), so
            // keying HVAC on those bits leaves it stuck true after a stop (the BCU
            // drops 0x88 -> 0x80 when conditioning ends). The conditioning bit (0x08)
            // clears on stop, so use it. Set unconditionally (before the cabin-temp
            // sentinel guard) so HVAC always tracks even when temp is the sentinel.
            StandardMetrics.ms_v_env_hvac->SetValue((d[3] & 0x08) != 0);

            // Cabin temperature: 10-bit, factor 0.1°C, offset -40°C.
            // Near-max raw value is a startup sentinel decoding to ~62°C. Discard it.
            tmp_u16 = ((uint16_t)(d[6] & 0xfc) >> 2) | ((uint16_t)(d[7] & 0xf) << 6);
            if (tmp_u16 < 0x3FE) {
                tmp_f32 = ((float)tmp_u16) * 0.1F - 40.0F;
                StandardMetrics.ms_v_env_cabintemp->SetValue(tmp_f32);
            }
            ESP_LOGV(TAG, "0x05EA hvac=%d d3=%02x", (d[3] & 0x08) != 0, d[3]);
            break;
        }
        case 0x5F5:  // Range estimates from the instrument cluster.
        {
            // Estimated range (matches instrument cluster display): 11-bit, factor 1 km.
            tmp_u16 = ((uint16_t)(d[3] & 0xe0) >> 5) | ((uint16_t)(d[4]) << 3);
            StandardMetrics.ms_v_bat_range_est->SetValue((float)tmp_u16);

            // Ideal range (BMS model, typically lower than estimated): 11-bit, factor 1 km.
            tmp_u16 = ((uint16_t)(d[0])) | ((uint16_t)(d[1] & 0x7) << 8);
            StdMetrics.ms_v_bat_range_ideal->SetValue((float)tmp_u16);
            ESP_LOGV(TAG, "0x05F5 range_est=%u range_ideal=%u km",
                     StandardMetrics.ms_v_bat_range_est->AsInt(),
                     StdMetrics.ms_v_bat_range_ideal->AsInt());
            break;
        }
        case 0x65A:  // BCM_01: bonnet/hood open indicator (MHWIVSchalter, d[4] bit 0).
        {
            StdMetrics.ms_v_door_hood->SetValue(d[4] & 0x1);
            ESP_LOGV(TAG, "0x065A hood=%u", d[4] & 0x1);
            break;
        }
        case 0x66E:  // InnenTemp: cabin interior temperature sensor.
        {
            // 0xFE is the ECU's "not ready" sentinel (decodes to 77°C). Discard it.
            if (d[4] == 0xFE) break;
            tmp_f32 = ((float)d[4]) * 0.5F - 50.0F;
            StandardMetrics.ms_v_env_cabintemp->SetValue(tmp_f32);
            ESP_LOGV(TAG, "0x066E cabin_temp=%.1f°C", tmp_f32);
            break;
        }
        case 0x6B0:  // FS temperature sensor (windshield/front area). Not yet mapped to a metric.
        {
            tmp_f32 = ((float)d[4]) * 0.5F - 40.0F;
            ESP_LOGV(TAG, "0x06B0 fs_temp=%.1f°C", tmp_f32);
            break;
        }
        case 0x6B5:  // Ambient temperature: solar sensor and outside air.
        {
            tmp_u16 = ((uint16_t)(d[6])) | ((uint16_t)(d[7] & 0x7) << 8);
            ESP_LOGV(TAG, "0x06B5 solar_sensor=%.1f°C", ((float)tmp_u16) * 0.1F - 40.0F);
            tmp_u16 = ((uint16_t)(d[2])) | ((uint16_t)(d[3] & 0x3) << 8);
            ESP_LOGV(TAG, "0x06B5 air_sensor=%.1f°C", ((float)tmp_u16) * 0.1F - 40.0F);
            break;
        }
        case 0x6B7:  // AussenTemp gefiltert Kilometerstand
        {
            tmp_u32 =
                ((uint32_t)(d[0] & 0xff) << 0) | ((uint32_t)(d[1] & 0xff) << 8) |
                ((uint32_t)(d[2] & 0xf) << 16) |
                0;  // odometer Faktor 1 Offset 0, Minimum 0, Maximum 1045873 [km] Initial 1045874
            tmp_u32 = (uint32_t)tmp_u32;
            // tmp_f32 = ((float)tmp_u32)*1.0F;
            StandardMetrics.ms_v_pos_odometer->SetValue(tmp_u32);  // working
            ESP_LOGV(TAG, "0x06B7 odo=%u km", tmp_u32);

            // Park time: 17-bit field at bit offset 20, factor 1 s.
            // d[2] bits [7:4] → result bits [3:0], d[3] → [11:4], d[4] bits [4:0] → [16:12].
            tmp_u32 =
                ((uint32_t)(d[2] & 0xf0) >> 4) | ((uint32_t)(d[3]) << 4) |
                ((uint32_t)(d[4] & 0x1f) << 12);
            StandardMetrics.ms_v_env_parktime->SetValue(tmp_u32);
            ESP_LOGV(TAG, "0x06B7 parktime=%u", tmp_u32);

            tmp_u8 = ((uint8_t)(d[7] & 0xff) << 0) |
                     0;  // outerTemp Faktor 0.5 Offset -50, Minimum -50, Maximum 75 [°C] Initial 77
            tmp_u8 = (uint8_t)tmp_u8;
            tmp_f32 = ((float)tmp_u8) * 0.5F - 50.0F;
            StandardMetrics.ms_v_env_temp->SetValue(tmp_f32);  // working
            ESP_LOGV(TAG, "0x06B7 outside=%.1f°C", tmp_f32);
            break;
        }
        case 0x391:  // OBD_01: drivetrain READY status
        {
            // d[7] bit 5 is OBD_Driving_Cycle: it goes high only once the drivetrain is fully
            // up and the car is ready to drive; it stays clear during charging, remote climate
            // and while the ignition is merely on but not yet READY. The frame keeps
            // broadcasting after the ignition goes off and the bit stays latched high for
            // several seconds into the power-down, so it is combined with KL_15 for v.e.on
            // rather than used on its own. if only ignition is turned on again this bit is cleared.
            // (d[5] carries the accelerator pedal position, OBD_Abs_Pedal_Pos - not mapped.)
            m_drivetrain_ready = (d[7] & 0x20) != 0;
            StandardMetrics.ms_v_env_on->SetValue(m_kl15_on && m_drivetrain_ready);
            ESP_LOGV(TAG, "0x391 READY=%u", m_drivetrain_ready);
            break;
        }
        case 0x3C0:  // clamp status received
        {
            // the following are from d[2]
            // KL_S Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_15 Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_X Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_50 Startanforderung Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // Remotestart Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_Infotainment Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // Remotestart_KL15_Anf Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // Remotestart_Motor_Start Faktor 1 Offset 0, Minimum 0, Maximum 1 [] Initial 0
            // KL_15 (terminal 15 = ignition) means the car is awake and switched on by the
            // user; drivable (v.e.on) additionally requires the drivetrain to report READY.
            m_kl15_on = (d[2] & 0x02) != 0;
            StandardMetrics.ms_v_env_awake->SetValue(m_kl15_on);
            StandardMetrics.ms_v_env_on->SetValue(m_kl15_on && m_drivetrain_ready);
            // Regen level (xvg.v.recup) is a driving concept — clear it to N/A when
            // ignition goes off so it doesn't linger on the last D/B value. The gear
            // frame 0x187 stops once the car is off, so without this the metric would
            // hold stale until its auto-stale timeout.
            if (!m_kl15_on)
                m_recup_level->SetValue(-1);
            ESP_LOGV(TAG, "0x3C0 KL_15=%u KL_S=%u", m_kl15_on, d[2] & 0x01);
            break;
        }
        default: {
            // only for debug log ALL Incoming frames As i didn't know what frames are coming in
            // after wakeup had to log all only all unknown ESP_LOGI(TAG, "T26: timestamp: %.24i
            // 3R29 %12x %02x %02x %02x %02x %02x %02x %02x %02x",
            // StandardMetrics.ms_m_monotonic->AsInt(), p_frame->MsgID, d[0], d[1], d[2], d[3],
            // d[4], d[5], d[6], d[7]);
            break;
        }
    }
}

// ms_v_env_awake;                     // Vehicle is fully awake (switched on by the user)
// ms_v_env_on;                        // Vehicle is in "ignition" state (drivable)

// 0x12dd546f BCM_04... ansteuerung LED Ladeanzeige mit Dimmung und Farbe?

// 0x5B0 => TimeDate

void OvmsVehicleVWeGolf::Ticker1(uint32_t ticker) {
    // 10 seconds after last received message we assume that the car is sleeping
    m_is_car_online = m_last_message_received < 10;

    if (m_last_message_received < 254) m_last_message_received++;
    ESP_LOGV(TAG, "0x5A7 last_msg=%u", m_last_message_received);

    // awake (KL_15) and drivable (KL_15 && READY) are set from the terminal / READY
    // frames in IncomingFrameCan3. When the bus goes silent the car is asleep: clear
    // both as a backstop in case those frames stopped before signalling the transition.
    if (!m_is_car_online) {
        m_kl15_on = false;
        m_drivetrain_ready = false;
        StandardMetrics.ms_v_env_awake->SetValue(false);
        StandardMetrics.ms_v_env_on->SetValue(false);
        // Clima ECU (0x5EA) is silent once the bus sleeps, so it can't refresh HVAC;
        // clear it so climate doesn't read "on" forever after conditioning ends.
        StandardMetrics.ms_v_env_hvac->SetValue(false);
        // Mit dem Bus schlaeft auch die BAP-Schicht ein.
        m_bap_ready = false;
        m_clima_bap_seen = false;
        m_online_secs = 0;
    } else if (m_online_secs < 255) {
        m_online_secs++;
    }

    // Netzwerkmanagement zyklisch senden, solange wir die Steuerung halten.
    // Das weckt einen schlafenden Bus und haelt ihn wach -- hoert man auf,
    // schlaeft das Fahrzeug binnen Sekunden wieder ein. Bewusst auch dann, wenn
    // der Bus noch still ist: genau das ist der Weckvorgang.
    if (m_is_control_active) {
        if (m_nm_backoff > 0) {
            // Voriger Frame haengt noch in der Warteschlange -- aussetzen. Der
            // Controller wiederholt ihn ohnehin selbst, und genau diese
            // Busaktivitaet weckt die schlafenden Steuergeraete.
            m_nm_backoff--;
        } else if (!SendNetworkManagement()) {
            m_nm_backoff = 3;
            if (m_nm_stalled < 255) m_nm_stalled++;
            if (m_nm_stalled == 8)
                ESP_LOGW(TAG,
                         "Network management stalled: bus is not acknowledging. "
                         "Vehicle may be unreachable until it wakes on its own.");
        } else {
            m_nm_stalled = 0;
        }
    } else {
        m_nm_backoff = 0;
        m_nm_stalled = 0;
    }

    if (m_is_control_active &&
        m_is_car_online)  // after wakeup the other ECUs waiting for the car to be online before we
                          // send some Heartbeat messages otherwise we have some serious txerrors
                          // just before the car ist active
    {
        SendOcuHeartbeat();  // working
        ESP_LOGV(TAG, "Heartbeat sending triggered");
    }

    // OCU heartbeat auto-teardown ("deliver and release"). CommandWakeup /
    // CommandClimateControl arm m_control_hold; the window counts down every second
    // (bounded even if the car never wakes) and then releases the heartbeat — no
    // manual `xvg offline` needed. SendClimateControl re-arms the window on delivery
    // so there's a full grace period after the command actually goes out; if the
    // window expires with a command still queued, the command is dropped. `xvg
    // offline` still forces it off immediately.
    if (m_is_control_active) {
        if (m_control_hold > 0)
            m_control_hold--;
        if (m_control_hold == 0) {
            if (m_climate_start_requested || m_climate_stop_requested)
                ESP_LOGW(TAG, "OCU hold expired with climate command undelivered — dropping");
            m_climate_start_requested = false;
            m_climate_stop_requested = false;
            m_is_control_active = false;
            ESP_LOGI(TAG, "OCU hold window expired — releasing heartbeat (control inactive)");
        }
    }
}

OvmsVehicle::vehicle_command_t OvmsVehicleVWeGolf::CommandLock(const char* pin) {
    if (!PinCheck(pin)) {
        ESP_LOGW(TAG, "PinCheck failed in CommandLock");
        return Fail;
    }
    m_lock_requested = true;
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleVWeGolf::CommandUnlock(const char* pin) {
    if (!PinCheck(pin)) {
        ESP_LOGW(TAG, "PinCheck failed in CommandUnlock");
        return Fail;
    }
    m_unlock_requested = true;
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleVWeGolf::CommandMirrorFoldIn() {
    m_mirror_fold_in_requested = true;
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleVWeGolf::CommandHorn() {
    m_horn_requested = true;
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleVWeGolf::CommandIndicators() {
    m_indicators_requested = true;
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleVWeGolf::CommandPanic() {
    m_panic_mode_requested = true;
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleVWeGolf::CommandClimateControl(bool enable) {
    ESP_LOGI(TAG, "CommandClimateControl %s", enable ? "ON" : "OFF");
    // Queue the request; the BAP frames are actually sent from SendOcuHeartbeat,
    // which only runs once the comfort bus is awake and the OCU heartbeat is
    // flowing (m_is_control_active && m_is_car_online). Latest request wins.
    m_climate_start_requested = enable;
    m_climate_stop_requested = !enable;
    // Climate needs the car awake — this deliberately uses the OCU wake/heartbeat
    // path. If the bus is asleep, wake it; the queued command goes out on the next
    // heartbeat tick after the car comes online.
    bool was_asleep = !m_is_car_online;
    if (was_asleep)
        CommandWakeup();
    m_is_control_active = true;
    // Aus dem Tiefschlaf braucht die BAP-Schicht rund 30 s. Mit den 15 s des
    // Normalfalls lief das Fenster ab, bevor der Befehl abgesetzt werden konnte.
    m_control_hold = was_asleep ? VWEGOLF_OCU_WAKE_SECS : VWEGOLF_OCU_HOLD_SECS;
    return Success;
}

void OvmsVehicleVWeGolf::Ticker10(uint32_t ticker) {
    // working
    m_climate_control_temp = MyConfig.GetParamValueInt("xvg", "cc_temp", 21);
    m_climate_control_on_battery = (MyConfig.GetParamValueBool("xvg", "cc_onbat", false) ? 1 : 0);

    ESP_LOGV(TAG,
             "Trigger10 cc_temp: %u °C, cc_onbat: %u, control_mirror %u, control_horn: %u, "
             "control_indicator: %u, control_panicMode: %u, control_unlock %u, control_lock %u",
             m_climate_control_temp, m_climate_control_on_battery, m_mirror_fold_in_requested,
             m_horn_requested, m_indicators_requested, m_panic_mode_requested, m_unlock_requested,
             m_lock_requested);
}

OvmsVehicle::vehicle_command_t OvmsVehicleVWeGolf::CommandWakeup() {
    ESP_LOGV(TAG, "Wakeup triggered");

    // Info: eGolf300 after sending only the heartbeat message ID:0x5A7 or the first message here
    // ID:0x17330301 one ECU with ID:0x5F5 is answering on the bus perhaps ID:0x66E and ID:0x6B5 too
    // Info: perhaps this could be used for another method to wakeup the car comf CAN perhaps
    // changing the settings is possible without weaking up everything

    if (!m_is_car_online) {
        ESP_LOGI(TAG, "Car is sleeping we are trying to wake it up");
        // Wake up the Bus //CLI: can can3 tx extended 0x17330301 0x40 0x00 0x01 0x1F 0x00 0x00 0x00
        // 0x00
        canbus* comfBus;
        comfBus = m_can3;
        uint8_t length = 8;
        uint8_t data[length];
        data[0] = 0x40;
        data[1] = 0x00;
        data[2] = 0x01;
        data[3] = 0x1F;
        data[4] = 0x00;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;
        comfBus->WriteExtended(0x17330301, length, data);
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_LOGV(TAG, "First message send ID: data 0->7");
        ESP_LOGV(TAG,
                 "First message send ID:0x17330301 data %02x %02x %02x %02x %02x %02x %02x %02x",
                 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

        vTaskDelay(pdMS_TO_TICKS(100));
        length = 8;
        // NM-Anmeldung mit der am Fahrzeug verifizierten Maske. Die frueheren
        // Nutzdaten (67 10 41 84 14 00 00 00) entsprachen nicht dem NM-Schema
        // dieses Busses; damit blieb die BAP-Schicht unten.
        SendNetworkManagement();
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_LOGV(TAG, "second message send ID: data 0->7");
        ESP_LOGV(TAG,
                 "second message send ID:0x1B000067 data %02x %02x %02x %02x %02x %02x %02x %02x",
                 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

        m_is_control_active = true;
        m_control_hold = VWEGOLF_OCU_HOLD_SECS;  // auto-release after the hold window
    } else {
        ESP_LOGI(TAG, "Wakeup not necessary car was online before");
    }
    return Success;
}

// Merkt die Profilabfrage vor. Gesendet wird sie erst, wenn der Bus wach und
// eingeschwungen ist -- ueber denselben Weg wie der Klimabefehl. Direkt auf einen
// schlafenden Bus zu senden verstopft die Sendewarteschlange dauerhaft.
void OvmsVehicleVWeGolf::QueueBapProfile() {
    m_profile_requested = true;
    m_prof_retries = 8;
    m_prof_wait = 0;
    m_prof_valid = false;
    bool was_asleep = !m_is_car_online;
    if (was_asleep) {
        ESP_LOGI(TAG, "Profile request queued, waking the car first");
        CommandWakeup();
    }
    m_is_control_active = true;
    m_control_hold = was_asleep ? VWEGOLF_OCU_WAKE_SECS : VWEGOLF_OCU_HOLD_SECS;
}

// Fordert die Profilliste an. Erst den Kanal oeffnen (die beiden Get-Anfragen
// auf die Standardeigenschaften, wie es auch das Werksgeraet tut), dann die
// Profilliste selbst. Antworten laufen ueber IncomingFrameCan3 auf.
void OvmsVehicleVWeGolf::RequestBapProfile() {
    canbus* comfBus = m_can3;
    if (comfBus->GetPowerMode() != On) {
        comfBus->SetPowerMode(On);
        comfBus->Start(CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    uint8_t data[2];
    data[0] = 0x19; data[1] = 0x42;
    comfBus->WriteExtended(VWEGOLF_BAP_REQ_ID, 2, data);
    vTaskDelay(pdMS_TO_TICKS(50));
    data[0] = 0x19; data[1] = 0x41;
    comfBus->WriteExtended(VWEGOLF_BAP_REQ_ID, 2, data);
    vTaskDelay(pdMS_TO_TICKS(50));
    data[0] = 0x19; data[1] = 0x59;
    comfBus->WriteExtended(VWEGOLF_BAP_REQ_ID, 2, data);
    ESP_LOGI(TAG, "BAP profile requested (19 59)");
}

// Wertet den zusammengesetzten Profilpuffer aus.
void OvmsVehicleVWeGolf::ParseBapProfile() {
    // Hexdump in Zeilen zu 16 Byte. Vorlage fuer das spaetere Zurueckschreiben.
    char line[64];
    for (uint16_t o = 0; o < m_prof_len; o += 16) {
        int n = 0;
        for (uint16_t i = o; i < o + 16 && i < m_prof_len; i++)
            n += snprintf(line + n, sizeof(line) - n, "%02x ", m_prof_buf[i]);
        ESP_LOGI(TAG, "BAP prof %03u: %s", o, line);
    }

    if (m_prof_len < VWEGOLF_PROF_MINLEN_LIST) {
        // Kurze Variante: einzelnes Profil, andere Byteanordnung. Nur melden.
        ESP_LOGI(TAG, "BAP short profile (%u bytes), offsets not applicable", m_prof_len);
        return;
    }
    if (m_prof_gap) {
        ESP_LOGW(TAG, "BAP profile has gaps -- values unreliable, do NOT use for write");
    }
    m_prof_valid = !m_prof_gap;
    uint8_t raw_current = m_prof_buf[VWEGOLF_PROF_OFS_CURRENT];
    uint8_t raw_soc = m_prof_buf[VWEGOLF_PROF_OFS_SOCLIMIT];
    uint8_t raw_temp = m_prof_buf[VWEGOLF_PROF_OFS_TEMP];
    float temp = (float)raw_temp / 10.0F + 10.0F;

    if (m_cc_temp) m_cc_temp->SetValue(temp);
    StandardMetrics.ms_v_charge_climit->SetValue(raw_current);
    StandardMetrics.ms_v_charge_limit_soc->SetValue(raw_soc);

    ESP_LOGI(TAG, "BAP profile: %u bytes, charge current %uA, SoC limit %u%%, climate target %.1f C",
             m_prof_len, raw_current, raw_soc, temp);
}

// Netzwerkmanagement des Komfort-CAN. Zwei Frames, wie sie das Werks-OCU
// sendet: die NM-Anmeldung mit voller Partial-Network-Anforderung und die
// zugehoerige Zustandsmeldung. Am Fahrzeug verifiziert -- damit wacht das
// Fahrzeug vollstaendig auf (alle acht Knoten inkl. 0x14, BAP-Kanal 0x25 offen)
// und die Klimatisierung laesst sich aus dem Tiefschlaf starten.
bool OvmsVehicleVWeGolf::SendNetworkManagement() {
    canbus* comfBus = m_can3;
    uint8_t data[8];

    // Der Komfort-CAN-Transceiver wird im Schlaf stromlos geschaltet. Ohne ihn
    // erreicht kein Weckframe die Leitung: Die Konsole meldet "Can bus is not
    // powered on", und saemtliche Zaehler bleiben auf null -- kein Interrupt,
    // kein Sendeversuch, nicht einmal ein Fehler. Also zuerst einschalten.
    if (comfBus->GetPowerMode() != On) {
        ESP_LOGI(TAG, "Powering up comfort CAN transceiver for wakeup");
        comfBus->SetPowerMode(On);
        // SetPowerMode(On) startet den Bus nur, wenn er zuvor nicht ganz
        // abgeschaltet war (mcp2515: "if (m_mode != CAN_MODE_OFF) Start(...)").
        // Nach einem echten Power-Off steht m_mode auf OFF -- dann muss der Bus
        // ausdruecklich gestartet werden, sonst quittiert der Treiber jeden
        // Sendeversuch mit "Cannot write can3 when not in ACTIVE mode".
        comfBus->Start(CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);
        vTaskDelay(pdMS_TO_TICKS(250));  // Anlaufzeit des MCP2515
    }

    // NM-Anmeldung unter EIGENER Knotenkennung. Aufbau wie bei allen Knoten
    // im Mitschnitt: d[0] = Knotenadresse, d[1] = 0x00, d[2..7] = Anforderung
    // der Teilnetze. Die Anforderung ist vom OCU uebernommen -- wir brauchen
    // dieselben Teilnetze wach, und sie sagt nichts darueber aus, wer wir sind.
    data[0] = VWEGOLF_NM_NODE;
    data[1] = 0x00;
    data[2] = 0x01;
    data[3] = 0x01;
    data[4] = 0x04;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    // Zustandsmeldung, eigener Puffer. Frueher teilte sie sich `data` mit der
    // Anmeldung -- seit dem zweiten Schuss je Sekunde ging dadurch im zweiten
    // Durchlauf die Anmeldung mit den Zustandsnutzdaten hinaus.
    // Byte 7 = 0x80 im eingeschwungenen Netz (0x00 nur waehrend des Hochlaufs).
    uint8_t state[8] = { 0x20, VWEGOLF_NM_NODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80 };
    // ESP_QUEUED ist normal: Der Hardwarepuffer ist belegt, der Frame geht
    // gleich darauf raus. Nur ESP_FAIL heisst, dass die Warteschlange voll ist
    // und der Frame verworfen wurde -- erst dann lohnt eine Pause.
    bool ok = true;

    // Waehrend des Weckens zweimal je Sekunde senden. Das VW-NM erwartet
    // Meldungen etwa alle 500 ms; mit nur einer pro Sekunde wacht der Bus zwar
    // kurz auf, faellt aber zurueck in den Schlaf, bevor die BAP-Schicht oben
    // ist. Sobald das Fahrzeug antwortet, genuegt eine Meldung pro Durchlauf.
    // Zwei Schuesse je Sekunde, also ~500 ms Abstand -- so taktet auch das
    // Werks-OCU im Mitschnitt. Mit 1000 ms (ein Schuss) blieb die Profilliste
    // aus, mit ~450 ms ueber die Konsole kam sie. Der wache Fall war bisher
    // ausgenommen; das war vermutlich zu duenn.
    int bursts = 2;
    for (int burst = 0; burst < bursts; burst++) {
    if (comfBus->WriteExtended(VWEGOLF_NM_ID, 8, data) == ESP_FAIL) ok = false;
    vTaskDelay(pdMS_TO_TICKS(10));

    if (comfBus->WriteExtended(VWEGOLF_NM_STATE_ID, 8, state) == ESP_FAIL) ok = false;
    if (burst + 1 < bursts) vTaskDelay(pdMS_TO_TICKS(480));
    }
    return ok;
}

void OvmsVehicleVWeGolf::SendOcuHeartbeat() {
    uint8_t tmp_u8 = 0;

    canbus* comfBus;
    comfBus = m_can3;
    uint8_t length = 8;
    uint8_t data[length];
    length = 8;
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    // Spiegelanklappen
    if (m_mirror_fold_in_requested) {
        tmp_u8 = 1;
        data[5] = (((uint8_t)tmp_u8) << 7) & 0x80;
        m_mirror_fold_in_requested = false;
        ESP_LOGI(TAG, "Mirror fold in");
    }

    // Hupen
    if (m_horn_requested) {
        tmp_u8 = 1;
        data[6] = (((uint8_t)tmp_u8) >> 0) & 0x1;
        m_horn_requested = false;
        ESP_LOGI(TAG, "Horn");
    }

    // Door Lock //TODO there must be some vehicle specific identification send together with this
    // signal so not working OOB
    if (m_lock_requested >= 1) {
        tmp_u8 = 1;
        data[6] = (((uint8_t)tmp_u8) << 1) & 0x2;
        m_lock_requested = false;
        ESP_LOGI(TAG, "DoorLock");
    }

    // Door Unlock //TODO there must be some vehicle specific identification send together with this
    // signal so not working OOB
    if (m_unlock_requested) {
        tmp_u8 = 1;
        data[6] = (((uint8_t)tmp_u8) << 2) & 0x4;
        m_unlock_requested = false;
        ESP_LOGI(TAG, "DoorUnlock");
    }

    // Warnblinken
    if (m_indicators_requested) {
        tmp_u8 = 1;
        data[6] = (((uint8_t)tmp_u8) << 3) & 0x8;
        m_indicators_requested = false;
        ESP_LOGI(TAG, "Hazard lights");
    }

    // Panicalarm
    if (m_panic_mode_requested) {
        tmp_u8 = 1;
        data[6] = (((uint8_t)tmp_u8) << 4) & 0x10;
        m_panic_mode_requested = false;
        ESP_LOGI(TAG, "PanicAlarm!");
    }

    comfBus->WriteStandard(0x5A7, length, data);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGV(TAG, "Heartbeat send ID: data 0->7");
    ESP_LOGI(TAG, "FHeartbeat send ID:0x5A7 data %02x %02x %02x %02x %02x %02x %02x %02x", data[0],
             data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

    // Send a queued climate command now that the bus is awake and the heartbeat is
    // flowing (this only runs while online + control active).
    bool bus_settled = m_bap_ready && (m_clima_bap_seen || m_online_secs >= VWEGOLF_BUS_SETTLE_SECS);
    if ((m_climate_start_requested || m_climate_stop_requested) && !bus_settled) {
        ESP_LOGI(TAG, "Climate command queued, waiting for bus to settle (%us online, bap=%d, clima=%d)",
                 m_online_secs, m_bap_ready, m_clima_bap_seen);
    }
    // Abfrage wiederholen, bis ein Profil da ist. Die ECU laesst sich Zeit --
    // im belegten Fall kam die Antwort erst 7 s nach der Anfrage, unmittelbar
    // nach dem Aufwachen gar nicht. 19 59 ist ein Get und veraendert nichts,
    // Wiederholen ist daher gefahrlos.
    if (m_profile_requested && bus_settled) {
        if (m_prof_valid) {
            m_profile_requested = false;
            m_prof_retries = 0;
        } else if (m_prof_wait > 0) {
            m_prof_wait--;
            m_control_hold = VWEGOLF_OCU_HOLD_SECS;
        } else if (m_prof_retries > 0) {
            m_prof_retries--;
            m_prof_wait = 5;
            ESP_LOGI(TAG, "BAP profile: attempt %u", 8 - m_prof_retries);
            RequestBapProfile();
            m_control_hold = VWEGOLF_OCU_HOLD_SECS;
        } else {
            ESP_LOGW(TAG, "BAP profile: no answer after 8 attempts, giving up");
            m_profile_requested = false;
        }
    }
    if ((m_climate_start_requested || m_climate_stop_requested) && bus_settled) {
        bool enable = m_climate_start_requested;
        m_climate_start_requested = false;
        m_climate_stop_requested = false;
        SendClimateControl(enable);
        m_control_hold = VWEGOLF_OCU_HOLD_SECS;  // full grace after the command is sent
    }
}

void OvmsVehicleVWeGolf::SendClimateControl(bool enable) {
    // Start/stop climate on CAN 0x17332501 (LSG 0x25 BatteryControl), reproducing the
    // sequence a real controller sends: BAP channel handshake (19 42 / 19 41), then a
    // profile-arm write on func 0x959, then the actual immediate-start trigger on func
    // 0x958. The trigger is the frame that starts conditioning — arming alone does
    // nothing. Called only after the controller has been reset to error-active
    // (retries on), so frames deliver reliably on the busy comfort bus.
    //
    // BAP wire func = 0x940 + propId: func 0x959 = SetBatteryControlProfileList
    // (propId 0x19, "arm"); func 0x958 = SetBatteryControlImmediately (propId 0x18,
    // body {profileId, controlFlag}). Header nibble 2 = SET (a status reply from the
    // BCU would be nibble 4, e.g. 49 58). Selector 0x22 = start, 0x23 = stop.
    canbus* comfBus = m_can3;
    uint8_t data[8];

    // BAP channel open handshake.
    data[0] = 0x19; data[1] = 0x42;
    comfBus->WriteExtended(0x17332501, 2, data);
    vTaskDelay(pdMS_TO_TICKS(10));
    data[0] = 0x19; data[1] = 0x41;
    comfBus->WriteExtended(0x17332501, 2, data);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Profile-arm, frame 1 (BAP long-message start): 80 08 29 59 <sel> 06 00 01
    data[0] = 0x80;                  // BAP long-message start; payload length in d[1]
    data[1] = 0x08;                  // payload length = 8
    data[2] = 0x29;                  // SET (nibble 2) | func hi
    data[3] = 0x59;                  // func 0x959 = SetBatteryControlProfileList (arm)
    data[4] = enable ? 0x22 : 0x23;  // selector: start / stop
    data[5] = 0x06;
    data[6] = 0x00;
    data[7] = 0x01;
    comfBus->WriteExtended(0x17332501, 8, data);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Profile-arm, frame 2 (continuation / final): C0 06 00 20 00
    data[0] = 0xC0;
    data[1] = 0x06;
    data[2] = 0x00;
    data[3] = 0x20;
    data[4] = 0x00;
    comfBus->WriteExtended(0x17332501, 5, data);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Trigger: SetBatteryControlImmediately (func 0x958, propId 0x18), body =
    // {profileId, controlFlag}. profileId 0 = global/"Optionen" profile (holds the
    // configured target temp); controlFlag 1 = start now, 0 = stop. This is the frame
    // that actually starts/stops conditioning.
    data[0] = 0x29;                  // SET (nibble 2) | func hi
    data[1] = 0x58;                  // func 0x958 = SetBatteryControlImmediately
    data[2] = 0x00;                  // profileId 0 = global profile
    data[3] = enable ? 0x01 : 0x00;  // controlFlag: start / stop
    comfBus->WriteExtended(0x17332501, 4, data);

    ESP_LOGI(TAG, "Climate %s command sent (0x17332501: arm sel=0x%02x + trigger 29 58 00 %02x)",
             enable ? "START" : "STOP", enable ? 0x22 : 0x23, enable ? 0x01 : 0x00);
}
