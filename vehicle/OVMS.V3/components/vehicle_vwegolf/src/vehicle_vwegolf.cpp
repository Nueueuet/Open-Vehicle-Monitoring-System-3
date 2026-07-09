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

#include "ovms_notify.h"            // MyNotify — user notifications for climate outcome
#include "egolf/battery_control.h"  // BatteryControl (LSG 0x25): climate command + status decode

#undef TAG
#define TAG "v-vwegolf"

OvmsVehicleVWeGolf::OvmsVehicleVWeGolf() {
    ESP_LOGI(TAG, "Start vehicle module: VW e-Golf");

    // init configs:
    MyConfig.RegisterParam("xvg", "VW e-Golf", true, true);

    // Regenerative-braking strength (numeric, cheap to transmit). Decoded from the
    // gear-selector frame 0x187 in IncomingFrameCan2. -1 until first seen in D/B.
    m_recup_level = MyMetrics.InitInt("xvg.v.recup", SM_STALE_MIN, -1);

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
        m_climate_wake_hold = 0;  // also drop any in-flight climate NM-wake bridge
        ESP_LOGI(TAG, "OCU keepalive stopped");
    });
    cmd_vweg->RegisterCommand("fold_mirrors", "Fold mirrors in",
                              [this](...) { CommandMirrorFoldIn(); });
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

    // BAP BatteryControl (LSG 0x25) FSG status on 0x17332510: reassemble the segmented
    // telegrams and read the authoritative climate on/off from the OperationMode status
    // reply (element "49 58 <flag>"). Only feed genuine KCAN frames — IncomingFrameCan2
    // forwards its frames here too, and feeding the bridged duplicates would corrupt the
    // single reassembly stream.
    if (p_frame->MsgID == bap::egolf::kCanIdStatus && p_frame->origin == m_can3) {
        // Any status frame here means the BCU's BAP layer is up and listening — this is the
        // readiness gate for the first climate send (Ticker1). The BCU broadcasts its status
        // unsolicited on wake, and boots ~1.5 s after the rest of the comfort domain, so this
        // is a far tighter/safer trigger than "any KCAN traffic".
        m_bcu_seen = true;
        bap::Element el;
        if (m_bap_asm.feed(p_frame->MsgID, d, p_frame->FIR.B.DLC, el) &&
            el.lsg == bap::egolf::kLsg) {
            if (el.opcode == bap::OP_ERROR) {
                // BCU rejected/failed a BatteryControl request (e.g. not plugged in, not
                // ready, invalid). Surface it — otherwise the Ticker1 loop would just
                // retry blindly. The retry window still bounds our attempts.
                m_climate_error = true;
                ESP_LOGW(TAG,
                         "BatteryControl BAP ERROR response (func 0x%02X) — request rejected/failed",
                         el.func);
            } else if (el.opcode == bap::OP_STATUS &&
                       el.func == bap::egolf::FUNC_OPERATION_MODE && el.bodyLen >= 1) {
                // OperationMode status echo "49 58 <flag>": authoritative climate on/off,
                // and the confirmation that our command landed.
                bool hvac_on = el.body[0] != 0;
                StandardMetrics.ms_v_env_hvac->SetValue(hvac_on);
                m_climate_confirmed = (hvac_on == m_climate_enable);
                m_climate_error = false;  // a good status supersedes an earlier transient error
                ESP_LOGI(TAG, "BatteryControl OperationMode status 49 58 %02x -> HVAC %s",
                         el.body[0], hvac_on ? "ON" : "OFF");
            }
        }
    }

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

            // Current: 12-bit, factor 1 A. The raw field is charge-positive; negate to
            // the OVMS convention (ms_v_bat_current is output=positive, i.e. discharge
            // positive / charge negative): I = 2047 - raw.
            tmp_u16 = ((uint16_t)(d[1] & 0xf0) >> 4) | ((uint16_t)(d[2]) << 4);
            tmp_f32 = 2047.0F - (float)tmp_u16;
            StandardMetrics.ms_v_bat_current->SetValue(tmp_f32);

            // Voltage: 12-bit, factor 0.25 V.
            tmp_u16 = ((uint16_t)(d[3])) | ((uint16_t)(d[4] & 0xf) << 8);
            tmp_f32 = ((float)tmp_u16) * 0.25F;
            StandardMetrics.ms_v_bat_voltage->SetValue(tmp_f32);

            // Power = V * I, following the output=positive current sign above
            // (ms_v_bat_power is output=positive: positive = driving, negative = charging).
            tmp_f32 = (StandardMetrics.ms_v_bat_voltage->AsFloat() *
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

    // Climate NM-wake bridge (independent of the OCU heartbeat above). Wakeup is NOT
    // instant: the transceivers come up in ms but the BCU needs ~1-2 s+ (more from deep
    // sleep) to bring its BAP layer up. So each second we (a) keep sending the spare-node
    // NmWake to wake and hold the cluster, and (b) once the BCU is heard (m_bcu_seen),
    // RE-SEND the BAP channel handshake + command every tick until the BCU echoes it
    // (m_climate_confirmed). Gating on the BCU's own status frame is the readiness probe —
    // no need to guess when it's ready. On confirmation we release; the BCU + cluster then
    // sustain their own NM for the session (no keepalive needed).
    if (m_climate_wake_hold > 0) {
        SendNmWake();  // ~1 Hz, within the AUTOSAR NM timeout, to wake and hold the cluster
        // Fire the BAP command as soon as the BCU is heard (m_bcu_seen), then re-send every
        // tick (1 Hz) until it echoes. Waiting for the BCU's own status rather than a fixed
        // cadence removes the ~1.3 s we otherwise waste sending before it has booted.
        if (!m_climate_confirmed && m_bcu_seen) {
            SendClimateControl(m_climate_enable);  // handshake + trigger; retried until confirmed
            m_climate_cmd_sent = true;
        }
        m_climate_wake_hold--;
        if (m_climate_confirmed && m_climate_wake_hold > 2)
            m_climate_wake_hold = 2;  // BCU confirmed (49 58 echo) — release soon, small grace
        if (m_climate_wake_hold == 0) {
            // Terminal outcome (fires once): release the NM. Success is confirmed by the
            // BCU echo and reflected in ms_v_env_hvac, so it needs no separate notification;
            // on failure we notify with the distinguished cause so a remote user knows what
            // happened rather than silently thinking it worked.
            if (m_climate_confirmed) {
                ESP_LOGI(TAG, "Climate %s confirmed — releasing NM, cluster self-sustains",
                         m_climate_enable ? "ON" : "OFF");
            } else {
                const char* reason =
                    m_climate_error   ? "Climate command rejected by the battery control unit"
                    : m_climate_tx_fail ? "Climate command failed: CAN transmit error (try 'can can3 reset')"
                                        : "Climate command: no response from ECU (timeout)";
                ESP_LOGE(TAG, "Climate %s FAILED — releasing NM: %s",
                         m_climate_enable ? "ON" : "OFF", reason);
                MyNotify.NotifyString("alert", "xvg.climate", reason);
            }
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
    // Non-colliding path (independent of the OCU 0x5A7 heartbeat): assert a spare-node
    // NM wake to bring up the comfort/EV cluster, then send the BAP climate command.
    // The wake is held only as a short bridge in Ticker1 — once the BCU accepts the
    // command it and the cluster sustain their own NM, so we release. No OCU
    // impersonation (which set OCU DTCs U0011/U1201).
    m_climate_enable = enable;
    m_climate_cmd_sent = false;
    m_climate_confirmed = false;
    m_climate_error = false;
    m_climate_tx_fail = false;
    m_bcu_seen = false;  // re-arm the readiness gate; the BCU re-announces within ~1 s on a
                         // warm bus (status ~1.9 Hz) and ~1.5 s from cold
    m_climate_wake_hold = VWEGOLF_CLIMATE_WAKE_SECS;
    SendNmWake();  // kick the wake now; Ticker1 sustains it and sends the BAP command
    return Success;
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
        data[0] = 0x67;  // source node identifier identity of the transmitter of the message
        data[1] = 0x10;  // could be anything
        data[2] = 0x41;  // 0-5 => State,  6 => eCall Car Wakeup
        data[3] = 0x84;  // 0-8 => eCall Wakeup
        data[4] = 0x14;
        data[5] = 0x00;
        data[6] = 0x00;
        data[7] = 0x00;
        comfBus->WriteExtended(0x1B000067, length, data);
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
    if (m_climate_start_requested || m_climate_stop_requested) {
        bool enable = m_climate_start_requested;
        m_climate_start_requested = false;
        m_climate_stop_requested = false;
        SendClimateControl(enable);
        m_control_hold = VWEGOLF_OCU_HOLD_SECS;  // full grace after the command is sent
    }
}

void OvmsVehicleVWeGolf::SendClimateControl(bool enable) {
    // Emit the BatteryControl climate command on 0x17332501 (LSG 0x25) via the BAP
    // transport (vendored bap-lib): the channel-open handshake, then the OperationMode
    // immediate trigger ("29 58 00 <flag>"). The trigger runs the global profile
    // (profileId 0), which already holds the configured target temp. The old
    // ProfilesArray "arm" write only set charge maxCurrent (irrelevant to climate) and
    // is documented as a no-op for starting conditioning, so it is omitted. The BCU
    // echoes "49 58 <flag>" on 0x17332510, decoded in IncomingFrameCan3 to confirm.
    canbus* comfBus = m_can3;
    auto sink = [comfBus](const uint8_t* frame, uint8_t dlc) -> bool {
        // ESP_OK = frame in a HW TX buffer; ESP_QUEUED = accepted into the driver's SW
        // TX queue (HW buffers busy — routine while the controller is error-passive right
        // after the NM wake, or when the bus is congested). Both mean the frame WILL be
        // transmitted in order, so both are success; only ESP_FAIL (SW queue overflow /
        // controller off / bus-off) is a real failure. Treating ESP_QUEUED as failure made
        // the atomic-handshake guard below abort before sending the trigger on a busy bus,
        // even though every frame was transmitting fine.
        esp_err_t r = comfBus->WriteExtended(bap::egolf::kCanIdCommand, dlc,
                                             const_cast<uint8_t*>(frame));
        vTaskDelay(pdMS_TO_TICKS(10));  // pace frames on the busy comfort bus
        return r == ESP_OK || r == ESP_QUEUED;
    };
    // BAP channel-open handshake: GET BapConfig (func 0x02, "19 42") + GetAll (func
    // 0x01, "19 41") open/sync the logical channel with the BCU (a cold BCU ignores the
    // trigger until the channel is up; a warm BCU skips it). Retried by Ticker1.
    bap::SendResult g1 = bap::sendElement(sink, bap::OP_GET, bap::egolf::kLsg, 0x02, nullptr, 0);
    bap::SendResult g2 = bap::sendElement(sink, bap::OP_GET, bap::egolf::kLsg, 0x01, nullptr, 0);
    // Atomic handshake: only fire the trigger if BOTH GETs were accepted by the controller
    // (queued counts — see sink). A genuinely rejected GET (ESP_FAIL: SW queue overflow /
    // controller off / bus-off) leaves the BCU's logical channel half-open, so it would
    // discard the trigger — bail and let Ticker1 re-send the whole handshake next tick.
    if (!g1.ok() || !g2.ok()) {
        m_climate_tx_fail = true;
        ESP_LOGW(TAG, "Climate handshake GET TX rejected (g1=%d g2=%d) — retrying whole handshake",
                 g1.ok(), g2.ok());
        return;
    }
    // Trigger: OperationMode immediate start/stop of the global profile (no arm).
    bap::SendResult r = bap::egolf::sendClimate(sink, enable);
    // Track CAN-layer failure: the sink returns false only on ESP_FAIL (SW queue overflow /
    // controller off / bus-off), which surfaces here as a non-Ok status. Distinguishes
    // "the bus/controller is broken" from "the ECU didn't reply".
    m_climate_tx_fail = !r.ok();
    if (m_climate_tx_fail)
        ESP_LOGW(TAG, "Climate BAP trigger CAN write FAILED (status %d, %u/%u frames) — bus/controller?",
                 (int)r.status, r.framesSent, bap::expectedFrames(2));
    else
        ESP_LOGI(TAG, "Climate %s BAP trigger sent to 0x%08x (%u frames)",
                 enable ? "START" : "STOP", (unsigned)bap::egolf::kCanIdCommand, r.framesSent);
}

void OvmsVehicleVWeGolf::SendNmWake() {
    // AUTOSAR CAN-NM wake from a SPARE (unused) node id (id = 0x1B000000 + node), so we
    // never impersonate the real OCU (node 0x67) — that duplicate-node clash set OCU
    // DTCs U0011/U1201. The payload requests the comfort/EV partial-network cluster:
    //   byte0 = source node id (spare)          byte1 = 0x10 CBV active-wakeup
    //   byte2 = 0x49 = 0x40 charge | 0x08 climate PNC | 0x01 comfort baseline
    //   byte3 = 0x85 = 0x84 (observed wake request) | 0x01 (Climatronic 0x46 PNC)
    //   byte4 = 0x14 (observed wake request bits)
    // The Climatronic (0x46) PNC bits (byte2 0x08, byte3 0x01) are firmware- and
    // wire-confirmed; the 0x40/0x84/0x14 bits replay the observed remote-service wake.
    uint8_t data[8] = {VWEGOLF_NM_WAKE_NODE, 0x10, 0x49, 0x85, 0x14, 0x00, 0x00, 0x00};
    m_can3->WriteExtended(0x1B000000u | VWEGOLF_NM_WAKE_NODE, 8, data);
}
