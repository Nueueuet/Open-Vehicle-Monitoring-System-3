/*
;    Project:       Open Vehicle Monitor System
;	 Subproject:    Integrate VW e-Golf
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

#ifndef __VEHICLE_VWEG_H__
#define __VEHICLE_VWEG_H__

#include <atomic>

#include "can.h"
#include "ovms_command.h"
#include "ovms_config.h"
#include "ovms_log.h"
#include "ovms_metrics.h"
#include "vehicle.h"

// Vendored BAP protocol library (src/bap): comfort-bus BAP transport / reassembler.
#include "bap/bap.h"

// Car (poll) states
#define VWEGOLF_OFF 0       // All systems sleeping
#define VWEGOLF_AWAKE 1     // Base systems online
#define VWEGOLF_CHARGING 2  // Base systems online & car is charging the main battery
#define VWEGOLF_ON 3        // All systems online & car is drivable

// Seconds to keep the OCU heartbeat (0x5A7) alive after a wakeup/command before
// auto-releasing it — "deliver and release", so we don't sit on the bus or need a
// manual `xvg offline`. A still-queued command re-arms the window until delivered.
#define VWEGOLF_OCU_HOLD_SECS 15

// Climate uses a dedicated, non-colliding wake instead of impersonating the real
// OCU: an AUTOSAR-NM frame from a SPARE (unused) node id requests the comfort/EV
// partial-network cluster. It is held only as a short bridge — once the BCU accepts
// the BAP climate command, it and the cluster sustain their own NM, so we release.
// (Impersonating the real OCU node 0x67 caused OCU DTCs U0011/U1201.)
#define VWEGOLF_NM_WAKE_NODE       0x7D  // spare node id (verified unused in captures)
#define VWEGOLF_CLIMATE_WAKE_SECS  20    // max seconds to sustain the NM-wake bridge
                                         // The BAP command is (re)sent from Ticker1 (1 Hz) as
                                         // soon as the BCU is heard (its 0x17332510 status) and
                                         // then every second until it echoes the command. The
                                         // whole comfort domain floods the bus within ms of the
                                         // wake, but the BCU itself boots ~1.5 s later — gating
                                         // on its own status frame (not generic traffic) avoids
                                         // firing into an ECU that isn't listening yet.

class OvmsVehicleVWeGolf : public OvmsVehicle {
 public:
    OvmsVehicleVWeGolf();
    ~OvmsVehicleVWeGolf();

    void IncomingFrameCan2(CAN_frame_t* p_frame) override;
    void IncomingFrameCan3(CAN_frame_t* p_frame) override;

    vehicle_command_t CommandHorn();
    vehicle_command_t CommandPanic();
    vehicle_command_t CommandIndicators();
    vehicle_command_t CommandMirrorFoldIn();
    vehicle_command_t CommandLock(const char* pin) override;
    vehicle_command_t CommandUnlock(const char* pin) override;
    vehicle_command_t CommandWakeup() override;
    vehicle_command_t CommandClimateControl(bool enable) override;
    void SendOcuHeartbeat();
    void SendClimateControl(bool enable);
    void SendNmWake();

 protected:
    void Ticker1(uint32_t ticker) override;

 private:
    bool m_is_car_online = true;
    bool m_kl15_on = false;
    bool m_drivetrain_ready = false;
    uint8_t m_last_message_received = 255;
    bool m_climate_start_requested = false;
    bool m_climate_stop_requested = false;
    bool m_mirror_fold_in_requested = false;
    bool m_horn_requested = false;
    bool m_indicators_requested = false;
    bool m_panic_mode_requested = false;
    bool m_unlock_requested = false;
    bool m_lock_requested = false;
    bool m_is_control_active = false;
    uint8_t m_control_hold = 0;   // Ticker1 countdown; 0 => release the OCU heartbeat
    // Climate: non-colliding NM-wake + BAP command path, independent of the OCU
    // heartbeat above (see CommandClimateControl / SendNmWake / SendClimateControl).
    bap::AssemblerSetT<2, 96, 4> m_bap_asm;  // reassembles BatteryControl FSG status (0x17332510)
    bool m_climate_enable = false;     // requested on/off for the in-flight command
    bool m_climate_cmd_sent = false;   // BAP command already emitted this wake cycle
    bool m_bcu_seen = false;           // BCU (0x17332510) heard since this command's wake —
                                       // readiness gate for the first BAP send
    bool m_climate_confirmed = false;  // BCU echoed 49 58 <flag> matching the request
    bool m_climate_error = false;      // BCU returned a BAP ERROR response to the request
    bool m_climate_tx_fail = false;    // last command's CAN write failed (bus/controller)
    uint8_t m_climate_wake_hold = 0;   // Ticker1 countdown for the NM-wake bridge
    uint8_t m_vin_parts_received = 0;
    char m_vin_buf[18] = {};
    // Regenerative-braking strength, decoded from 0x187 (see IncomingFrameCan2).
    // The e-Golf's five regen levels as a 0..4 scale (least->most): D0 (coast) = 0,
    // D1 = 1, D2 = 2, D3 = 3, B = 4. -1 = N/A (not in gear D or B).
    OvmsMetricInt* m_recup_level = nullptr;
};

#endif  // #ifndef __VEHICLE_VWEG_H__
