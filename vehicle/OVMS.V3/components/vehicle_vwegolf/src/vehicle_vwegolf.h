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

// Car (poll) states
#define VWEGOLF_OFF 0       // All systems sleeping
#define VWEGOLF_AWAKE 1     // Base systems online
#define VWEGOLF_CHARGING 2  // Base systems online & car is charging the main battery
#define VWEGOLF_ON 3        // All systems online & car is drivable

// Seconds to keep the OCU heartbeat (0x5A7) alive after a wakeup/command before
// auto-releasing it — "deliver and release", so we don't sit on the bus or need a
// manual `xvg offline`. A still-queued command re-arms the window until delivered.
#define VWEGOLF_OCU_HOLD_SECS 15

// Netzwerkmanagement (AUTOSAR NM) des Komfort-CAN. 0x67 ist die Knotenadresse
// des OCU (im CAN-Mitschnitt des Fahrzeugs bestaetigt). Die Bytes 2..7 der
// NM-Anmeldung sind die Partial-Network-Anforderung; nur mit der vollen Maske
// faehrt die BAP-Anwendungsschicht inklusive Klima-Kanal 0x25 hoch. Mit der
// zuvor verwendeten Maske wachte zwar der Bus auf, aber weder Knoten 0x14 noch
// irgendein BAP-Kanal -- der Klimabefehl lief ins Leere.
#define VWEGOLF_NM_ID 0x1B000067UL
#define VWEGOLF_NM_STATE_ID 0x17F00067UL

// Nach dem Wecken aus dem Tiefschlaf dauert es rund 30 s, bis der Klima-Kanal
// offen ist (das Werks-OCU sendet seine erste Transaktion bei +35 s). Solange
// muss das Haltefenster reichen, sonst wird der Befehl verworfen.
#define VWEGOLF_OCU_WAKE_SECS 75

// Sekunden, die der Komfort-CAN nach dem Wecken laufen muss, bevor ein
// Klimabefehl abgesetzt wird. Die BAP-Frames erscheinen zwar sofort, die
// Klima-ECU nimmt Kommandos aber erst spaeter an.
#define VWEGOLF_BUS_SETTLE_SECS 15

// BAP-Kanal 0x25 (BatteryControl): Anfragen an die Klima-ECU, Antworten zurueck.
#define VWEGOLF_BAP_REQ_ID 0x17332501UL
#define VWEGOLF_BAP_RSP_ID 0x17332510UL
// Antwort auf 19 59 (Get SetBatteryControlProfileList). Nibble 4 = Status.
#define VWEGOLF_BAP_FUNC_PROFILE 0x4959
// Nur die unteren 12 Bit vergleichen: das obere Nibble ist die BAP-Operation
// (1 Get, 2 Set, 3 unaufgeforderter Status, 4 Antwort auf Get). Uns interessiert
// der Inhalt, nicht der Anlass.
#define VWEGOLF_BAP_FUNC_MASK 0x0FFF
#define VWEGOLF_BAP_PROP_PROFILE 0x959
// Ab dieser Laenge gilt der Puffer als vollstaendige Profilliste. Die kurze
// Variante (34 Byte) hat eine andere Byteanordnung -- Offsets nicht anwendbar.
#define VWEGOLF_PROF_MINLEN_LIST 100
// Offsets im Profil, am Fahrzeug verifiziert.
#define VWEGOLF_PROF_OFS_CURRENT 8
#define VWEGOLF_PROF_OFS_SOCLIMIT 9
#define VWEGOLF_PROF_OFS_TEMP 18
#define VWEGOLF_PROF_MAXLEN 160

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
    bool SupportsClimateControl() override { return true; }
    void SendOcuHeartbeat();
    // Liefert true, wenn die NM-Frames tatsaechlich auf die Leitung gingen.
    // false = Rueckstau in der Sendewarteschlange (Bus quittiert nicht).
    bool SendNetworkManagement();
    // Fordert die Profilliste von der Klima-ECU an (Kanal oeffnen, dann Get).
    void RequestBapProfile();
    // Merkt die Abfrage vor und weckt den Bus; gesendet wird aus SendOcuHeartbeat.
    void QueueBapProfile();
    // Wertet den zusammengesetzten Puffer aus und setzt die Metriken.
    void ParseBapProfile();
    void SendClimateControl(bool enable);

 protected:
    void Ticker1(uint32_t ticker) override;
    void Ticker10(uint32_t ticker) override;

 private:
    bool m_is_car_online = true;
    bool m_kl15_on = false;
    bool m_drivetrain_ready = false;
    uint8_t m_last_message_received = 255;
    uint8_t m_climate_control_temp = 19;
    bool m_climate_control_on_battery = false;
    bool m_climate_start_requested = false;
    bool m_climate_stop_requested = false;
    bool m_mirror_fold_in_requested = false;
    bool m_horn_requested = false;
    bool m_indicators_requested = false;
    bool m_panic_mode_requested = false;
    bool m_unlock_requested = false;
    bool m_lock_requested = false;
    bool m_is_control_active = false;
    // BAP-Anwendungsschicht laeuft (0x1733xxxx gesehen). Ohne sie nimmt die
    // Klima-ECU keine Kommandos an, auch wenn der Bus sonst wach ist.
    bool m_bap_ready = false;
    // Ticker-Pause nach einem Sendeversuch, der nur in der Warteschlange
    // gelandet ist. Verhindert, dass sich der Puffer zustaut und danach
    // ueberhaupt kein Frame mehr die Leitung erreicht.
    uint8_t m_nm_backoff = 0;
    uint8_t m_nm_stalled = 0;
    // Sekunden, die der Bus ununterbrochen antwortet (Ticker1).
    uint8_t m_online_secs = 0;
    // Kanal 0x25 der Klima-ECU hat sich gemeldet (0x17332510).
    bool m_clima_bap_seen = false;
    // Zusammensetzen der BAP-Langnachricht vom Kanal 0x25.
    uint8_t m_prof_buf[VWEGOLF_PROF_MAXLEN] = {};
    uint16_t m_prof_len = 0;    // erwartete Datenlaenge laut Kopf
    uint16_t m_prof_pos = 0;    // bereits eingesammelt
    uint8_t m_prof_cont = 0;    // naechster erwarteter Fortsetzungsmarker
    bool m_prof_active = false; // Zusammensetzen laeuft
    bool m_prof_valid = false;  // gueltiges Profil vorhanden
    bool m_prof_gap = false;    // Luecke gestopft -- nicht als Vorlage nutzen
    bool m_profile_requested = false;  // Abfrage vorgemerkt, wartet auf wachen Bus
    // Letzter verarbeiteter BAP-Frame, zum Erkennen der doppelten Zustellung.
    uint8_t m_prof_prev[8] = {};
    uint8_t m_prof_prev_dlc = 0;
    uint8_t m_prof_retries = 0;  // verbleibende Wiederholungen der Abfrage
    uint8_t m_prof_wait = 0;     // Sekunden bis zur naechsten Wiederholung
    OvmsMetricFloat* m_cc_temp = nullptr;
    uint8_t m_control_hold = 0;   // Ticker1 countdown; 0 => release the OCU heartbeat
    uint8_t m_vin_parts_received = 0;
    char m_vin_buf[18] = {};
    // Regenerative-braking strength, decoded from 0x187 (see IncomingFrameCan2).
    // The e-Golf's five regen levels as a 0..4 scale (least->most): D0 (coast) = 0,
    // D1 = 1, D2 = 2, D3 = 3, B = 4. -1 = N/A (not in gear D or B).
    OvmsMetricInt* m_recup_level = nullptr;
};

#endif  // #ifndef __VEHICLE_VWEG_H__
