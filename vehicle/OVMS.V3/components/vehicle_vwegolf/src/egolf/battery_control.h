/*
;    Project:       BAP protocol library
;    Subproject:    e-Golf BatteryControl (LSG 0x25) semantics
;
;    (C) 2026  Jona Wagner <jona@jonawagner.me>
;    SPDX-License-Identifier: MIT
*/

// Vehicle-specific layer: the Battery Control Unit (BCU) logical device,
// LSG 0x25, on the MQB comfort bus. It manages both charging AND EV climate
// preconditioning through "Battery Control Profiles" (charge locations). This
// layer sits on top of the generic bap:: transport and adds only the meaning
// of LSG 0x25's functions and payloads.
//
// Provenance is tagged per item: [WIRE] = confirmed on a real capture; [RE] =
// from reverse-engineered firmware / a third-party OCU implementation, not all
// independently capture-confirmed.

#ifndef BAP_EGOLF_BATTERY_CONTROL_H_
#define BAP_EGOLF_BATTERY_CONTROL_H_

#include <cstddef>
#include <cstdint>

#include "bap/bap.h"

namespace bap {
namespace egolf {

// 29-bit CAN ids. The LSG is embedded in the id: base(16) | lsg(8) | role(8),
// role 0x01 = ASG (display/command side), 0x10 = FSG (function ECU broadcast).
constexpr uint8_t  kLsg              = 0x25;  // [WIRE] BatteryControl
constexpr uint8_t  kSubsystemCommand = 0x01;  // [WIRE] BCU command/ASG-side low byte

// Composed from the framework CAN-id scheme, not hardcoded: change kLsg and both
// ids follow. Yields 0x17332501 (command) and 0x17332510 (status).
constexpr uint32_t kCanIdCommand = bap::mqbCanId(kLsg, kSubsystemCommand);
constexpr uint32_t kCanIdStatus  = bap::mqbCanId(kLsg, bap::kSubsystemFsg);

// LSG 0x25 function ids (the 6-bit `func` in the element header).
enum Func : uint8_t {
  FUNC_PLUG_STATE      = 0x10,  // [WIRE] plug/connection state (status "49 50")
  FUNC_CHARGE_STATE    = 0x11,  // [WIRE] charge status (status "49 51")
  FUNC_CLIMATE_STATE   = 0x12,  // [RE] climate status (firmware FID 0x12)
  FUNC_TIMER           = 0x14,  // [RE] SetBatteryControlTimer (departure timer)
  FUNC_OPERATION_MODE  = 0x18,  // [WIRE] execute/stop the immediate/global profile
  FUNC_PROFILES_ARRAY  = 0x19,  // [WIRE] battery-control profile array
  FUNC_POWER_PROVIDERS = 0x1A,  // [WIRE] status "49 5a"
};
// NB: an OCU reference enumerates 0x14=START_STOP_CHARGE / 0x15=START_STOP_CLIMATE,
// but those ids are never used there and the MIB2 firmware pins 0x14 =
// SetBatteryControlTimer; charge and climate are both executed via 0x18. They are
// intentionally not reproduced here.

// ProfileOperation bitfield (profile record byte [0]). [RE]
enum ProfileOperation : uint8_t {
  PO_CHARGING      = 0x01,  // charge at this location
  PO_CLIMATE       = 0x02,  // climatise (preconditioning)
  PO_ALLOW_BATTERY = 0x04,  // climatise without external supply (use HV battery)
};

// ProfileOperation2 bitfield (profile record byte [1]). [RE]
enum ProfileOperation2 : uint8_t {
  PO2_WINDOW_HEATER_FRONT = 0x01,
  PO2_WINDOW_HEATER_REAR  = 0x02,
  PO2_PARK_HEATER         = 0x04,
  PO2_PARK_HEATER_AUTO    = 0x08,
};

// ---------------------------------------------------------------------------
// Temperature encoding
// ---------------------------------------------------------------------------
// Profile target temperature is a single byte. [WIRE] confirmed at profile
// record offset [12]: captures show 0x78 = 22 degC and 0x64 = 20 degC.
//
// Integer API (preferred on the MCU -- no floating point): temperature in
// deci-degrees C (tenths), so 22.0 degC == 220.
//     raw = deciDegC - 100   (== degC*10 - 100)   inverse: deciDegC = raw + 100
inline uint8_t tempToRawDeci(int deciDegC) {
  int r = deciDegC - 100;
  if (r < 0) r = 0;
  if (r > 255) r = 255;
  return (uint8_t)r;
}
inline int rawToDeciDegC(uint8_t raw) { return (int)raw + 100; }

// Floating-point convenience -- HOST/UI ONLY. Avoid on the CAN task/ISR:
// touching float pins the Xtensa FPU context across FreeRTOS task switches, and
// float in an ISR is unsupported. The decode paths never call these; they keep
// the raw byte and callers convert with rawToDeciDegC() where FP is free.
inline uint8_t tempToRaw(float degC) {
  return tempToRawDeci((int)(degC * 10.0f + (degC < 0 ? -0.5f : 0.5f)));
}
inline float rawToTemp(uint8_t raw) { return rawToDeciDegC(raw) / 10.0f; }

// ---------------------------------------------------------------------------
// OperationMode (0x18) -- execute / stop
// ---------------------------------------------------------------------------
// Two-byte body. PROVEN on the wire (command id 0x17332501): {0x00,0x01} starts
// the immediate/global profile (profile 0), {0x00,0x00} stops it.
//
// The byte model for NON-global (timer) profiles is UNRESOLVED -- two RE sources
// disagree and no capture exercises it:
//   * OCU firmware:  {0x00, bitmap}    bit0=profile0, bits 1..3=timer 1..3
//   * MIB2 firmware: {profileId, ctl}  byte0 selects the profile, byte1 = 0/1
// Both collapse to {0x00,0x01}/{0x00,0x00} for the global case, which is all the
// captures show. We expose only the proven global start/stop plus a raw builder,
// and deliberately do not bake in either timer model.
inline void buildOperationModeRaw(uint8_t (&body)[2], uint8_t b0, uint8_t b1) {
  body[0] = b0;
  body[1] = b1;
}
inline void buildClimateStart(uint8_t (&body)[2]) { buildOperationModeRaw(body, 0x00, 0x01); }
inline void buildClimateStop(uint8_t (&body)[2])  { buildOperationModeRaw(body, 0x00, 0x00); }

// ---------------------------------------------------------------------------
// Battery Control Profile record (full, RecordAddr = 0)
// ---------------------------------------------------------------------------
// Byte layout of one profile record. The record LENGTH (20 fixed + name) and the
// [WIRE]-marked offsets are confirmed against real captured profile dumps; the
// [RE]-marked middle fields read as padding (0xFF/0x00) in every capture, so
// their positions fit but their semantics rest on the RE sources only.
//   [0]  operation (ProfileOperation bits)      [WIRE]
//   [1]  operation2 (ProfileOperation2 bits)    [WIRE]
//   [2]  maxCurrent (amps)                       [WIRE]
//   [3]  minChargeLevel (%)                       [WIRE]
//   [4-5] minRange (LE u16)                        [RE]
//   [6]  targetChargeLevel (%)                    [WIRE]
//   [7]  targetChargeDuration                       [RE]
//   [8-9] targetChargeRange (LE u16)                [RE]
//   [10] unitRange                                  [RE]
//   [11] rangeCalculationSetup                      [RE]
//   [12] temperature (raw; see tempToRaw)          [WIRE]
//   [13] temperatureUnit                            [RE]
//   [14] leadTime                                   [RE]
//   [15] holdingTimePlug                            [RE]
//   [16] holdingTimeBattery                         [RE]
//   [17-18] providerDataId (LE u16)                 [RE]
//   [19] nameLength                                [WIRE]
//   [20..] name (ASCII, nameLength bytes)          [WIRE]
constexpr uint8_t kProfileFixedLen = 20;   // fixed fields before the name
constexpr uint8_t kProfileMaxName  = 24;   // longest observed name (~17) + margin;
                                           // longer names decode truncated (nameTruncated)

struct Profile {
  uint8_t operation = 0;
  uint8_t operation2 = 0;
  uint8_t maxCurrent = 0;
  uint8_t minChargeLevel = 0;
  uint16_t minRange = 0;
  uint8_t targetChargeLevel = 0;
  uint8_t targetChargeDuration = 0;
  uint16_t targetChargeRange = 0;
  uint8_t unitRange = 0;
  uint8_t rangeCalculationSetup = 0;
  uint8_t temperatureRaw = 0;
  uint8_t temperatureUnit = 0;
  uint8_t leadTime = 0;
  uint8_t holdingTimePlug = 0;
  uint8_t holdingTimeBattery = 0;
  uint16_t providerDataId = 0;
  uint8_t nameLen = 0;
  char name[kProfileMaxName + 1] = {0};  // NUL-terminated for convenience
  bool nameTruncated = false;  // true if the on-wire name exceeded kProfileMaxName
  uint16_t position = 0;  // array position index (from a PosTransmit STATUS array)
};

// Encode a full profile record into `out` (capacity `cap`). Returns the number
// of bytes written (20 + nameLen), or 0 if it would not fit.
size_t encodeProfile(uint8_t* out, size_t cap, const Profile& p);

// Decode one full profile record (20 + nameLen bytes). `len` must cover the
// whole record. Returns false if the record is too short or its declared name
// runs past `len`; on false, `out` is zero-initialized and must not be used.
bool decodeProfile(const uint8_t* rec, uint16_t len, Profile& out);

// Outcome of decodeProfileArray, so a caller can tell a clean decode from a
// short one. `count` profiles were written; `declared` is the header's element
// count. Exactly one of the flags may be set when count < declared.
struct ArrayResult {
  uint16_t count = 0;        // profiles written to out[]
  uint16_t declared = 0;     // elementCount from the array header
  bool complete = false;     // decoded all `declared` records
  bool truncated = false;    // stopped because out[] (maxOut) filled first
  bool malformed = false;    // stopped on a bad record / header (or header too short)
};

// ---------------------------------------------------------------------------
// Profile array (0x19) STATUS decode
// ---------------------------------------------------------------------------
// A BCU status broadcast on FUNC_PROFILES_ARRAY (element header "49 59", from
// 0x17332510) carries a BAP array of profile records. The on-wire framing,
// confirmed against real dumps and the RE sources, is:
//
//   [0] arrayId (0x29 = full list / 0x2A = changed-profile notification)
//   [1] totalElementsInList
//   [2] flags: [LargeIdx:1][PosTransmit:1][Backward:1][Shift:1][RecordAddr:4]
//   [3] startIndex     (2 bytes, LE, if LargeIdx=1)
//   [4] elementCount   (2 bytes, LE, if LargeIdx=1)
//   then, per element: [position (1 or 2 bytes) if PosTransmit] + record
//
// This decoder handles RecordAddr=0 (the 20-byte full record). Only the full
// format has been observed; compact (RecordAddr=6, 4 bytes) elements are skipped.
// The global "Optionen"/immediate profile is element 0; it holds the climate
// target temperature and global charge limits.
//
// Fills up to `maxOut` Profile entries and returns the count. Pass `result` to
// learn whether the decode was complete, truncated by maxOut, or stopped on a
// malformed record (otherwise a short count is indistinguishable from success).
size_t decodeProfileArray(const uint8_t* body, uint16_t bodyLen,
                          Profile* out, size_t maxOut, ArrayResult* result = nullptr);

// ---------------------------------------------------------------------------
// Climate commands
// ---------------------------------------------------------------------------
// Selector byte of the ProfilesArray (0x19) climate telegram. [WIRE] the whole
// telegram is the literal captured start/stop command; only the selector
// (0x22/0x23) is firmly understood -- the trailing compact-record bytes are the
// observed payload with partly [RE] semantics.
constexpr uint8_t kClimateSelectStart = 0x22;
constexpr uint8_t kClimateSelectStop  = 0x23;

// Fill the 8-byte ProfilesArray (0x19) climate-select body exactly as captured:
// selector + a compact profile-0 record (maxCurrent 0x20).
inline void buildClimateProfileSelect(uint8_t (&body)[8], bool start) {
  const uint8_t tail[7] = {0x06, 0x00, 0x01, 0x06, 0x00, 0x20, 0x00};
  body[0] = start ? kClimateSelectStart : kClimateSelectStop;
  for (uint8_t i = 0; i < 7; i++) body[1 + i] = tail[i];
}

// Start/stop the immediate/global profile via OperationMode (0x18) ALONE:
// `on` -> "29 58 00 01", off -> "29 58 00 00".
template <typename Sink>
bap::SendResult sendClimate(Sink&& sink, bool on) {
  uint8_t body[2];
  if (on) buildClimateStart(body); else buildClimateStop(body);
  return bap::sendElement(sink, OP_SET_GET, kLsg, FUNC_OPERATION_MODE, body, 2);
}

// Full climate command as seen on the wire: the ProfilesArray (0x19) select
// telegram THEN the OperationMode (0x18). Sends the array telegram first and
// proceeds only if it left the sink cleanly; returns the OperationMode result,
// or the array telegram's result if that one failed.
template <typename Sink>
bap::SendResult sendClimateSequence(Sink&& sink, bool on) {
  uint8_t sel[8];
  buildClimateProfileSelect(sel, on);
  bap::SendResult r = bap::sendElement(sink, OP_SET_GET, kLsg, FUNC_PROFILES_ARRAY, sel, 8);
  if (!r.ok()) return r;
  return sendClimate(sink, on);
}

}  // namespace egolf
}  // namespace bap

#endif  // BAP_EGOLF_BATTERY_CONTROL_H_
