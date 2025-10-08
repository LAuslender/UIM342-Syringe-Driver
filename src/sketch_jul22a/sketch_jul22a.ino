#include "UirSimpleCAN.h"

#define __NEED_ACK 0x80
#define __BG 0x16
#define __MS 0x11
#define __SP 0x1E
#define __PR 0x1F
#define __MO 0x15

struct can_frame Canmsg;
MCP2515 mcp2515(10);
SimpleCAN simplecan;

static const uint8_t PID = 4;  // Producer ID (Arduino)
static const uint8_t CID = 5;  // Consumer ID (UIM342)

int32_t parse_i32_le(const uint8_t *d) {
  return (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
}

// Function to read/print abs. position with MS[1]
int32_t read_abs_pos() {
  uint8_t sub = 0x01; // MS[1]: speed + absolute position
  can_frame resp = simplecan.CommandSet(PID, CID, (__MS | __NEED_ACK), 1, &sub);
  return parse_i32_le(&resp.data[4]); // d7:d4 = PA (LSB first)
}

// Function that polls MS[0] until the PTP "in-position" flag is set, then reads back
// the new abs. position with MS[1]
bool poll_in_position(uint32_t timeout_ms, int32_t *outRelPos) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t sub = 0x00; // MS[0]: flags + relative position
    can_frame resp = simplecan.CommandSet(PID, CID, (__MS | __NEED_ACK), 1, &sub);
    uint8_t d2 = resp.data[2];
    bool PAIF = (d2 & 0b00000010) != 0; // d2.bit1 = PAIF (in position)
    if (outRelPos) *outRelPos = parse_i32_le(&resp.data[4]); // d7:d4 = PR
    if (PAIF) return true;
    delay(5);
  }
  return false;
}

void setup() {
  // Serial, CAN, and UIM347XS Initilization
  Serial.begin(2000000);
  while (!Serial) {
    Serial.print("No serial connection.");
  }

  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();


  // MO=1 (Motor ON)
  uint8_t data1[1] = {0x01};
  simplecan.CommandSet(PID, CID, (__MO | __NEED_ACK), 1, data1);
  delay(200);

}

void loop() {

   // --- Read and print current ABS position (MS[1]) ---
  int32_t pa_before = read_abs_pos();
  Serial.print("ABS before = "); Serial.println(pa_before);


  // --- Build a PTP move: PR then SP then BG ---
  // Example: move +1000 pulses at 10000 pps
  const int32_t rel_move = 1000;
  uint8_t prData[4] = {
    (uint8_t)(rel_move & 0xFF),
    (uint8_t)((rel_move >> 8) & 0xFF),
    (uint8_t)((rel_move >> 16) & 0xFF),
    (uint8_t)((rel_move >> 24) & 0xFF)
  };


  // 1) PR (set relative position)
  simplecan.CommandSet(PID, CID, (__PR | __NEED_ACK), 4, prData);


  // 2) SP (set PTP speed) — must come AFTER PR/PA in PTP, per manual
  const int32_t sp_val = 10000; // pps
  uint8_t spData[4] = {
    (uint8_t)(sp_val & 0xFF),
    (uint8_t)((sp_val >> 8) & 0xFF),
    (uint8_t)((sp_val >> 16) & 0xFF),
    (uint8_t)((sp_val >> 24) & 0xFF)
  };


  simplecan.CommandSet(PID, CID, (__SP | __NEED_ACK), 4, spData);


  // 3) BG (begin motion)
  simplecan.CommandSet(PID, CID, (__BG | __NEED_ACK), 0, NULL);

  // --- Wait until in-position (PAIF set via MS[0]) ---
  int32_t pr_at_end = 0;
  bool inPos = poll_in_position(10000, &pr_at_end); // 10s timeout
  Serial.print("In-position? "); Serial.println(inPos ? "YES" : "NO");
  Serial.print("Rel pos at end (MS[0].PR) = "); Serial.println(pr_at_end);

  // --- Read and print ABS after move (MS[1]) ---
  int32_t pa_after = read_abs_pos();
  Serial.print("ABS after = "); Serial.println(pa_after);

  delay(5000);
}
