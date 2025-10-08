/*
  SyringeDriver_StateMachine_CAL_Primer_BL_BTNcal.ino + OLED HUD
  - Arduino Nano ESP32 + MCP2515 (16 MHz) @ 500 kbps
  - Buttons ACTIVE-LOW with internal pull-ups; wire to GND: UP=A0, DOWN=A1, CONFIRM=A2
  - OLED: SSD1306 128x32 I2C (Frienda 0.91"), VCC=3.3V, GND=GND, SDA=A4, SCL=A5
*/

#include "UirSimpleCAN.h"
#include <SPI.h>
#include <math.h>

// -------- OLED adds --------
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET   -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
uint8_t oled_addr = 0x3C;  // will auto-try 0x3D if needed

// ---------- Control words ----------
#define NEED_ACK 0x80
#define CW_MO 0x15
#define CW_BG 0x16
#define CW_ST 0x17
#define CW_MS 0x11
#define CW_SP 0x1E
#define CW_PR 0x1F
#define CW_ER 0x0F
#define CW_BL 0x2D   // Backlash compensation (pulses)

// ---------- CAN roles & MCP ----------
static const uint8_t PID = 4;   // Arduino
static const uint8_t CID = 5;   // UIM342
MCP2515  mcp2515(10);           // MCP2515 CS = D10
SimpleCAN simplecan;

// ---------- Buttons (ACTIVE-LOW; pressed = LOW) ----------
const int BTN_UP      = A0;
const int BTN_DOWN    = A1;
const int BTN_CONFIRM = A2;

// ---------- Debounce (falling edges) ----------
struct DebEdge {
  bool rawLast = HIGH;
  bool stable  = HIGH;
  uint32_t lastChange = 0;
};
const uint16_t DEBOUNCE_MS = 60;

inline bool fellEdge(DebEdge &d, bool raw) {
  if (raw != d.rawLast) { d.lastChange = millis(); d.rawLast = raw; }
  if ((millis() - d.lastChange) > DEBOUNCE_MS) {
    if (raw != d.stable) {
      d.stable = raw;
      return (d.stable == LOW);   // we only care about HIGH->LOW (press)
    }
  }
  return false;
}

DebEdge dbU, dbD, dbO;

// ---------- Gating to avoid double taps ----------
uint32_t gateUntil=0;
inline void gate(uint16_t ms){ gateUntil = millis() + ms; }
inline bool ready(){ return (int32_t)(millis() - gateUntil) >= 0; }

// ---------- Mechanics & calibration ----------
float    PULSES_PER_ML = 1600.0f;  // refine via CAL: Total
uint32_t BL_PULSES     = 0;        // set via CAL: Backlash

const int    DIR_ASPIRATE  = -1;  // pull plunger back
const int    DIR_DISPENSE  = +1;  // push plunger forward

inline int32_t ml2p(float ml,int dir){
  double p = (double)ml * (double)PULSES_PER_ML * (double)dir;
  if (p >  2147483647.0) p =  2147483647.0;
  if (p < -2147483648.0) p = -2147483648.0;
  return (int32_t)llround(p);
}

// ---------- Primer ----------
const bool    ENABLE_PRIMER      = true;
const float   PRIMER_ML          = 0.5f;   // tiny purge
const int32_t PRIMER_SPEED_PPS   = 10000;
const uint16_t PRIMER_SETTLE_MS  = 60;

// ---------- Motion parameters ----------
const int32_t DEFAULT_SPEED_PPS  = 10000;
const uint32_t MOVE_TIMEOUT_MS   = 20000;

// ---------- Helpers ----------
static inline void le32(int32_t v, uint8_t out[4]){
  out[0]=(uint8_t)(v & 0xFF);
  out[1]=(uint8_t)((v>>8)&0xFF);
  out[2]=(uint8_t)((v>>16)&0xFF);
  out[3]=(uint8_t)((v>>24)&0xFF);
}

bool BL_set(uint32_t pulses) {
  uint8_t d[4];
  d[0] = (uint8_t)(pulses & 0xFF);
  d[1] = (uint8_t)((pulses >> 8) & 0xFF);
  d[2] = (uint8_t)((pulses >> 16) & 0xFF);
  d[3] = (uint8_t)((pulses >> 24) & 0xFF);
  can_frame r = simplecan.CommandSet(PID, CID, (CW_BL|NEED_ACK), 4, d);
  return (r.can_dlc >= 1);
}

// MS[0] sub=0: flags + relative position
bool MS0_get(uint8_t &d1,uint8_t &d2,int32_t &relPos){
  uint8_t sub=0x00;
  can_frame r = simplecan.CommandSet(PID, CID, (CW_MS|NEED_ACK), 1, &sub);
  if (r.can_dlc < 8 || r.data[0] != sub) return false;
  d1 = r.data[1]; d2 = r.data[2];
  relPos = (int32_t)((uint32_t)r.data[4] | ((uint32_t)r.data[5]<<8) | ((uint32_t)r.data[6]<<16) | ((uint32_t)r.data[7]<<24));
  return true;
}

bool MS0_clear(){
  uint8_t d[2] = {0x00, 0x00};
  (void)simplecan.CommandSet(PID, CID, (CW_MS|NEED_ACK), 2, d);
  return true;
}

// ER clear helpers
inline void ER_clear_idx(uint8_t idx){
  uint8_t d[2] = { idx, 0x00 };
  (void)simplecan.CommandSet(PID, CID, (CW_ER|NEED_ACK), 2, d);
}

// MO / BG / ST
inline void MO_set(uint8_t onOff){
  uint8_t d[1] = { onOff ? 1 : 0 };
  (void)simplecan.CommandSet(PID, CID, (CW_MO|NEED_ACK), 1, d);
}
inline void BG_begin(){ (void)simplecan.CommandSet(PID, CID, (CW_BG|NEED_ACK), 0, NULL); }
inline void ST_stop(){  (void)simplecan.CommandSet(PID, CID, (CW_ST|NEED_ACK), 0, NULL); }

// PR / SP (relaxed ACK checks)
bool PR_set_relaxed(int32_t rel){
  uint8_t d[4]; le32(rel, d);
  can_frame r = simplecan.CommandSet(PID, CID, (CW_PR|NEED_ACK), 4, d);
  return (r.can_dlc >= 1);
}
bool SP_set_relaxed(int32_t pps){
  uint8_t d[4]; le32(pps, d);
  can_frame r = simplecan.CommandSet(PID, CID, (CW_SP|NEED_ACK), 4, d);
  return (r.can_dlc >= 1);
}

// Wait for PAIF (bit1) or timeout
bool waitPAIF(uint32_t ms){
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    uint8_t d1=0, d2=0; int32_t rp=0;
    if (MS0_get(d1,d2,rp)) {
      bool PAIF = (d2 & (1<<1)) != 0;
      if (PAIF) return true;
    }
    delay(5);
  }
  return false;
}

// Recovery: stop, clear flags & errors, freewheel briefly, re-enable
void recover_to_safe_idle() {
  Serial.println("Recovery: ST → clear MS/ER → MO=0 → MO=1");
  ST_stop(); delay(50);
  MS0_clear(); delay(30);
  ER_clear_idx(10); ER_clear_idx(0); delay(30);
  MO_set(0); delay(150);
  MS0_clear(); ER_clear_idx(10); ER_clear_idx(0);
  MO_set(1); delay(150);
}

// One relative move (PR → SP → clear → BG → waitPAIF)
bool move_rel_steps(int32_t steps, int32_t speed_pps) {
  if (!PR_set_relaxed(steps)) { Serial.println("PR_set failed (ignored, PAIF decides)"); }
  if (!SP_set_relaxed(speed_pps)) { Serial.println("SP_set failed (ignored, PAIF decides)"); }
  MS0_clear(); delay(8);
  BG_begin();
  bool ok = waitPAIF(MOVE_TIMEOUT_MS);
  if (!ok) Serial.println("Move timeout (no PAIF).");
  MS0_clear();
  return ok;
}

// ---------- UI values ----------
int vol_mL = 1;                 // 1..20
int count  = 1;                 // 1..20
int remaining = 0;
int commanded_total_mL = 0;

// ---------- Calibration working vars ----------
int  cal_total_measured_tenths = 0;     // 0..200 => 0.0..20.0 mL
const float  BL_TEST_ML = 0.50f;
int  bl_measured_tenths = 0;            // 0..200 => 0.0..20.0 mL

// ---------- States ----------
enum State {
  STARTUP, INPUT_VOLUME, INPUT_COUNT, PREP_ASP, WAIT_ASP_CONFIRM,
  ASPIRATING, ASP_IDLE, DISPENSING, COMPLETE, ERROR_STATE,
  CAL_TOTAL, CAL_BL_WAIT, CAL_BL_ADJUST
};
State state = STARTUP;

// -------- OLED helpers --------
const char* stateLabel(State s) {
  switch (s) {
    case STARTUP:          return "STARTUP";
    case INPUT_VOLUME:     return "SET VOL";
    case INPUT_COUNT:      return "SET COUNT";
    case PREP_ASP:         return "PREP ASP";
    case WAIT_ASP_CONFIRM: return "CONFIRM ASP";
    case ASPIRATING:       return "ASPIRATING";
    case ASP_IDLE:         return "ASP IDLE";
    case DISPENSING:       return "DISPENSING";
    case COMPLETE:         return "COMPLETE";
    case ERROR_STATE:      return "ERROR";
    case CAL_TOTAL:        return "CAL TOTAL";
    case CAL_BL_WAIT:      return "CAL BL RUN";
    case CAL_BL_ADJUST:    return "CAL BL SET";
    default:               return "-";
  }
}


void oledRender() {
  static bool inited = false;
  if (!inited) {
    Wire.begin(A4, A5);          // explicit: SDA=A4, SCL=A5
    Wire.setClock(400000);
    if (!display.begin(SSD1306_SWITCHCAPVCC, oled_addr)) {
      oled_addr = 0x3D;
      (void)display.begin(SSD1306_SWITCHCAPVCC, oled_addr);
    }
    inited = true;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Row 0: State
  display.setCursor(0, 0);
  display.print("State: ");
  display.print(stateLabel(state));

  // Row 1: Vol / Count / Rem
  display.setCursor(0, 10);
  display.print("Vol:");
  display.print(vol_mL);
  display.print("mL  Cnt:");
  display.print(count);
  display.print("  Rem:");
  display.print(remaining);

  // Row 2: Steps/mL and BL
  display.setCursor(0, 20);
  display.print("SPM:");
  display.print((long)llround(PULSES_PER_ML));
  display.print("  BL:");
  display.print(BL_PULSES);

  display.display();
}

// -------- Existing prompt() now also refreshes OLED --------
void prompt() {
  switch (state) {
    case INPUT_VOLUME:
      Serial.print("Set dispense volume [1..20 mL]: "); Serial.println(vol_mL); break;
    case INPUT_COUNT:
      Serial.print("Set reservoir count [1..20]: "); Serial.println(count); break;
    case WAIT_ASP_CONFIRM:
      Serial.println("Place syringe in liquid, then press CONFIRM to aspirate."); break;
    case ASP_IDLE:
      Serial.println("Aspiration complete. CONFIRM=Dispense, UP=CAL Backlash, DOWN=CAL Total."); break;
    case COMPLETE:
      Serial.println("Workload complete. CONFIRM=New setup, UP/DOWN=CAL Total."); break;
    case ERROR_STATE:
      Serial.println("FAULT. Press CONFIRM to re-initialize."); break;
    case CAL_TOTAL:
      Serial.print("[CAL: Total] Enter measured total mL (0.1 steps): ");
      Serial.print(cal_total_measured_tenths / 10.0f, 1); Serial.println(" mL");
      break;
    case CAL_BL_WAIT:
      Serial.println("[CAL: Backlash] Running 0.50 mL test... measure result then press CONFIRM."); break;
    case CAL_BL_ADJUST:
      Serial.print("[CAL: Backlash] Enter measured test mL (0.1 steps): ");
      Serial.print(bl_measured_tenths / 10.0f, 1); Serial.println(" mL");
      break;
    default: break;
  }
  oledRender();  // <--- refresh screen on every UI change
}

void setup() {
  Serial.begin(2000000);
  while (!Serial) {}

  // Bring up I2C/OLED early with a splash (optional)
  Wire.begin(A4, A5);
  Wire.setClock(400000);
  (void)display.begin(SSD1306_SWITCHCAPVCC, oled_addr);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Syringe Driver");
  display.setCursor(0, 10);
  display.println("Init...");
  display.display();

  pinMode(BTN_UP,      INPUT_PULLUP);
  pinMode(BTN_DOWN,    INPUT_PULLUP);
  pinMode(BTN_CONFIRM, INPUT_PULLUP);

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();

  Serial.println("Power on.");
  state = STARTUP;
  gate(250);
}

void loop() {
  bool upFall   = ready() && fellEdge(dbU, digitalRead(BTN_UP));
  bool downFall = ready() && fellEdge(dbD, digitalRead(BTN_DOWN));
  bool okFall   = ready() && fellEdge(dbO, digitalRead(BTN_CONFIRM));

  switch (state) {
    case STARTUP: {
      Serial.println("Startup: safe clear + driver ON...");
      recover_to_safe_idle();   // includes MO=1
      if (BL_set(BL_PULSES)) {
        Serial.print("BL set to "); Serial.print(BL_PULSES); Serial.println(" pulses.");
      } else {
        Serial.println("BL set failed (continuing).");
      }
      Serial.println("Ready.");
      state = INPUT_VOLUME;
      prompt();
      gate(220);
    } break;

    case INPUT_VOLUME: {
      if (upFall   && vol_mL < 20) { vol_mL++; prompt(); gate(200); }
      if (downFall && vol_mL >  1) { vol_mL--; prompt(); gate(200); }
      if (okFall) { state = INPUT_COUNT; prompt(); gate(240); }
    } break;

    case INPUT_COUNT: {
      if (upFall   && count < 20) { count++; prompt(); gate(200); }
      if (downFall && count >  1) { count--; prompt(); gate(200); }
      if (okFall) {
        if (vol_mL * count <= 20) {
          remaining = count;
          commanded_total_mL = vol_mL * count;
          state = PREP_ASP;
        } else {
          Serial.print("Invalid: "); Serial.print(vol_mL);
          Serial.print(" mL x "); Serial.print(count);
          Serial.println(" > 20 mL capacity.");
          state = INPUT_VOLUME;
        }
        prompt();
        gate(240);
      }
    } break;

    case PREP_ASP: {
      Serial.print("Total to aspirate: "); Serial.print(commanded_total_mL); Serial.println(" mL");
      Serial.println("Prep done.");
      state = WAIT_ASP_CONFIRM;
      prompt();
      gate(240);
    } break;

    case WAIT_ASP_CONFIRM: {
      if (okFall) {
        (void)BL_set(BL_PULSES);

        bool didPrimer = false;
        if (ENABLE_PRIMER && PRIMER_ML > 0.0f) {
          int32_t psteps = ml2p(PRIMER_ML, DIR_DISPENSE);
          Serial.print("Primer: dispensing "); Serial.print(PRIMER_ML); Serial.println(" mL...");
          if (move_rel_steps(psteps, PRIMER_SPEED_PPS)) { didPrimer = true; delay(PRIMER_SETTLE_MS); }
          else Serial.println("Primer warning: timeout (continuing).");
        }

        float targetMl = (float)commanded_total_mL + (didPrimer ? PRIMER_ML : 0.0f);
        int32_t steps = ml2p(targetMl, DIR_ASPIRATE);
        Serial.print("Aspirating "); Serial.print(targetMl); Serial.println(" mL...");
        if (!move_rel_steps(steps, DEFAULT_SPEED_PPS)) {
          state = ERROR_STATE; prompt(); gate(300); break;
        }
        state = ASP_IDLE; prompt(); gate(200);
      }
    } break;

    case ASP_IDLE: {
      if (upFall)   { state = CAL_BL_WAIT; prompt(); gate(250); break; }
      if (downFall) {
        cal_total_measured_tenths = commanded_total_mL * 10; // prefill
        state = CAL_TOTAL; prompt(); gate(250); break;
      }
      if (okFall) {
        state = DISPENSING;
        gate(180);
      }
    } break;

    case DISPENSING: {
      if (remaining <= 0) { state = COMPLETE; prompt(); gate(240); break; }

      int32_t steps = ml2p((float)vol_mL, DIR_DISPENSE);
      Serial.print("Dispensing "); Serial.print(vol_mL); Serial.println(" mL...");
      if (!move_rel_steps(steps, DEFAULT_SPEED_PPS)) {
        state = ERROR_STATE; prompt(); gate(300); break;
      }

      remaining--;
      Serial.print("Remaining slots: "); Serial.println(remaining);

      state = (remaining > 0) ? ASP_IDLE : COMPLETE;
      prompt();
      gate(200);
    } break;

    case COMPLETE: {
      if (upFall || downFall) {
        cal_total_measured_tenths = commanded_total_mL * 10; // prefill
        state = CAL_TOTAL; prompt(); gate(250); break;
      }
      if (okFall) {
        vol_mL = 1; count = 1; remaining = 0; commanded_total_mL = 0;
        state = INPUT_VOLUME;
        prompt();
        gate(240);
      }
    } break;

    // ----- CAL: Total (gain) -----
    case CAL_TOTAL: {
      if (upFall   && cal_total_measured_tenths < 200) { cal_total_measured_tenths++; prompt(); gate(120); }
      if (downFall && cal_total_measured_tenths >   0) { cal_total_measured_tenths--; prompt(); gate(120); }
      if (okFall) {
        float meas = cal_total_measured_tenths / 10.0f;
        if (meas > 0.0f && commanded_total_mL > 0) {
          float old = PULSES_PER_ML;
          PULSES_PER_ML = PULSES_PER_ML * ((float)commanded_total_mL / meas);
          Serial.print("CAL: Total applied. Steps/mL "); Serial.print(old,3);
          Serial.print(" -> "); Serial.println(PULSES_PER_ML, 3);
        } else {
          Serial.println("CAL: Total skipped (zeros).");
        }
        state = INPUT_VOLUME; prompt(); gate(240);
      }
    } break;

    // ----- CAL: Backlash -----
    case CAL_BL_WAIT: {
      Serial.println("CAL: Backlash test starting (BL temporarily off)...");
      (void)BL_set(0);
      int32_t steps = ml2p(BL_TEST_ML, DIR_DISPENSE);
      if (!move_rel_steps(steps, DEFAULT_SPEED_PPS)) {
        Serial.println("CAL: Backlash test move failed.");
        state = INPUT_VOLUME; prompt(); gate(240); break;
      }
      bl_measured_tenths = (int)llround(BL_TEST_ML * 10.0f);
      state = CAL_BL_ADJUST; prompt(); gate(240);
    } break;

    case CAL_BL_ADJUST: {
      if (upFall   && bl_measured_tenths < 200) { bl_measured_tenths++; prompt(); gate(120); }
      if (downFall && bl_measured_tenths >   0) { bl_measured_tenths--; prompt(); gate(120); }
      if (okFall) {
        float meas_ml = bl_measured_tenths / 10.0f;
        float deficit = BL_TEST_ML - meas_ml;               // mL "lost" due to slack
        long newBL = (long)llround(fmax(0.0f, deficit * PULSES_PER_ML));
        uint32_t oldBL = BL_PULSES;
        BL_PULSES = (uint32_t)newBL;
        (void)BL_set(BL_PULSES);
        Serial.print("CAL: Backlash applied. BL ");
        Serial.print(oldBL); Serial.print(" -> "); Serial.print(BL_PULSES); Serial.println(" pulses.");
        Serial.println("Tip: run a 1 mL shot to verify first-dose accuracy.");
        state = INPUT_VOLUME; prompt(); gate(260);
      }
    } break;

    case ERROR_STATE: {
      if (okFall) {
        recover_to_safe_idle();
        (void)BL_set(BL_PULSES);
        state = INPUT_VOLUME; prompt(); gate(260);
      }
    } break;
  }
}
