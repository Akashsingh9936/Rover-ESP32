/*
  ROVER — Differential Turning + Safety Features
  ================================================
  Hardware : ESP32 DevKitV1 + MCP4922 dual 12-bit SPI DAC
  Receiver : RadioMaster RP1 (ExpressLRS)
  Motors   : driven by a KLE motor driver per side.
             Each KLE channel takes:
               - speed input   : 0 – 3.3 V analog (from MCP4922)
               - direction pin : needs 12 V to flip to REVERSE
                                 (0 V / floating → FORWARD)

  Wiring:
    RP1 TX  →  ESP32 GPIO16 (RX2)
    RP1 RX  →  ESP32 GPIO17 (TX2)  [optional telemetry]
    RP1 5V  →  ESP32 5V
    RP1 GND →  ESP32 GND

    MCP4922 VDD   →  3.3V        MCP4922 VSS   →  GND
    MCP4922 CS    →  ESP32 GPIO5
    MCP4922 SCK   →  ESP32 GPIO18
    MCP4922 SDI   →  ESP32 GPIO23
    MCP4922 LDAC  →  GND (immediate output update)
    MCP4922 VREFA →  3.3V        MCP4922 VREFB →  3.3V
    MCP4922 VOUTA →  KLE LEFT  speed input
    MCP4922 VOUTB →  KLE RIGHT speed input

    GPIO14 →  LEFT  direction relay
    GPIO27 →  RIGHT direction relay
    GPIO12 →  LEFT  brake relay    (⚠ strapping pin — see note)
    GPIO13 →  RIGHT brake relay
    GPIO15 →  LED relay

  ⚠ GPIO12 is a strapping pin. It must be LOW during boot or the ESP32
  may fail to start. Ensure the relay module does not pull GPIO12 HIGH at
  power-on (most relay boards have their own pull-down — verify yours).

  ─── Channel assignments ────────────────────────────────────────────
  CH3  — throttle     (proportional: fwd / rev)
  CH4  — yaw          (proportional: turn left / right)
  CH5  — mode toggle  : 191 = MANUAL, 1792 = AUTO
  CH7  — LED relay    : 1004 = OFF, 1792 = ON
  CH8  — ARM/DISARM   : 191 = DISARM, 1792 = ARM
  CH10 — dead man's switch (manual only): 191 = released, 1792 = pressed

  ─── Mixing (differential / tank-drive) ─────────────────────────────
  CH3/CH4 normalized to -1.0 .. +1.0, dead-band at center.
    Left  wheel = Throttle + Yaw
    Right wheel = Throttle - Yaw
  Clamped to ±1.0.  Sign → direction relay, magnitude → DAC.

  ─── Safety chain ───────────────────────────────────────────────────
  Movement requires ALL of:
    1. RC link up
    2. ARMed          (CH8 = 1792)
    3. Manual mode    (CH5 = 191)  +  dead man pressed (CH10 = 1792)
       — OR —
       Auto mode      (CH5 = 1792) +  AGX commands     (not yet implemented)

  Brakes engage whenever motors are not actively driving.
  LED (CH7) is independent of arm/drive state; OFF on link loss.

  Fail-safe:
    No RC link → DACs = 0, direction relays OFF, brakes ON, LED OFF.
*/

#include <Arduino.h>
#include <SPI.h>
#include <CRSFforArduino.hpp>

// ─── PIN DEFINITIONS ────────────────────────────────────────────────
#define CRSF_RX_PIN       16
#define CRSF_TX_PIN       17

#define DAC_CS_PIN         5    // MCP4922 chip-select (active LOW)

#define RELAY_LEFT_PIN    14    // LEFT  direction relay
#define RELAY_RIGHT_PIN   27    // RIGHT direction relay
#define BRAKE_LEFT_PIN    12    // LEFT  brake relay
#define BRAKE_RIGHT_PIN   13    // RIGHT brake relay
#define LED_RELAY_PIN     15    // LED on/off relay

// Set to 1 if your relay module energises on HIGH; 0 for active-LOW boards.
#define RELAY_ACTIVE_HIGH 1

#if RELAY_ACTIVE_HIGH
  #define RELAY_ON   HIGH
  #define RELAY_OFF  LOW
#else
  #define RELAY_ON   LOW
  #define RELAY_OFF  HIGH
#endif

// ─── CRSF VALUE BAND POINTS ─────────────────────────────────────────
#define CRSF_MIN            174
#define CRSF_CENTER_LOW     970
#define CRSF_CENTER_HIGH    1050
#define CRSF_MAX            1811

// Toggle switch threshold (midpoint between 191/1004 and 1792)
#define SWITCH_THRESH       1500

// ─── CRSF INSTANCE ──────────────────────────────────────────────────
CRSFforArduino crsf(&Serial2, CRSF_RX_PIN, CRSF_TX_PIN);

// ─── LINK STATUS ────────────────────────────────────────────────────
volatile bool linkIsUp = false;
void onLinkUp()   { linkIsUp = true; }
void onLinkDown() { linkIsUp = false; }

// ─── CONTROL STATE ──────────────────────────────────────────────────
enum DriveMode { MODE_STOP, MODE_FWD, MODE_REV,
                 MODE_FWD_L, MODE_FWD_R, MODE_REV_L, MODE_REV_R,
                 MODE_SPIN_L, MODE_SPIN_R };

struct Outputs {
  uint16_t  dacLeft;
  uint16_t  dacRight;
  bool      relayLeft;
  bool      relayRight;
  bool      brakeLeft;
  bool      brakeRight;
  bool      led;
  DriveMode drive;
  bool      armed;
  bool      autoMode;
  bool      deadMan;
};

static float normalizeStick(int ch)
{
  if (ch >= CRSF_CENTER_HIGH)
    return (float)(ch - CRSF_CENTER_HIGH) / (CRSF_MAX - CRSF_CENTER_HIGH);
  if (ch <= CRSF_CENTER_LOW)
    return -(float)(CRSF_CENTER_LOW - ch) / (CRSF_CENTER_LOW - CRSF_MIN);
  return 0.0f;
}

static Outputs computeOutputs(int ch3, int ch4, int ch5,
                               int ch7, int ch8, int ch10)
{
  Outputs o = {};
  o.drive      = MODE_STOP;
  o.brakeLeft  = true;
  o.brakeRight = true;

  // ── Toggle switches ──
  o.armed    = (ch8  < SWITCH_THRESH);
  o.autoMode = (ch5  > SWITCH_THRESH);
  o.deadMan  = (ch10 > SWITCH_THRESH);
  o.led      = (ch7  > SWITCH_THRESH);

  // ── Can we move? ──
  bool canMove = o.armed;
  if (o.autoMode)
    canMove = false;                   // AGX not implemented yet
  else
    canMove = canMove && o.deadMan;    // manual needs dead man pressed

  if (!canMove)
    return o;   // brakes ON, DACs zero, direction relays OFF

  // ── Differential mixing ──
  ch3 = constrain(ch3, CRSF_MIN, CRSF_MAX);
  ch4 = constrain(ch4, CRSF_MIN, CRSF_MAX);

  float throttle = normalizeStick(ch3);
  float yaw      = normalizeStick(ch4);

  float left  = constrain(throttle + yaw, -1.0f, 1.0f);
  float right = constrain(throttle - yaw, -1.0f, 1.0f);

  o.dacLeft    = (uint16_t)(fabsf(left)  * 4095.0f);
  o.dacRight   = (uint16_t)(fabsf(right) * 4095.0f);
  o.relayLeft  = (left  < 0.0f);
  o.relayRight = (right < 0.0f);

  // ── Brakes: release when motors are driving ──
  bool driving = (o.dacLeft > 0 || o.dacRight > 0);
  o.brakeLeft  = !driving;
  o.brakeRight = !driving;

  // ── Display mode ──
  bool hasThrottle = (throttle != 0.0f);
  bool hasYaw      = (yaw != 0.0f);

  if      (!hasThrottle && !hasYaw)        o.drive = MODE_STOP;
  else if (!hasThrottle &&  yaw > 0)       o.drive = MODE_SPIN_R;
  else if (!hasThrottle &&  yaw < 0)       o.drive = MODE_SPIN_L;
  else if ( throttle > 0 && !hasYaw)       o.drive = MODE_FWD;
  else if ( throttle < 0 && !hasYaw)       o.drive = MODE_REV;
  else if ( throttle > 0 &&  yaw > 0)      o.drive = MODE_FWD_R;
  else if ( throttle > 0 &&  yaw < 0)      o.drive = MODE_FWD_L;
  else if ( throttle < 0 &&  yaw > 0)      o.drive = MODE_REV_R;
  else                                      o.drive = MODE_REV_L;

  return o;
}

// MCP4922 16-bit command word:
//   bit 15   : channel (0 = A / left, 1 = B / right)
//   bit 14   : BUF  (1 = buffered Vref)
//   bit 13   : GA   (1 = 1x gain, 0 = 2x gain)
//   bit 12   : SHDN (1 = active, 0 = shutdown)
//   bits 11-0: 12-bit data
static void writeMCP4922(uint8_t channel, uint16_t value)
{
  uint16_t cmd = ((channel & 1) << 15)
               | (1 << 14)   // buffered
               | (1 << 13)   // 1x gain
               | (1 << 12)   // active
               | (value & 0x0FFF);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DAC_CS_PIN, LOW);
  SPI.transfer16(cmd);
  digitalWrite(DAC_CS_PIN, HIGH);
  SPI.endTransaction();
}

static void applyOutputs(const Outputs &o)
{
  writeMCP4922(0, o.dacLeft);   // channel A → left motor
  writeMCP4922(1, o.dacRight);  // channel B → right motor
  digitalWrite(RELAY_LEFT_PIN,  o.relayLeft  ? RELAY_ON : RELAY_OFF);
  digitalWrite(RELAY_RIGHT_PIN, o.relayRight ? RELAY_ON : RELAY_OFF);
  digitalWrite(BRAKE_LEFT_PIN,  o.brakeLeft  ? RELAY_ON : RELAY_OFF);
  digitalWrite(BRAKE_RIGHT_PIN, o.brakeRight ? RELAY_ON : RELAY_OFF);
  digitalWrite(LED_RELAY_PIN,   o.led        ? RELAY_ON : RELAY_OFF);
}

// ────────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("==================================================");
  Serial.println("ROVER — Differential Turning + Safety Features");
  Serial.println("==================================================");
  Serial.println("CH3/4 : throttle + yaw (tank-drive mix)");
  Serial.println("CH5   : MAN/AUTO   CH7: LED   CH8: ARM/DISARM");
  Serial.println("CH10  : dead man's switch (manual mode)");
  Serial.println("Safety: ARM + dead man → move.  Link loss → stop.");

  // Direction relays
  pinMode(RELAY_LEFT_PIN,  OUTPUT);
  pinMode(RELAY_RIGHT_PIN, OUTPUT);
  digitalWrite(RELAY_LEFT_PIN,  RELAY_OFF);
  digitalWrite(RELAY_RIGHT_PIN, RELAY_OFF);

  // Brake relays — engaged at startup (safe)
  pinMode(BRAKE_LEFT_PIN,  OUTPUT);
  pinMode(BRAKE_RIGHT_PIN, OUTPUT);
  digitalWrite(BRAKE_LEFT_PIN,  RELAY_ON);
  digitalWrite(BRAKE_RIGHT_PIN, RELAY_ON);

  // LED relay — off at startup
  pinMode(LED_RELAY_PIN, OUTPUT);
  digitalWrite(LED_RELAY_PIN, RELAY_OFF);

  // MCP4922 external DAC via SPI
  pinMode(DAC_CS_PIN, OUTPUT);
  digitalWrite(DAC_CS_PIN, HIGH);
  SPI.begin();
  writeMCP4922(0, 0);
  writeMCP4922(1, 0);

  if (!crsf.begin())
  {
    Serial.println("[ERROR] CRSF init failed — check wiring on GPIO16/17");
    while (true) { delay(1000); }
  }

  crsf.setLinkUpCallback(onLinkUp);
  crsf.setLinkDownCallback(onLinkDown);
  Serial.println("[OK] CRSF initialised — waiting for RC link...");
}

#define PRINT_INTERVAL_MS   100
unsigned long lastPrint = 0;

// ────────────────────────────────────────────────────────────────────
void loop()
{
  crsf.update();

  // Safe defaults: stopped, brakes on, LED off, disarmed, manual
  Outputs out = {};
  out.drive      = MODE_STOP;
  out.brakeLeft  = true;
  out.brakeRight = true;

  int ch3 = 992, ch4 = 992, ch5 = 191, ch7 = 1004, ch8 = 1792, ch10 = 191;

  if (linkIsUp)
  {
    ch3  = crsf.getChannel(3);
    ch4  = crsf.getChannel(4);
    ch5  = crsf.getChannel(5);
    ch7  = crsf.getChannel(7);
    ch8  = crsf.getChannel(8);
    ch10 = crsf.getChannel(10);
    out  = computeOutputs(ch3, ch4, ch5, ch7, ch8, ch10);
  }

  applyOutputs(out);

  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL_MS)
  {
    lastPrint = now;

    if (!linkIsUp)
    {
      Serial.println("[WARN] No RC link — brakes ON, LED OFF");
      return;
    }

    const char *driveName;
    switch (out.drive)
    {
      case MODE_FWD:    driveName = "FWD  "; break;
      case MODE_REV:    driveName = "REV  "; break;
      case MODE_FWD_L:  driveName = "FWD-L"; break;
      case MODE_FWD_R:  driveName = "FWD-R"; break;
      case MODE_REV_L:  driveName = "REV-L"; break;
      case MODE_REV_R:  driveName = "REV-R"; break;
      case MODE_SPIN_L: driveName = "SPINL"; break;
      case MODE_SPIN_R: driveName = "SPINR"; break;
      default:          driveName = "STOP "; break;
    }

    float vL = 3.3f * out.dacLeft  / 4095.0f;
    float vR = 3.3f * out.dacRight / 4095.0f;

    Serial.printf("%s %s %s [%s] L=%4u(%.2fV) R=%4u(%.2fV) dirL=%d dirR=%d brk=%d led=%d\n",
                  out.armed    ? "ARM " : "DSRM",
                  out.autoMode ? "AUTO" : "MAN ",
                  (!out.autoMode && out.deadMan) ? "DM" : "  ",
                  driveName,
                  out.dacLeft,  vL,
                  out.dacRight, vR,
                  out.relayLeft  ? 1 : 0,
                  out.relayRight ? 1 : 0,
                  out.brakeLeft ? 1 : 0,
                  out.led ? 1 : 0);
  }
}
