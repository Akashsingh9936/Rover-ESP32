/*
  ROVER — Step 2: CRSF CH3 → split DAC output
  =============================================
  Hardware : ESP32 DevKitV1 (classic ESP32 only — DACs don't exist on S2/S3/C3)
  Receiver : RadioMaster RP1 (ExpressLRS)

  Wiring:
    RP1 TX  →  ESP32 GPIO16 (RX2)
    RP1 RX  →  ESP32 GPIO17 (TX2)  [optional telemetry]
    RP1 5V  →  ESP32 5V
    RP1 GND →  ESP32 GND

    DAC outputs (0 – ~3.3 V analog):
      GPIO25 (DAC1)  →  "below center" output   (CH3 going down from 988 toward 172)
      GPIO26 (DAC2)  →  "above center" output   (CH3 going up from 995 toward 1811)

  Behaviour of CH3 (single stick → two outputs):

    CH3 value : 172 ─────── 988 │ 995 ─────── 1811
    GPIO25 V  : 3.3 ─────── 0.0 │ 0.0 ─────── 0.0
    GPIO26 V  : 0.0 ─────── 0.0 │ 0.0 ─────── 3.3

    Dead zone 988–995 → both DACs = 0 V (neutral).
    On RC link loss  → both DACs = 0 V (safe).

  Typical use: feed GPIO25 → "reverse" input and GPIO26 → "forward" input of an
  analog motor driver (or any device that wants two unipolar command voltages).
*/

#include <Arduino.h>
#include <CRSFforArduino.hpp>

// ─── PIN DEFINITIONS ────────────────────────────────────────────────
#define CRSF_RX_PIN   16
#define CRSF_TX_PIN   17

#define DAC_LOW_PIN   25    // DAC1 — active when CH3 is below center
#define DAC_HIGH_PIN  26    // DAC2 — active when CH3 is above center

// ─── CRSF VALUE BAND POINTS ─────────────────────────────────────────
#define CRSF_MIN            172
#define CRSF_CENTER_LOW     988     // top of the "below center" band
#define CRSF_CENTER_HIGH    995     // bottom of the "above center" band
#define CRSF_MAX            1811

// ─── CRSF INSTANCE ──────────────────────────────────────────────────
CRSFforArduino crsf(&Serial2, CRSF_RX_PIN, CRSF_TX_PIN);

// ─── TIMING ─────────────────────────────────────────────────────────
#define PRINT_INTERVAL_MS   100
unsigned long lastPrint = 0;

// ─── LINK STATUS ────────────────────────────────────────────────────
volatile bool linkIsUp = false;
void onLinkUp()   { linkIsUp = true; }
void onLinkDown() { linkIsUp = false; }

// ─── CORE MAPPING ───────────────────────────────────────────────────
// Splits one CRSF channel value into two DAC bytes.
// Returned via out-params so loop() can also log them.
static void splitChannelToDacs(int ch, uint8_t &outLow, uint8_t &outHigh)
{
  if (ch < CRSF_MIN)  ch = CRSF_MIN;
  if (ch > CRSF_MAX)  ch = CRSF_MAX;

  if (ch <= CRSF_CENTER_LOW)
  {
    // Below center → GPIO25 climbs 0 → 255 as ch goes 988 → 172.
    outLow  = (uint8_t) map(ch, CRSF_CENTER_LOW, CRSF_MIN, 0, 255);
    outHigh = 0;
  }
  else if (ch >= CRSF_CENTER_HIGH)
  {
    // Above center → GPIO26 climbs 0 → 255 as ch goes 995 → 1811.
    outLow  = 0;
    outHigh = (uint8_t) map(ch, CRSF_CENTER_HIGH, CRSF_MAX, 0, 255);
  }
  else
  {
    // Dead zone (988 < ch < 995) → both off.
    outLow  = 0;
    outHigh = 0;
  }
}

// ────────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("=========================================");
  Serial.println("ROVER Step 2 — CRSF CH3 → split DAC");
  Serial.println("=========================================");
  Serial.println("CH3 below 988 → GPIO25 (0 V → 3.3 V as stick drops)");
  Serial.println("CH3 above 995 → GPIO26 (0 V → 3.3 V as stick rises)");
  Serial.println("Dead zone 988–995 → both at 0 V");

  dacWrite(DAC_LOW_PIN,  0);
  dacWrite(DAC_HIGH_PIN, 0);

  if (!crsf.begin())
  {
    Serial.println("[ERROR] CRSF init failed — check wiring on GPIO16/17");
    while (true) { delay(1000); }
  }

  crsf.setLinkUpCallback(onLinkUp);
  crsf.setLinkDownCallback(onLinkDown);

  Serial.println("[OK] CRSF initialised — waiting for RC link...");
}

// ────────────────────────────────────────────────────────────────────
void loop()
{
  crsf.update();

  uint8_t dLow = 0, dHigh = 0;

  if (linkIsUp)
  {
    int ch3 = crsf.getChannel(3);
    splitChannelToDacs(ch3, dLow, dHigh);
  }
  // else: both stay at 0 → safe stop.

  dacWrite(DAC_LOW_PIN,  dLow);
  dacWrite(DAC_HIGH_PIN, dHigh);

  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL_MS)
  {
    lastPrint = now;

    if (!linkIsUp)
    {
      Serial.println("[WARN] No RC link — both DACs = 0 V");
      return;
    }

    int ch3 = crsf.getChannel(3);
    float vLow  = 3.3f * dLow  / 255.0f;
    float vHigh = 3.3f * dHigh / 255.0f;

    const char *zone;
    if      (ch3 <= CRSF_CENTER_LOW)  zone = "LOW ";
    else if (ch3 >= CRSF_CENTER_HIGH) zone = "HIGH";
    else                              zone = "DEAD";

    Serial.printf("CH3=%4d [%s]  GPIO25=%3u (~%.2f V)   GPIO26=%3u (~%.2f V)\n",
                  ch3, zone, dLow, vLow, dHigh, vHigh);
  }
}
