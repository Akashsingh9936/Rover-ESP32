# TX_RX — ESP32 Rover with ELRS Control

ESP32-based rover controller using **ExpressLRS (CRSF)** for RC input, with differential tank-drive mixing, multiple safety features, and a placeholder for autonomous control via NVIDIA AGX.

## Build Environments

```
pio run -e esp32dev   -t upload   → src/main.cpp        (raw channel reader)
pio run -e esp32dac   -t upload   → src/main_dac.cpp    (CH3 → split DAC test)
pio run -e esp32rover -t upload   → src/main_rover.cpp  (full rover — use this)
```

## Hardware

| Part | Example |
|---|---|
| MCU | ESP32 DevKit V1 (needs the two internal DACs) |
| RC receiver | RadioMaster RP1 (ExpressLRS, 2.4 GHz) |
| Transmitter | Any ELRS-compatible radio (e.g. RadioMaster Pocket / TX16S) |
| Motor driver | KLE driver × 2 (one per side, analog speed + direction pin) |
| Relays | 5 total — 2 direction, 2 brake, 1 LED |

### Wiring

```
   RadioMaster RP1              ESP32 DevKit V1
   ───────────────              ───────────────
   TX  ─────────────────────▶   GPIO16 (RX2)
   RX  ◀─────────────────────   GPIO17 (TX2)  [telemetry]
   5V  ───────────────────────  5V / VIN
   GND ───────────────────────  GND
```

### GPIO Map

| GPIO | Function | Notes |
|------|----------|-------|
| 16 | CRSF RX (from RP1 TX) | UART2 RX |
| 17 | CRSF TX (to RP1 RX) | UART2 TX, optional telemetry |
| 25 | DAC1 → KLE LEFT speed | 0–3.3 V analog |
| 26 | DAC2 → KLE RIGHT speed | 0–3.3 V analog |
| 14 | LEFT direction relay | Relay switches 12 V → KLE reverse pin |
| 27 | RIGHT direction relay | Same |
| 12 | LEFT brake relay | ⚠ Strapping pin — must be LOW at boot |
| 13 | RIGHT brake relay | |
| 15 | LED relay | |

## Channel Assignments

| Channel | Function | Values |
|---------|----------|--------|
| CH3 | Throttle (proportional) | 174 ← reverse · 970–1050 dead-band · 1811 → forward |
| CH4 | Yaw (proportional) | 174 ← left · 970–1050 dead-band · 1811 → right |
| CH5 | Manual / Auto toggle | 191 = Manual, 1792 = Auto |
| CH7 | LED relay | 1004 = OFF, 1792 = ON |
| CH8 | ARM / DISARM | 191 = Disarm, 1792 = Arm |
| CH10 | Dead man's switch (manual) | 191 = Released, 1792 = Pressed |

## Control Logic

### Differential Mixing (Tank-Drive)

```
Throttle = normalize(CH3)   →  -1.0 (rev) .. 0 .. +1.0 (fwd)
Yaw      = normalize(CH4)   →  -1.0 (left) .. 0 .. +1.0 (right)

Left  wheel = Throttle + Yaw     (clamped ±1.0)
Right wheel = Throttle - Yaw     (clamped ±1.0)

Sign      → direction relay (negative = reverse)
Magnitude → DAC output (0–255 → 0–3.3 V)
```

| Stick combo | Result |
|-------------|--------|
| CH3 fwd + CH4 center | Straight forward |
| CH3 fwd + CH4 right | Arc right (left faster, right slower) |
| CH3 fwd + CH4 full right | Sharp right (left full, right stopped/reversed) |
| CH3 center + CH4 right | Zero-degree spin clockwise |
| CH3 rev + CH4 left | Arc left in reverse |

### Safety Chain

Movement requires **all** of:

1. RC link up
2. **ARMed** (CH8 = 1792)
3. **Manual mode** (CH5 = 191) + **dead man pressed** (CH10 = 1792)
   — or — **Auto mode** (CH5 = 1792) + AGX commands (not yet implemented)

**Brakes** engage whenever motors are not actively driving.

**LED** (CH7) works independently of arm/drive state; turns OFF on link loss.

### Fail-Safe

No RC link → DACs = 0, direction relays OFF, brakes ON, LED OFF.

## Serial Output

```
ARM  MAN  DM [FWD-R] L=150(1.94V) R= 80(1.04V) brk=0 led=1
DSRM MAN     [STOP ] L=  0(0.00V) R=  0(0.00V) brk=1 led=0
[WARN] No RC link — brakes ON, LED OFF
```

## Dependencies

Declared in `platformio.ini`:

```ini
lib_deps = zz-cat/CRSFforArduino
```

PlatformIO fetches it automatically on first build.

## What's Next

- **AGX UART integration**: wire ESP32 ↔ AGX UART for autonomous drive commands in auto mode.
- **External DAC**: swap 8-bit internal DAC for higher-resolution I²C/SPI DAC.
- **Telemetry**: send battery voltage / RSSI back to the radio via TX2.
