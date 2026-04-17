# Theseus Avionics Telemetry

Flight telemetry system for the Theseus liquid biprop rocket (Beach Launch Team).  
Streams altitude, pressure, bay temperature, acceleration, and gyro data from the avionics bay to the ground station over LoRa RF.

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Flight computer | Teensy 4.1 | 600 MHz Cortex-M7, 3.3V logic |
| Barometer | BMP390 | I2C, pins 18/19, address 0x77 |
| IMU | MPU6050 | I2C, pins 18/19, address 0x68 |
| LoRa (airborne) | SX1276 915 MHz | UART Serial1, TX pin 1, RX pin 0 |
| LoRa (ground) | SX1276 915 MHz | USB-serial adapter, COM8 |
| Power | LiPo → buck converter → 5V | Fed to Teensy VIN |

---

## Telemetry channels

| Channel | Source | Unit |
|---|---|---|
| `theseus_alt_m` | BMP390 | metres |
| `theseus_press_pa` | BMP390 | Pascal |
| `theseus_temp_c` | BMP390 | °C (bay temperature) |
| `theseus_ax` | MPU6050 | g |
| `theseus_ay` | MPU6050 | g |
| `theseus_az` | MPU6050 | g — primary burnout/apogee indicator |
| `theseus_gx` | MPU6050 | deg/s |
| `theseus_gy` | MPU6050 | deg/s |
| `theseus_gz` | MPU6050 | deg/s |

Packet format over LoRa UART at 9600 baud, 10 Hz:
```
THS,<alt_m>,<press_pa>,<temp_c>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\r\n
```

---

## Firmware

**Location:** `Tiny_Theseus_Telemetry/` — Arduino sketch for Teensy 4.1

**Files:**
```
Tiny_Theseus_Telemetry/
├── Tiny_Theseus_Telemetry.ino   main loop — init, 10 Hz transmit
├── bmp390.h / bmp390.cpp        BMP390 driver — init, calib, read, altitude
├── mpu6050.h / mpu6050.cpp      MPU6050 driver — init, accel + gyro read
```

**Dependencies:** Arduino IDE + Teensyduino. No external libraries required.

**Flash:**
1. Open `Tiny_Theseus_Telemetry.ino` in Arduino IDE
2. Tools → Board → Teensyduino → Teensy 4.1
3. Tools → USB Type → Serial
4. Upload

**Verify on bench:**  
Open Serial Monitor at 115200 baud. You should see:
```
Theseus telemetry starting...
BMP390 OK
MPU6050 OK
alt_m,press_pa,temp_c,ax,ay,az,gx,gy,gz
36.90,100882.46,28.06,-0.12,0.00,1.04,...
```

---

## Ground software

### Dependencies
```
pip install pyserial synnax numpy pygame PyOpenGL PyOpenGL_accelerate
```

### Option 1 — Terminal dashboard (no dependencies beyond pyserial)
```
python theseus_read.py --port COM8 --baud 9600
```
With CSV logging:
```
python theseus_read.py --port COM8 --baud 9600 --log
```
Displays live altitude, temperature, acceleration, gyro with colour coding and running maximums. Saves timestamped CSV to current folder when `--log` is set.

### Option 2 — 3D visualizer
```
python theseus_viz_v2.py --port COM8 --baud 9600
```
Demo mode (no hardware needed):
```
python theseus_viz_v2.py --demo
```
Shows a 3D rocket model rotating in real time based on attitude from a complementary filter. Left-click drag to orbit the camera. Full data panel on the left.

### Option 3 — Synnax integration (launch day)
Runs alongside the BLTDAQ LabJack backend on the ground laptop.

**Step 1 — create channels once:**
```
python lora_rx.py --setup
```

**Step 2 — launch day terminal layout:**
```
Terminal 1: synnax start --listen localhost:9091 --insecure
Terminal 2: python backend/src/main.py live_go
Terminal 3: python lora_rx.py --port COM8 --baud 9600
```

Open `localhost:9091` in a browser. Create a new schematic tab and add line charts for `theseus_alt_m`, `theseus_temp_c`, and `theseus_az`.

**Environment overrides** (if Synnax is on a different machine):
```
set SYNNAX_HOST=192.168.1.50
set SYNNAX_PORT=9091
```

---

## Wiring

### I2C bus (both sensors share pins 18/19)
```
BMP390  SDA → Teensy pin 18
BMP390  SCL → Teensy pin 19
BMP390  VIN → Teensy 3.3V
BMP390  GND → GND

MPU6050 SDA → Teensy pin 18
MPU6050 SCL → Teensy pin 19
MPU6050 VCC → Teensy 3.3V
MPU6050 GND → GND
MPU6050 AD0 → GND  (sets address to 0x68)
```

### LoRa UART (Serial1)
```
LoRa TX → Teensy pin 0 (RX1)
LoRa RX → Teensy pin 1 (TX1)
LoRa VCC → Teensy 3.3V
LoRa GND → GND
```

### Power
```
LiPo → buck converter → 5V rail → Teensy VIN
                                 → (other 5V devices)
All GND must be common.
```

---

## I2C scan

Before flashing the main firmware, confirm both sensors are on the bus:

```cpp
#include <Wire.h>
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Wire.begin();
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("Device found at 0x");
            Serial.println(addr, HEX);
        }
    }
}
void loop() {}
```

Expected output:
```
Device found at 0x68   <- MPU6050
Device found at 0x77   <- BMP390
```

---

## Flight event reference

| Event | What to watch |
|---|---|
| Motor ignition | `theseus_az` spikes above 1g |
| Burnout | `theseus_az` drops back toward 0g |
| Apogee | `theseus_alt_m` peaks |
| Instability | `theseus_gx` or `theseus_gy` nonzero during burn |
| Bay thermal limit | `theseus_temp_c` above 60°C |

---

## File summary

| File | Purpose |
|---|---|
| `Tiny_Theseus_Telemetry.ino` | Main firmware sketch |
| `bmp390.h / .cpp` | BMP390 barometer driver |
| `mpu6050.h / .cpp` | MPU6050 IMU driver |
| `theseus_read.py` | Terminal dashboard + CSV logger |
| `theseus_viz_v2.py` | 3D visualizer |
| `lora_rx.py` | Synnax ground receiver |
