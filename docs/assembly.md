# Drone Assembly

## Parts

- ESP32-C3 SuperMini
- ICM-20948 IMU breakout
- 4× 100N03A N-channel MOSFET (one per motor)
- 4× 10kΩ resistor (gate pull-down, one per MOSFET)
- 4× 8520 brushed DC motor
- 5V boost converter
- 1S LiPo battery (3.7V)
- 55mm or 65mm propellers

---

## Connection Tables

### Power

| From                      | To                  | Notes                              |
| ------------------------- | ------------------- | ---------------------------------- |
| Battery +                 | Boost converter IN+ |                                    |
| Battery −                 | Boost converter IN− |                                    |
| Boost converter 5V out    | ESP32 VIN           | 5V regulated                       |
| Boost converter GND       | ESP32 GND           | Common ground for all logic        |
| Boost converter BAT+      | MOSFET Source ×4    | Raw battery rail to motor switches |
| Boost converter BAT− /GND | All GND lines       | Shared ground for motors and logic |

### ICM-20948 IMU

| ICM-20948 | ESP32-C3 | Notes                       |
| --------- | -------- | --------------------------- |
| VIN       | 3.3V     | Regulated output from ESP32 |
| GND       | GND      |                             |
| SDA       | GPIO20   | I2C data                    |
| SCL       | GPIO21   | I2C clock (400 kHz)         |
| INT       | GPIO6    | Data-ready interrupt        |
| CS        | GND      | Tie low to select I2C mode  |

### Motors (via 100N03A MOSFET)

| Motor       | Gate (ESP32) | Gate-Source    | Source     | Drain   |
| ----------- | ------------ | -------------- | ---------- | ------- |
| Front Left  | GPIO0        | 10k pull-down  | Boost BAT+ | Motor + |
| Front Right | GPIO9        | 10k pull-down  | Boost BAT+ | Motor + |
| Rear Left   | GPIO1        | 10k pull-down  | Boost BAT+ | Motor + |
| Rear Right  | GPIO10       | 10k pull-down  | Boost BAT+ | Motor + |

Motor − on all four motors connects to common GND.

---

## Wiring Diagrams

### Power Distribution

```text
                        ┌─────────────────────────┐
                        │      Boost Converter    │
  Battery + ───── IN+ ──┤                         ├── 5V out ───► ESP32 VIN
  Battery − ───── IN− ──┤                         ├── GND ──────► ESP32 GND
                        │                         ├── BAT+ ────┬─ MOSFET FL Source
                        │                         │            ├─ MOSFET FR Source
                        │                         │            ├─ MOSFET RL Source
                        │                         │            └─ MOSFET RR Source
                        │                         ├── BAT−/GND ── all GND lines
                        └─────────────────────────┘
```

### ESP32-C3 Connections

```text
          ┌───────────────────────┐
   3.3V ──┤                       ├──► ICM-20948 VIN
    GND ──┤                       ├──► ICM-20948 GND
    GND ──┤       ESP32-C3        ├──► ICM-20948 CS
          │                       │
 GPIO20 ──┤ SDA                   ├──► ICM-20948 SDA
 GPIO21 ──┤ SCL                   ├──► ICM-20948 SCL
  GPIO6 ──┤ INT                   ├──► ICM-20948 INT
          │                       │
  GPIO0 ──┤ FL PWM                ├──► MOSFET FL Gate
  GPIO9 ──┤ FR PWM                ├──► MOSFET FR Gate
  GPIO1 ──┤ RL PWM                ├──► MOSFET RL Gate
 GPIO10 ──┤ RR PWM                ├──► MOSFET RR Gate
          └───────────────────────┘
```

### Motor Layout (top-down view)

```text
                    FRONT
                      ▲
                      │
     FL (GPIO0)       │       FR (GPIO9)
         ◎────────────┼────────────◎
         │            │            │
         │         ┌──┴──┐         │
         │         │ C3  │         │
         │         └──┬──┘         │
         │            │            │
         ◎────────────┼────────────◎
     RL (GPIO1)       │       RR (GPIO10)
                      │
                      ▼
                     BACK
```

### MOSFET Wiring (per motor, repeated ×4)

```text
  ESP32 GPIO ──────────── Gate
                        ┌──┘
                     [10kΩ]        pull-down holds gate low during boot/reset
                        └──┐
  Boost BAT+ ──────────── Source
                           │
                        [100N03A]
                           │
                         Drain ──────── Motor +
                                            │
                                         [Motor]
                                            │
  GND ──────────────────────────────────── Motor −
```

---

## Assembly Notes

- Tie ICM-20948 CS to GND before power-on — a floating CS pin can boot the chip into SPI mode
- The 10kΩ gate–source resistor on each MOSFET holds the gate low when the ESP32 GPIO is floating (boot/reset), preventing unintended motor spin-up
- All four MOSFET sources share the BAT+ rail from the boost converter; run a single wire to a bus point and branch from there
- Keep IMU signal wires (SDA/SCL/INT) routed away from motor wires to reduce noise coupling
- The ESP32 3.3V output powers the IMU; do not connect IMU VIN to the 5V boost output
