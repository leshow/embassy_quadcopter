# Drone Assembly

## Parts

- ESP32-C3 SuperMini
- ICM-20948 IMU breakout
- 4× 100N03A N-channel MOSFET (one per motor)
- 4× 10kΩ resistor (gate pull-down, one per MOSFET)
- 4× 8520 brushed DC motor
- 5V boost converter
- 470μF electrolytic capacitor
- 1S LiPo battery (3.7V)
- 55mm or 65mm propellers

---

## Connection Tables

### Power

| From                   | To                     | Notes                                  |
| ---------------------- | ---------------------- | -------------------------------------- |
| Battery +              | Boost converter IN+    |                                        |
| Battery −              | Boost converter IN−    |                                        |
| Boost converter 5V out | ESP32 VIN              | 5V regulated                           |
| Boost converter GND    | ESP32 GND              | Common ground for all logic            |
| 470μF cap +            | Boost converter 5V out | Positive leg to 5V out                 |
| 470μF cap −            | Boost converter GND    | Prevents brownout during motor inrush  |
| Battery +              | MOSFET Drain ×4        | Raw battery rail to motor switches     |
| Battery − / GND        | All GND lines          | Shared ground for motors and logic     |

### ICM-20948 IMU

| ICM-20948 | ESP32-C3 | Notes                              |
| --------- | -------- | ---------------------------------- |
| VIN       | 3.3V     | Regulated output from ESP32        |
| GND       | GND      |                                    |
| SDA       | GPIO20   | I2C data                           |
| SCL       | GPIO21   | I2C clock (400 kHz)                |
| INT       | GPIO6    | Data-ready interrupt               |
| CS        | 3.3V     | Tie HIGH to select I2C mode        |
| SDO       | 3.3V     | Sets I2C address to 0x69 (AD0 high)|

### Motors (via 100N03A MOSFET)

| Motor       | Gate (ESP32) | Gate-Source    | Drain      | Source |
| ----------- | ------------ | -------------- | ---------- | ------ |
| Front Left  | GPIO0        | 10k pull-down  | Battery +  | GND    |
| Front Right | GPIO9        | 10k pull-down  | Battery +  | GND    |
| Rear Left   | GPIO1        | 10k pull-down  | Battery +  | GND    |
| Rear Right  | GPIO10       | 10k pull-down  | Battery +  | GND    |

Motor + on all four motors connects directly to Battery +. Motor − connects to MOSFET Drain.

---

## Wiring Diagrams

### Power Distribution

```text
                        ┌─────────────────────────┐
                        │      Boost Converter    │
  Battery + ───── IN+ ──┤                         ├── 5V out ───┬──► ESP32 VIN
  Battery − ───── IN− ──┤                         ├── GND ───┬──┼──► ESP32 GND
                        └─────────────────────────┘         │  │
                                                            │  └─[470μF]─┘
                                                            └─── Battery −
                                                                    │
  Battery + ──────────────────────────────────────────┬─ MOSFET Drain FL
                                                      ├─ MOSFET Drain FR
                                                      ├─ MOSFET Drain RL
                                                      └─ MOSFET Drain RR
```

### ESP32-C3 Connections

```text
          ┌───────────────────────┐
   3.3V ──┤                       ├──► ICM-20948 VIN
   3.3V ──┤                       ├──► ICM-20948 CS
   3.3V ──┤       ESP32-C3        ├──► ICM-20948 SDO
    GND ──┤                       ├──► ICM-20948 GND
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
  Battery + ──────────────────────── Motor +
                                       │
                                    [Motor]
                                       │
                                    Motor − ──── Drain
                                              [100N03A]
  ESP32 GPIO ──────────── Gate         Source ──── GND
                        ┌──┘
                     [10kΩ]        pull-down holds gate low during boot/reset
                        └──┐
                          GND
```

---

## Assembly Notes

- Tie ICM-20948 CS to 3.3V — CS HIGH selects I2C mode; CS low puts the chip in SPI mode and it will not respond to I2C
- Tie ICM-20948 SDO to 3.3V — this sets the I2C address to 0x69, matching the firmware default
- The 10kΩ gate–source resistor on each MOSFET holds the gate low when the ESP32 GPIO is floating (boot/reset), preventing unintended motor spin-up
- MOSFETs are wired low-side: Drain to Motor −, Source to GND, Motor + connects directly to Battery +. N-channel MOSFETs cannot be used as high-side switches with a 3.3V gate signal
- Solder the 470μF electrolytic capacitor across the boost converter 5V output and GND, as close to those pads as possible (positive leg to 5V). This prevents the ESP32 from browning out during motor inrush current spikes
- All four MOSFET sources share GND; run a single wire to a bus point and branch from there
- Keep IMU signal wires (SDA/SCL/INT) routed away from motor wires to reduce noise coupling
- The ESP32 3.3V output powers the IMU; do not connect IMU VIN to the 5V boost output
