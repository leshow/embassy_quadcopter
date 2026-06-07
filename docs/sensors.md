# Sensor Reference

## Axis Convention (ICM-20948 and MPU-6050)

```
        +Y (forward)
         ↑
         |
-X ------+------ +X (right)
         |
         ↓
        -Y (back)

+Z points UP out of the chip surface
-Z points DOWN into the desk
```

The physical orientation of the chip on the board determines which way these
axes point relative to the drone frame. If the chip is rotated 90°, X and Y
swap. Verify by placing the board flat and checking that `acc.z ≈ +1g`.

---

## Accelerometer (`acc.x`, `acc.y`, `acc.z`)

Measures force per unit mass in **g** (1g = 9.8 m/s²). At rest the sensor
measures gravity, which registers as a +Z force because the chip is being
pushed upward against gravity.

| Board position  | acc.x | acc.y | acc.z |
| --------------- | ----- | ----- | ----- |
| Flat on desk    | ≈ 0   | ≈ 0   | ≈ +1g |
| Tilted right    | ≈ +1g | ≈ 0   | ≈ 0   |
| Tilted left     | ≈ -1g | ≈ 0   | ≈ 0   |
| Nose up (pitch) | ≈ 0   | ≈ -1g | ≈ 0   |
| Nose down       | ≈ 0   | ≈ +1g | ≈ 0   |
| Upside down     | ≈ 0   | ≈ 0   | ≈ -1g |

**Weakness:** Cannot distinguish tilt from linear acceleration. Under throttle
or vibration the readings are corrupted.

---

## Gyroscope (`gyro.x`, `gyro.y`, `gyro.z`)

Measures angular velocity in **rad/s** around each axis. Sign follows the
right-hand rule: curl the right hand's fingers in the direction of rotation,
thumb points along the positive axis.

| Motion             | Primary axis |
| ------------------ | ------------ |
| Rolling right      | gyro.x > 0   |
| Rolling left       | gyro.x < 0   |
| Pitching nose up   | gyro.y > 0   |
| Pitching nose down | gyro.y < 0   |
| Yawing left        | gyro.z > 0   |
| Yawing right       | gyro.z < 0   |

**Weakness:** Integrating gyro rate over time accumulates drift. A stationary
sensor will slowly show a growing angle that isn't real.

---

## Roll and Pitch from Accelerometer

From [NXP application note AN3461](https://www.nxp.com/docs/en/application-note/AN3461.pdf),
equations 28 and 29.

$$\phi_{roll} = \text{atan2}(a_y,\ \sqrt{a_x^2 + a_z^2})$$

$$\theta_{pitch} = \text{atan2}(-a_x,\ \sqrt{a_y^2 + a_z^2})$$

`atan2` is used instead of `atan` to correctly handle all four quadrants
(full ±180° range rather than ±90°).

Yaw cannot be derived from the accelerometer — gravity is parallel to the yaw
axis so rotating flat on a table produces no change in `acc.x/y/z`.

---

## Complementary Filter

Fuses the gyro (accurate short-term, drifts long-term) with the accelerometer
(stable long-term, noisy short-term):

```
angle = α * (angle + gyro_rate * dt) + (1 - α) * accel_angle
```

`α` controls the crossover. Derived from a time constant τ (how long drift
correction takes):

$$\alpha = \frac{\tau}{\tau + dt}$$

| τ    | α (at dt=5ms) | Behaviour                          |
| ---- | ------------- | ---------------------------------- |
| 0.1s | 0.980         | Fast correction, more accel noise  |
| 0.5s | 0.990         | Balanced                           |
| 2.0s | 0.9975        | Very smooth, slow drift correction |

For a drone, `α = 0.98–0.99` is typical. Motor vibration and linear
acceleration from throttle corrupt the accel signal during flight, so leaning
heavily on the gyro is preferred.

---

## Yaw from Magnetometer (ICM-20948 only)

Raw magnetometer heading is computed as `atan2(-mag.y, mag.x)`, but this is
only valid when the board is perfectly flat. Tilting the board rotates the
magnetometer axes, projecting the field incorrectly.

**Tilt-compensated heading:**

$$\psi = \text{atan2}(m_y \cos\phi - m_z \sin\phi,\quad m_x \cos\theta + m_y \sin\theta \sin\phi + m_z \sin\theta \cos\phi)$$

where φ = roll, θ = pitch (radians from the complementary filter).

---

## Sensor Roles Summary

| Sensor        | Provides                    | Good at              | Bad at                  |
| ------------- | --------------------------- | -------------------- | ----------------------- |
| Accelerometer | Absolute tilt (roll, pitch) | Long-term stability  | Linear accel, vibration |
| Gyroscope     | Rotation rate               | Fast motion tracking | Drift over time         |
| Magnetometer  | Absolute heading (yaw)      | Long-term yaw ref    | Magnetic interference   |
