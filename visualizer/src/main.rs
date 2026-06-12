use anyhow::Context;
use macroquad::prelude::*;
use std::{
    io::BufRead,
    str::FromStr,
    sync::{Arc, Mutex},
    thread,
};

#[derive(Clone, Copy, Default)]
struct Attitude {
    roll: f32,
    pitch: f32,
    yaw: f32,
}

impl FromStr for Attitude {
    type Err = anyhow::Error;

    fn from_str(line: &str) -> Result<Self, Self::Err> {
        let (roll, rest) = extract_val(line, "roll: ").context("failed to parse roll")?;
        let (pitch, rest) = extract_val(rest, "pitch: ").context("failed to parse pitch")?;
        let (yaw, _) = extract_val(rest, "yaw: ").context("failed to parse yaw")?;
        Ok(Attitude { roll, pitch, yaw })
    }
}

/// Parse a float value immediately following `key` in `line`.
/// Stops at the first character that is not a digit, dot, or minus sign,
/// which correctly handles the UTF-8 degree symbol '°' (0xC2 0xB0).
fn extract_val<'a>(line: &'a str, key: &str) -> Option<(f32, &'a str)> {
    let (_, rest) = line.trim().split_once(key)?;
    let end = rest
        .find(|c: char| !c.is_ascii_digit() && c != '.' && c != '-')
        .unwrap_or(rest.len());
    Some((rest[..end].parse().ok()?, &rest[end..]))
}

fn draw_attitude(att: Attitude) {
    // macroquad axes (camera at +Z looking toward origin):
    //   X = right  → pitch axis (tilts forward/back)
    //   Y = up     → yaw axis   (spins left/right)
    //   Z = toward viewer → roll axis (tilts left/right)
    let rot = Quat::from_rotation_y(att.yaw.to_radians())
        * Quat::from_rotation_x(att.pitch.to_radians())
        * Quat::from_rotation_z(att.roll.to_radians());

    // Generate 8 corners of a unit cube from all ±0.5 sign combinations.
    let corners: Vec<Vec3> = (0..8)
        .map(|i| {
            let x = if i & 1 != 0 { 0.5 } else { -0.5 };
            let y = if i & 2 != 0 { 0.5 } else { -0.5 };
            let z = if i & 4 != 0 { 0.5 } else { -0.5 };
            rot * Vec3::new(x, y, z)
        })
        .collect();

    // Draw an edge between every pair of corners that differ in exactly one bit
    // (i.e. one coordinate), which gives all 12 edges of a cube.
    for a in 0..8_usize {
        for b in (a + 1)..8_usize {
            if (a ^ b).count_ones() == 1 {
                draw_line_3d(corners[a], corners[b], WHITE);
            }
        }
    }

    // Body-frame axes: X=red (pitch), Y=green (yaw), Z=blue (roll).
    let o = Vec3::ZERO;
    draw_line_3d(o, rot * Vec3::new(0.75, 0.0, 0.0), RED);
    draw_line_3d(o, rot * Vec3::new(0.0, 0.75, 0.0), GREEN);
    draw_line_3d(o, rot * Vec3::new(0.0, 0.0, 0.75), BLUE);
}

fn window_conf() -> Conf {
    Conf {
        window_title: "IMU Attitude Visualizer".to_string(),
        window_width: 900,
        window_height: 600,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let state: Arc<Mutex<Attitude>> = Arc::new(Mutex::new(Attitude::default()));

    // Spawn a background thread to read stdin line by line.
    // Use `2>&1 | cargo run --manifest-path visualizer/Cargo.toml` to pipe
    // espflash monitor output into this program.
    {
        let state = Arc::clone(&state);
        thread::spawn(move || {
            let stdin = std::io::stdin();
            for line in stdin.lock().lines().map_while(Result::ok) {
                // Passthrough to stderr so raw serial is still visible.
                eprintln!("{}", line);
                if let Ok(att) = Attitude::from_str(&line) {
                    *state.lock().expect("failed to acquire stdin lock") = att;
                }
            }
        });
    }

    loop {
        clear_background(Color::from_rgba(12, 12, 22, 255));

        let att = *state.lock().unwrap();

        set_camera(&Camera3D {
            position: Vec3::new(0.0, 1.4, 3.6),
            target: Vec3::ZERO,
            up: Vec3::Y,
            ..Default::default()
        });

        draw_attitude(att);

        set_default_camera();

        // HUD: roll/pitch/yaw readout.
        let hud_x = 20.0_f32;
        let font_sz = 26.0_f32;
        let line_h = 32.0_f32;
        // HUD colors match body axis lines: roll=blue(Z), pitch=red(X), yaw=green(Y)
        draw_text(
            format!("roll  {:+7.1}\u{b0}", att.roll),
            hud_x,
            36.0,
            font_sz,
            BLUE,
        );
        draw_text(
            format!("pitch {:+7.1}\u{b0}", att.pitch),
            hud_x,
            36.0 + line_h,
            font_sz,
            RED,
        );
        draw_text(
            format!("yaw   {:+7.1}\u{b0}", att.yaw),
            hud_x,
            36.0 + 2.0 * line_h,
            font_sz,
            GREEN,
        );

        // Axis legend at the bottom.
        draw_text(
            "Z\u{2192} blue = roll    X red = pitch    Y\u{2191} green = yaw",
            20.0,
            screen_height() - 14.0,
            16.0,
            GRAY,
        );

        next_frame().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_extract() {
        let str = "roll: -1.9°  pitch: -2.9°  yaw: -41.3°";
        let (val, rest) = extract_val(str, "roll: ").unwrap();
        assert_eq!(val, -1.9f32);
        let (val, rest) = extract_val(rest, "pitch: ").unwrap();
        assert_eq!(val, -2.9f32);
        let (val, _) = extract_val(rest, "yaw: ").unwrap();
        assert_eq!(val, -41.3f32);
    }
    #[test]
    fn test_line() {
        let str = "qx: 1 roll: -1.9°  pitch: -2.9°  yaw: -41.3°";
        let a = Attitude::from_str(str).unwrap();
        assert_eq!(a.roll, -1.9f32);
        assert_eq!(a.pitch, -2.9f32);
        assert_eq!(a.yaw, -41.3f32);
    }
}
