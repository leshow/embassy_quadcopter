#[cfg(feature = "telemetry")]
use std::io::ErrorKind;
use std::{net::UdpSocket, thread, time::Duration};

use gilrs::{Axis, Button, Event, EventType, Gilrs};
use libs::control::ControlPacket;
#[cfg(feature = "telemetry")]
use libs::control::{TELEMETRY_SIZE, TelemetryPacket};
use tracing::{info, warn};

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();

    let mut gilrs = Gilrs::new().map_err(|e| anyhow::anyhow!("{}", e))?;
    info!("starting up ground_control");
    for (_id, gamepad) in gilrs.gamepads() {
        info!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    loop {
        let (ip, port) = (libs::get_ip(), libs::get_port());
        let socket = match UdpSocket::bind("0.0.0.0:0").and_then(|s| {
            s.connect((ip, port))?;
            Ok(s)
        }) {
            Ok(s) => {
                info!("connected to {}:{}", ip, port);
                // short timeout so a missing telemetry reply doesn't stall the input loop
                #[cfg(feature = "telemetry")]
                s.set_read_timeout(Some(Duration::from_millis(20))).ok();
                s
            }
            Err(e) => {
                info!("connect failed: {e}, retrying in 200ms...");
                thread::sleep(Duration::from_millis(200));
                continue;
            }
        };

        let mut pkt = ControlPacket::new(0, 0.0, 0.0, 0.0, false);

        'inner: loop {
            // block up to 20ms waiting for input, then send current state regardless
            while let Some(Event { event, .. }) =
                gilrs.next_event_blocking(Some(Duration::from_millis(20)))
            {
                match event {
                    // upper half only: center = 0%, full up = 100%
                    EventType::AxisChanged(Axis::LeftStickY, value, _) => {
                        pkt.throttle = (value.max(0.0) * 100.0) as u8;
                        info!("throttle: {}", pkt.throttle);
                    }
                    EventType::AxisChanged(Axis::RightStickX, value, _) => {
                        pkt.roll = value;
                        info!("roll: {}", value);
                    }
                    EventType::AxisChanged(Axis::RightStickY, value, _) => {
                        pkt.pitch = value;
                        info!("pitch: {}", value);
                    }
                    EventType::AxisChanged(Axis::LeftStickX, val, _) => {
                        pkt.yaw = val;
                        info!("yaw: {}", val);
                    }
                    // start button toggles arm
                    EventType::ButtonPressed(Button::Start, _) => {
                        pkt.set_armed(!pkt.armed());
                        info!("armed: {}", pkt.armed());
                    }
                    EventType::Disconnected => {
                        info!("gamepad disconnected — disarming");
                        pkt.throttle = 0;
                        pkt.set_armed(false);
                        break 'inner;
                    }
                    EventType::Connected => info!("gamepad connected"),
                    _ => {}
                }
            }
            if let Err(e) = socket.send(&pkt.to_bytes()) {
                info!("send error: {e}");
                break 'inner;
            }

            #[cfg(feature = "telemetry")]
            {
                let mut tbuf = [0u8; TELEMETRY_SIZE];
                match socket.recv(&mut tbuf) {
                    Ok(n) if n == TELEMETRY_SIZE => {
                        if let Some(t) = TelemetryPacket::from_bytes(&tbuf) {
                            #[cfg(not(feature = "telemetry-verbose"))]
                            let msg = format!(
                                "telemetry: roll={:.1} pitch={:.1} yaw={:.1} armed={} failsafe={} fifo_overflow={} motors={:?}",
                                t.roll,
                                t.pitch,
                                t.yaw,
                                t.armed(),
                                t.failsafe(),
                                t.fifo_overflow(),
                                t.motors
                            );
                            #[cfg(feature = "telemetry-verbose")]
                            let msg = format!(
                                "telemetry: roll={:.1} pitch={:.1} yaw={:.1} armed={} failsafe={} fifo_overflow={} motors={:?} gyro={:?}",
                                t.roll,
                                t.pitch,
                                t.yaw,
                                t.armed(),
                                t.failsafe(),
                                t.fifo_overflow(),
                                t.motors,
                                t.gyro
                            );

                            if t.fifo_overflow() {
                                warn!("{msg}");
                            } else {
                                info!("{msg}");
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                    Err(e) => info!("telemetry recv error: {e}"),
                }
            }
        }
    }
}
