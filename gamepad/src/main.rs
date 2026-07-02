use std::{net::UdpSocket, time::Duration};

use gilrs::{Axis, Button, Event, EventType, Gilrs};
use libs::control::ControlPacket;
use tracing::info;

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();

    let mut gilrs = Gilrs::new().map_err(|e| anyhow::anyhow!("{}", e))?;
    info!("starting up gamepad");
    for (_id, gamepad) in gilrs.gamepads() {
        info!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    let (ip, port) = (libs::get_ip(), libs::get_port());
    let socket = UdpSocket::bind("0.0.0.0:0")?;
    socket.connect((ip, port))?;
    info!("sending to {}:{}", ip, port);

    let mut pkt = ControlPacket::new(0, 0.0, 0.0, 0.0, false);

    loop {
        // block up to 20ms waiting for input, then send current state regardless
        while let Some(Event { event, .. }) =
            gilrs.next_event_blocking(Some(Duration::from_millis(20)))
        {
            match event {
                // upper half only: center = 0%, full up = 100%
                EventType::AxisChanged(Axis::RightStickY, value, _) => {
                    pkt.throttle = (value.max(0.0) * 100.0) as u8;
                    info!("throttle: {}", pkt.throttle);
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
                }
                EventType::Connected => info!("gamepad connected"),
                _ => {}
            }
        }
        if let Err(e) = socket.send(&pkt.to_bytes()) {
            info!("send error: {e}");
        }
    }
}
