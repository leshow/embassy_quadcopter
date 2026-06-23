use std::net::UdpSocket;

use anyhow::Context;
use gilrs::{Button, Event, Filter, Gilrs, ev};
use tracing::info;

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();

    let mut gilrs = Gilrs::new().map_err(|e| anyhow::anyhow!("{}", e))?; // their error type doesn't implement Error or Sync?

    // Iterate over all connected gamepads
    for (_id, gamepad) in gilrs.gamepads() {
        info!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    while let Some(Event {
        id, event, time, ..
    }) = gilrs.next_event_blocking(None)
    {
        info!("{:?} New event from {}: {:?}", time, id, event);
    }

    Ok(())

    //     // You can also use cached gamepad state
    //     if let Some(gamepad) = active_gamepad.map(|id| gilrs.gamepad(id)) {
    //         if gamepad.is_pressed(Button::South) {
    //             println!("Button South is pressed (XBox - A, PS - X)");
    //         }
    //     }
    // }
}
