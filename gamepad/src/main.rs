use gilrs::{Event, Gilrs};
use tracing::info;

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();

    let mut gilrs = Gilrs::new().map_err(|e| anyhow::anyhow!("{}", e))?; // their error type doesn't implement Error or Sync?

    info!("starting up gamepad");
    // Iterate over all connected gamepads
    for (_id, gamepad) in gilrs.gamepads() {
        info!("{} is {:?}", gamepad.name(), gamepad.power_info());
    }

    loop {
        while let Some(Event {
            id, event, time, ..
        }) = gilrs.next_event_blocking(Some(std::time::Duration::from_millis(10)))
        {
            info!("{:?} New event from {}: {:?}", time, id, event);
        }
    }

    //     // You can also use cached gamepad state
    //     if let Some(gamepad) = active_gamepad.map(|id| gilrs.gamepad(id)) {
    //         if gamepad.is_pressed(Button::South) {
    //             println!("Button South is pressed (XBox - A, PS - X)");
    //         }
    //     }
    // }
}
