use core::{net::Ipv4Addr, str::FromStr};

use embassy_executor::Spawner;
use embassy_net::{Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use esp_hal::{peripherals::WIFI, rng::Rng};
use esp_radio::wifi::{
    AccessPointStationEventInfo, Config, ControllerConfig, Interface, WifiController,
    ap::AccessPointConfig,
};
use static_cell::StaticCell;

pub const SSID: &str = "esp-quad";
pub const GW_IP: &str = "192.168.4.1";
const GW_IP_ADDR_ENV: Option<&'static str> = option_env!("GATEWAY_IP");

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        STATIC_CELL.uninit().write($val)
    }};
}

pub struct AP {
    pub stack: Stack<'static>,
}

impl AP {
    /// Initialise the WiFi AP and return an AP holding the network stack.
    /// Call esp_alloc::heap_allocator! in main before this.
    pub async fn init(wifi: WIFI<'static>, spawner: Spawner) -> Self {
        let gw_ip =
            Ipv4Addr::from_str(GW_IP_ADDR_ENV.unwrap_or(GW_IP)).expect("no IP given for AP");
        let ap_config = Config::AccessPoint(AccessPointConfig::default().with_ssid(SSID));

        let (controller, interfaces) = esp_radio::wifi::new(
            wifi,
            ControllerConfig::default().with_initial_config(ap_config),
        )
        .expect("failed to initialise WiFi");

        let device = interfaces.access_point;

        let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(gw_ip, 24),
            gateway: Some(gw_ip),
            dns_servers: Default::default(),
        });

        let rng = Rng::new();
        let seed = (rng.random() as u64) << 32 | rng.random() as u64;

        let (stack, runner) = embassy_net::new(
            device,
            net_config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed,
        );

        spawner.spawn(ap_task(controller).expect("ap_task already spawned"));
        spawner.spawn(net_task(runner).expect("net_task already spawned"));

        stack.wait_config_up().await;
        defmt::info!(
            "AP up - SSID: {}, IP: {}",
            SSID,
            GW_IP_ADDR_ENV.unwrap_or(GW_IP)
        );

        Self { stack }
    }
}

#[embassy_executor::task]
async fn ap_task(controller: WifiController<'static>) {
    loop {
        match controller
            .wait_for_access_point_connected_event_async()
            .await
        {
            Ok(AccessPointStationEventInfo::Connected(info)) => {
                defmt::info!(
                    "[AP TASK] station connected: {:?}",
                    defmt::Debug2Format(&info)
                )
            }
            Ok(AccessPointStationEventInfo::Disconnected(info)) => {
                defmt::info!(
                    "[AP TASK] station disconnected: {:?}",
                    defmt::Debug2Format(&info)
                )
            }
            Err(err) => {
                defmt::error!("[AP TASK] wifi error: {:?}", defmt::Debug2Format(&err))
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await
}
