#![no_std]

use core::{net::Ipv4Addr, str::FromStr};

pub const SSID_DEFAULT: &str = "esp-quad";
pub const GW_IP_DEFAULT: &str = "192.168.4.1";
pub const UDP_PORT_DEFAULT: u16 = 4444;
pub const UDP_PORT_ENV: Option<&'static str> = option_env!("UDP_PORT");
pub const GW_IP_ADDR_ENV: Option<&'static str> = option_env!("GATEWAY_IP");
// set AP_PASSWORD at build time; if unset the AP is open
pub const AP_PASSWORD: Option<&'static str> = option_env!("AP_PASSWORD");

pub fn get_ip() -> Ipv4Addr {
    Ipv4Addr::from_str(GW_IP_ADDR_ENV.unwrap_or(GW_IP_DEFAULT)).expect("invalid GATEWAY_IP")
}

pub fn get_port() -> u16 {
    UDP_PORT_ENV
        .map(|o| o.parse::<u16>())
        .transpose()
        .expect("invalid UDP_PORT")
        .unwrap_or(UDP_PORT_DEFAULT)
}

pub mod control;
