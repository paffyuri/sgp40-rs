#![no_std]
#![no_main]

mod fmt;

use core::mem::MaybeUninit;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use fmt::info;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_stm32::i2c::I2c;
use embassy_stm32::{bind_interrupts, SharedData};
use embassy_stm32::peripherals::I2C1;
use embassy_stm32::time::Hertz;
use static_cell::StaticCell;
use sgp40::Sgp40;
use embassy_stm32::mode::Async;

static I2C_BUS: StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, Async>>> = StaticCell::new();

bind_interrupts!(struct Irqs {
    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C1>;
});

#[unsafe(link_section = ".shared_data")]
static SHARED_DATA: MaybeUninit<SharedData> = MaybeUninit::uninit();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init_primary(Default::default(), &SHARED_DATA);

    let i2c = I2c::new(p.I2C1, p.PB8, p.PB7, Irqs, p.DMA1_CH6, p.DMA1_CH7, Hertz(100_000), Default::default());
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));
    let i2c_dev = I2cDevice::new(i2c_bus);
    let mut sgp = Sgp40::new(i2c_dev, 0x59, embassy_time::Delay);

    loop {
        if let Ok(raw) = sgp.measure_raw().await {
            info!("Raw: {:?}", raw);
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}
