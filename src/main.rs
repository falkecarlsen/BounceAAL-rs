// include bindings from TOF driver


mod vl53l5cx;

use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::gpio::{AnyIOPin, InputPin, OutputPin, PinDriver};
use esp_idf_hal::i2c::{I2c, I2cConfig, I2cDriver, I2cSlaveConfig, I2cSlaveDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::units::Hertz;

use embedded_graphics::pixelcolor::Rgb555;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use esp_idf_hal::sys::EspError;
use esp_idf_svc::sys::esp;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const OLED_ADDR: u8 = 0x22;
const OLED_BUFFER_SIZE: usize = 128;

fn i2c_master_init<'d>(
    i2c: impl Peripheral<P = impl I2c> + 'd,
    sda: AnyIOPin,
    scl: AnyIOPin,
    baudrate: Hertz,
) -> anyhow::Result<I2cDriver<'d>> {
    let config = I2cConfig::new().baudrate(baudrate);
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;
    Ok(driver)
}

fn i2c_slave_init<'d>(
    i2c: impl Peripheral<P = impl I2c> + 'd,
    sda: AnyIOPin,
    scl: AnyIOPin,
    buflen: usize,
    slave_addr: u8,
) -> anyhow::Result<I2cSlaveDriver<'d>> {
    let config = I2cSlaveConfig::new()
        .rx_buffer_length(buflen)
        .tx_buffer_length(buflen);
    let driver = I2cSlaveDriver::new(i2c, sda, scl, slave_addr, &config)?;
    Ok(driver)
}

#[cfg(not(esp32))]
fn main() -> anyhow::Result<()> {
    eprintln!("Firmware only tested on ESP32");
    Ok(())
}

#[cfg(esp32)]
fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();
    esp_idf_svc::sys::link_patches();

    let peripherals = Peripherals::take()?;

    let mut i2c_master = i2c_master_init(
        peripherals.i2c0,
        peripherals.pins.gpio21.into(),
        peripherals.pins.gpio22.into(),
        400.kHz().into(),
    )?;

    let mut led = PinDriver::output(peripherals.pins.gpio2)?;

    unsafe {
        vl53l5cx::range(&mut i2c_master);
    }

    let interface = I2CDisplayInterface::new(i2c_master);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let blackout_style = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::Off)
        .build();

    Text::with_baseline(
        concat!("BounceAAL (", env!("VERGEN_GIT_DESCRIBE"), ")"),
        Point::zero(),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    // write git commit as text to indicate version, using vergen
    Text::with_baseline(
        "warming up sensors",
        Point::new(0, 16),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    display.flush().unwrap();
    let mut c = 0;
    loop {
        led.set_high().unwrap();
        Rectangle::new(Point::new(12, 32), Size::new(128, 16))
            .into_styled(blackout_style)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(
            &*format!("c: {}", c),
            Point::new(0, 32),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();
        c += 1;
        display.flush().unwrap();
        led.set_low().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}
