//! Test communication between a master and a slave I2C on the same device
//!
//! Wiring required, but can be changed in main():
//! - GPIO21 to GPIO18
//! - GPIO22 to GPIO19
//!
//! ESP32-C2/C3 does not have two I2C peripherals, so this ecample will not work.
//!
//! Description:
//! Consists of three parts:
//! 1. Simple master write, master writes 8 bytes and print out what slave receives
//! 2. Simple master read, master read 8 bytes and print out.
//! 3. Read/write register, write a value to a register addr and read it back.
//!
#![allow(unused)]

// include bindings from TOF driver
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

use esp_idf_hal::delay::BLOCK;
use esp_idf_hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_hal::i2c::{I2c, I2cConfig, I2cDriver, I2cSlaveConfig, I2cSlaveDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::units::Hertz;

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use esp_idf_svc::sys::esp;

const SLAVE_ADDR: u8 = 0x22;
const SLAVE_BUFFER_SIZE: usize = 128;

fn i2c_master_init<'d>(
    i2c: impl Peripheral<P=impl I2c> + 'd,
    sda: AnyIOPin,
    scl: AnyIOPin,
    baudrate: Hertz,
) -> anyhow::Result<I2cDriver<'d>> {
    let config = I2cConfig::new().baudrate(baudrate);
    let driver = I2cDriver::new(i2c, sda, scl, &config)?;
    Ok(driver)
}

fn i2c_slave_init<'d>(
    i2c: impl Peripheral<P=impl I2c> + 'd,
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
    println!("Test only configured for ESP32");
    Ok(())
}

fn init_tof_config() -> VL53L5CX_Configuration {
    VL53L5CX_Configuration {
        platform: VL53L5CX_Platform { address: 0x53 },
        streamcount: 0,
        data_read_size: 0,
        default_configuration: std::ptr::null_mut(),
        default_xtalk: std::ptr::null_mut(),
        offset_data: [0; 488],
        xtalk_data: [0; 776],
        temp_buffer: [0; 1452],
        is_auto_stop_enabled: 0,
    }
}

extern fn range(i2c_master: &I2cDriver) -> u8 {
    /*********************************/
    /*   VL53L5CX ranging variables  */
    /*********************************/
    unsafe {
        let mut dev: VL53L5CX_Configuration;
        let mut results: VL53L5CX_ResultsData;
        let mut status: u8 = 0;
        let mut iter: u8 = 0;
        let mut is_alive: u8 = 0;
        let mut is_ready: u8 = 0;
        let mut integration_time_ms: u32 = 0;

        /* (Optional) Reset sensor toggling PINs (see platform, not in API) */
        //Reset_Sensor(&(Dev.platform));

        status = vl53l5cx_set_i2c_address(&mut dev, 0x20);
        println!("check tof alive");
        status = vl53l5cx_is_alive(&mut dev, &mut is_alive);
        if (is_alive == 0 || status != 0) {
            println!("VL53L5CX not detected at requested address");
            return status;
        }
        return 0;
    }
    /*
    /* (Mandatory) Init VL53L5CX sensor */
    ESP_LOGI(TAG, "init sensor");
    status = vl53l5cx_init(&Dev);
    if (status) {
        ESP_LOGI(TAG, "VL53L5CX ULD Loading failed");
        return status;
    }

    ESP_LOGI(TAG, "VL53L5CX ULD ready ! (Version : %s)",
             VL53L5CX_API_REVISION);

    /*********************************/
    /*        Set some params        */
    /*********************************/

    /* Set resolution in 8x8. WARNING : As others settings depend to this
     * one, it must be the first to use.
     */
    status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8);
    if (status) {
        ESP_LOGI(TAG, "vl53l5cx_set_resolution failed, status %u", status);
        return status;
    }

    /* Set ranging frequency to 10Hz.
     * Using 4x4, min frequency is 1Hz and max is 60Hz
     * Using 8x8, min frequency is 1Hz and max is 15Hz
     */
    status = vl53l5cx_set_ranging_frequency_hz(&Dev, 10);
    if (status) {
        ESP_LOGI(TAG, "vl53l5cx_set_ranging_frequency_hz failed, status %u", status);
        return status;
    }

    /* Set target order to closest */
    status = vl53l5cx_set_target_order(&Dev, VL53L5CX_TARGET_ORDER_CLOSEST);
    if (status) {
        ESP_LOGI(TAG, "vl53l5cx_set_target_order failed, status %u", status);
        return status;
    }

    /* Get current integration time */
    status = vl53l5cx_get_integration_time_ms(&Dev, &integration_time_ms);
    if (status) {
        ESP_LOGI(TAG, "vl53l5cx_get_integration_time_ms failed, status %u", status);
        return status;
    }
    ESP_LOGI(TAG, "Current integration time is : %ld ms", integration_time_ms);

    /*********************************/
    /*         Ranging loop          */
    /*********************************/

    status = vl53l5cx_start_ranging(&Dev);

    loop = 0;
    while (loop < 10) {
        /* Use polling function to know when a new measurement is ready.
         * Another way can be to wait for HW interrupt raised on PIN A3
         * (GPIO 1) when a new measurement is ready */

        status = vl53l5cx_check_data_ready(&Dev, &isReady);

        if (isReady) {
            vl53l5cx_get_ranging_data(&Dev, &Results);

            /* As the sensor is set in 8x8 mode, we have a total
             * of 64 zones to print. For this example, only the data of
             * first zone are print */
            ESP_LOGI(TAG, "Print data no : %3u", Dev.streamcount);
            for (i = 0; i < 64; i++) {
                ESP_LOGI(TAG, "Zone : %3d, Status : %3u, Distance : %4d mm",
                         i,
                         Results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
                         Results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i]);
            }
            ESP_LOGI(TAG, "");
            loop++;
        }

        /* Wait a few ms to avoid too high polling (function in platform
         * file, not in API) */
        WaitMs(&(Dev.platform), 5);
    }

    status = vl53l5cx_stop_ranging(&Dev);
    ESP_LOGI(TAG, "End of ULD demo");
    return status;

     */
}

#[cfg(esp32)]
fn main() -> anyhow::Result<()> {
    esp_idf_hal::sys::link_patches();

    //let dp = stm32::Peripherals::take().unwrap();
    let peripherals = Peripherals::take()?;

    let mut i2c_master = i2c_master_init(
        peripherals.i2c0,
        peripherals.pins.gpio21.into(),
        peripherals.pins.gpio22.into(),
        400.kHz().into(),
    )?;

    range(&i2c_master);

    let interface = I2CDisplayInterface::new(i2c_master);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline(concat!("BounceAAL (", env!("VERGEN_GIT_DESCRIBE"), ")"), Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    // write git commit as text to indicate version, using vergen
    Text::with_baseline("warming up sensors", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    loop {
        // trigger watchdog
        std::thread::sleep(std::time::Duration::from_millis(2000));
    }
}
