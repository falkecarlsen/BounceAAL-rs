// include bindings from TOF driver
#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

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

fn init_tof_results() -> VL53L5CX_ResultsData {
    VL53L5CX_ResultsData {
        silicon_temp_degc: 0,
        ambient_per_spad: [0u32; 64usize],
        nb_target_detected: [0u8; 64usize],
        nb_spads_enabled: [0u32; 64usize],
        signal_per_spad: [0u32; 64usize],
        range_sigma_mm: [0u16; 64usize],
        distance_mm: [0i16; 64usize],
        reflectance: [0u8; 64usize],
        target_status: [0u8; 64usize],
        motion_indicator: VL53L5CX_ResultsData__bindgen_ty_1 {
            global_indicator_1: 0,
            global_indicator_2: 0,
            status: 0,
            nb_of_detected_aggregates: 0,
            nb_of_aggregates: 0,
            spare: 0,
            motion: [0u32; 32usize],
        },

    }
}

unsafe fn range(i2c_master: &mut I2cDriver) -> u8 {
    /*********************************/
    /*   VL53L5CX ranging variables  */
    /*********************************/
    let mut dev: VL53L5CX_Configuration = init_tof_config();
    let mut results: VL53L5CX_ResultsData = init_tof_results();
    let mut status: u8 = 0;
    let mut iter: u8 = 0;
    let mut is_alive: u8 = 0;
    let mut is_ready: u8 = 0;
    let mut integration_time_ms: u32 = 0;

    /* (Optional) Reset sensor toggling PINs (see platform, not in API) */
    Reset_Sensor(&mut (dev.platform));

    println!("check tof alive");
    status = vl53l5cx_is_alive(&mut dev, &mut is_alive);
    if (is_alive == 0 || status != 0) {
        println!("VL53L5CX not detected at requested address");
        return status;
    }

    /* (Mandatory) Init VL53L5CX sensor */
    println!("init sensor");
    status = vl53l5cx_init(&mut dev);
    if (status != 0) {
        println!("VL53L5CX ULD Loading failed");
        return status;
    }

    println!(
        "VL53L5CX ULD ready ! (Version : {:?})",
        VL53L5CX_API_REVISION
    );

    /*********************************/
    /*        Set some params        */
    /*********************************/

    /* Set resolution in 8x8. WARNING : As others settings depend to this
     * one, it must be the first to use.
     */
    /*
       #define VL53L5CX_RESOLUTION_4X4			((uint8_t) 16U)
       #define VL53L5CX_RESOLUTION_8X8			((uint8_t) 64U)
    */
    status = vl53l5cx_set_resolution(&mut dev, 16);
    if (status != 0) {
        println!("vl53l5cx_set_resolution failed, status {}", status);
        return status;
    }

    /* Set ranging frequency to 10Hz.
     * Using 4x4, min frequency is 1Hz and max is 60Hz
     * Using 8x8, min frequency is 1Hz and max is 15Hz
     */
    status = vl53l5cx_set_ranging_frequency_hz(&mut dev, 10);
    if (status != 0) {
        println!(
            "vl53l5cx_set_ranging_frequency_hz failed, status {}",
            status
        );
        return status;
    }

    /* Set target order to closest */
    /*
       #define VL53L5CX_TARGET_ORDER_CLOSEST		((uint8_t) 1U)
       #define VL53L5CX_TARGET_ORDER_STRONGEST		((uint8_t) 2U)
    */
    status = vl53l5cx_set_target_order(&mut dev, 1);
    if (status != 0) {
        println!("vl53l5cx_set_target_order failed, status {}", status);
        return status;
    }

    /* Get current integration time */
    status = vl53l5cx_get_integration_time_ms(&mut dev, &mut integration_time_ms);
    if (status != 0) {
        println!("vl53l5cx_get_integration_time_ms failed, status {}", status);
        return status;
    }
    println!("Current integration time is : {} ms", integration_time_ms);

    /*********************************/
    /*         Ranging loop          */
    /*********************************/

    status = vl53l5cx_start_ranging(&mut dev);

    let mut iter = 0;
    while (iter < 10) {
        /* Use polling function to know when a new measurement is ready.
         * Another way can be to wait for HW interrupt raised on PIN A3
         * (GPIO 1) when a new measurement is ready */

        status = vl53l5cx_check_data_ready(&mut dev, &mut is_ready);

        if (is_ready != 0) {
            vl53l5cx_get_ranging_data(&mut dev, &mut results);

            /* As the sensor is set in 8x8 mode, we have a total
             * of 64 zones to print. For this example, only the data of
             * first zone are print */
            println!("Print data no : {}", dev.streamcount);
            for i in 0..64 {
                //println!("Zone : %3d, Status : %3u, Distance : %4d mm",
                /*
                println!("Zone : {}, Status : {}, Distance : {} mm",
                         i,
                         results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * i],
                         results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i]);
                         */
            }
            iter += 1;
        }

        /* Wait a few ms to avoid too high polling (function in platform
         * file, not in API) */
        WaitMs(&mut (dev.platform), 5);
    }

    status = vl53l5cx_stop_ranging(&mut dev);
    println!("End of ULD demo");
    return status;
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
        range(&mut i2c_master);
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
