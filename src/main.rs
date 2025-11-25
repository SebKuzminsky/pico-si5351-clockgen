//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use fugit::RateExtU32;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use si5351::Si5351; // pull in the Si5351 Trait

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    let mut led_pin = pins.led.into_push_pull_output();

    //
    // I2C for SI5351 Clock Generator
    //

    let mut i2c = bsp::hal::I2C::i2c0(
        pac.I2C0,
        pins.gpio16.reconfigure(),
        pins.gpio17.reconfigure(),
        400.kHz(),
        &mut pac.RESETS,
        125_000_000.Hz(),
    );

    // Scan for devices on the bus by attempting to read from them
    for i in 0..=127u8 {
        let mut readbuf: [u8; 1] = [0; 1];
        let result = i2c.read(i, &mut readbuf);
        if result.is_ok() {
            info!("i2c device found at address 0x{:02x}", i);
        }
    }

    let mut clock = si5351::Si5351Device::new(i2c, false, 25_000_000);
    clock.init(si5351::CrystalLoad::_10).unwrap();

    clock.set_clock_enabled(si5351::ClockOutput::Clk0, false);
    clock.set_clock_enabled(si5351::ClockOutput::Clk1, false);

    let freq = 10_000_000;
    let clk0_phase_offset: u8 = 0x00;
    let mut clk1_phase_offset: u8 = 0x00;

    clock
        .set_frequency(si5351::PLL::A, si5351::ClockOutput::Clk0, freq)
        .unwrap();
    clock
        .set_phase_offset(si5351::ClockOutput::Clk0, clk0_phase_offset)
        .unwrap();
    clock
        .flush_clock_control(si5351::ClockOutput::Clk0)
        .unwrap();
    clock.set_clock_enabled(si5351::ClockOutput::Clk0, true);

    loop {
        clock.set_clock_enabled(si5351::ClockOutput::Clk1, false);
        clock
            .set_frequency(si5351::PLL::A, si5351::ClockOutput::Clk1, freq)
            .unwrap();
        clock
            .set_phase_offset(si5351::ClockOutput::Clk1, clk1_phase_offset)
            .unwrap();
        clock
            .flush_clock_control(si5351::ClockOutput::Clk1)
            .unwrap();
        clock.set_clock_enabled(si5351::ClockOutput::Clk1, true);

        info!("phase offset {}", clk1_phase_offset);
        clk1_phase_offset += 10;
        if clk1_phase_offset > 100 {
            clk1_phase_offset = 0;
        }

        // info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        // info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}
