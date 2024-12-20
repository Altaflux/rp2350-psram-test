//! # Alloc Example
//!
//! Uses alloc to create a Vec.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED. It may need to be adapted to your particular board layout
//! and/or pin assignment.
//!
//! While blinking the LED, it will continuously push to a `Vec`, which will
//! eventually lead to a panic due to an out of memory condition.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use embedded_alloc::Heap;

#[global_allocator]
static ALLOCATOR: Heap = Heap::empty();

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal::{self as hal, Clock};

// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2350 peripherals, then blinks the LED in an
/// infinite loop where the duration indicates how many items were allocated.

mod psram;

#[hal::entry]
fn main() -> ! {



    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    //PSRAM INITIALIZATION
    let _ = pins.gpio47.into_function::<hal::gpio::FunctionXipCs1>();
    let psram_size = psram::psram_init(
        clocks.peripheral_clock.freq().to_Hz(),
        &pac.QMI,
        &pac.XIP_CTRL,
    );
    
    //USE PSRAM AS HEAP SPACE
    {
        const PSRAM_ADDRESS: usize = 0x11000000;
        unsafe { ALLOCATOR.init(PSRAM_ADDRESS, psram_size as usize) }
    }

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();

    let mut xs = Vec::new();
    xs.push(1);

    // Blink the LED at 1 Hz
    loop {
        led_pin.set_high().unwrap();
        let len = xs.len() as u32;
        timer.delay_ms(100 * len);
        xs.push(1);
        led_pin.set_low().unwrap();
        timer.delay_ms(100 * len);
        xs.push(1);
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Memory Allocation Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
