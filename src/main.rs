#![no_main]
#![no_std]

use fft_analyzer as _; // global logger + panicking-behavior + memory layout
use cortex_m_rt::entry;
use defmt::*;
use embassy_executor::Executor;
use embassy_stm32::{
    Config, 
    bind_interrupts, 
    dma, 
    peripherals, 
    i2s,
    time::mhz,
    mode::Async,
};
use static_cell::StaticCell;
use grounded::uninit::GroundedArrayCell;

#[unsafe(link_section = ".sram3")]
static mut SRAM3: GroundedArrayCell<u8, 1024> = GroundedArrayCell::uninit();

#[cortex_m_rt::entry]
fn main() -> ! {
    for i in 0..=10 {
        defmt::println!("i: {:?}", i);
    }

    fft_analyzer::exit()
}
