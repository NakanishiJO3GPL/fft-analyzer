#![no_main]
#![no_std]

use fft_analyzer as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    for i in 0..=10 {
        defmt::println!("i: {:?}", i);
    }

    fft_analyzer::exit()
}
