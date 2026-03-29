#![no_main]
#![no_std]

use fft_analyzer as _; // global logger + panicking-behavior + memory layout
use defmt::*;
use embassy_stm32 as hal;
use hal::{*, time::*, sai::*};
use grounded::uninit::GroundedArrayCell;


const SAMPLE_RATE: u32 = 48_000;
const BLOCK_SIZE: usize = 2048;
const HALF_DMA_BUFFER_SIZE: usize = BLOCK_SIZE * 2; // 2ch
const DMA_BUFFER_SIZE: usize = HALF_DMA_BUFFER_SIZE * 2; // 2 half blocks

#[unsafe(link_section = ".sram1")]
static mut TX_BUFFER: GroundedArrayCell<u32, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();
#[unsafe(link_section = ".sram1")]
static mut RX_BUFFER: GroundedArrayCell<u32, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => dma::InterruptHandler<hal::peripherals::DMA1_CH0>;
    DMA1_STREAM1 => dma::InterruptHandler<hal::peripherals::DMA1_CH1>;
});

fn mclk_div_from_u8(div: u8) -> MasterClockDivider {
    MasterClockDivider::from_bits(div)
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    println!("--- Starting FFT Analyzer ---");

    let mut config = hal::Config::default();
    {
        use hal::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: mhz(8),               // 8MHz external crystal
            mode: HseMode::Bypass,      // Bypass mode since we're using an external oscillator
        });
        config.rcc.hsi = None;
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,     // HSE : 8MHz
            prediv: PllPreDiv::DIV2,    // PLL prediv: 8MHz / 2 = 4MHz   
            mul: PllMul::MUL200,        // PLL mul: 4MHz * 200 = 800MHz
            divp: Some(PllDiv::DIV2),   // PLL div for P clock (SYSCLK): 800MHz / 2 = 400MHz
            divq: Some(PllDiv::DIV5),   // PLL div for Q clock (for USB/I2S/SAI/QSPI)
            divr: Some(PllDiv::DIV2),   // PLL div for R clock 
        });

        // PLL3 is used for I2S clock generation. 
        // We want to generate a 12.288MHz clock for 48kHz sample rate with 256x oversampling 
        // The frequency minimum is 48kHz * 256 = 12.288MHz.
        config.rcc.pll3 = Some(Pll {
            source: PllSource::HSE,     // HSE : 8MHz
            prediv: PllPreDiv::DIV3,    // PLL prediv: 8MHz / 3 = 2.6667MHz   
            mul: PllMul::MUL295,        // PLL mul: 2.6667MHz * 295 = 786.667MHz
            divp: Some(PllDiv::DIV64),  // PLL div for P clock : 786.667MHz / 64 = 12.292MHz (for SAI)
            divq: Some(PllDiv::DIV4),
            divr: Some(PllDiv::DIV32),
        });

        config.rcc.sys = Sysclk::PLL1_P;            // 400MHz
        config.rcc.mux.sai1sel = hal::rcc::mux::Saisel::PLL3_P;  // 12.288MHz for SAI1
        config.rcc.mux.sai23sel = hal::rcc::mux::Saisel::PLL3_P; // 12.288MHz for SAI2/3
        config.rcc.ahb_pre = AHBPrescaler::DIV2;    // 200MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2;   // 100MHz
        config.rcc.apb2_pre = APBPrescaler::DIV2;   // 100MHz
        config.rcc.apb3_pre = APBPrescaler::DIV2;   // 100MHz
        config.rcc.apb4_pre = APBPrescaler::DIV2;   // 100MHz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }

    let peri = hal::init(config);

    let kernel_clock = hal::rcc::frequency::<hal::peripherals::SAI1>().0;
    let mclk_div= mclk_div_from_u8((kernel_clock / (SAMPLE_RATE * 256)) as u8);

    let mut sai_tx_config = sai::Config::default();
    sai_tx_config.mode = Mode::Master;
    sai_tx_config.tx_rx = TxRx::Transmitter;
    sai_tx_config.sync_output = true;
    sai_tx_config.clock_strobe = ClockStrobe::Falling;
    sai_tx_config.master_clock_divider = mclk_div;
    sai_tx_config.data_size = DataSize::Data24;
    sai_tx_config.bit_order = BitOrder::MsbFirst;
    sai_tx_config.frame_sync_polarity = FrameSyncPolarity::ActiveHigh;
    sai_tx_config.frame_sync_active_level_length = sai::word::U7(32);
    sai_tx_config.fifo_threshold = FifoThreshold::Half;

    let tx_buffer: &mut [u32] = unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(TX_BUFFER);
        buf.initialize_all_copied(0);
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let (sai1_sub_block_tx, _sai1_sub_block_rx) 
        = hal::sai::split_subblocks(peri.SAI1);
    let mut sai_tx = sai::Sai::new_asynchronous(
        sai1_sub_block_tx,  // SubBlock
        peri.PE5,           // SCK
        peri.PE6,           // SD
        peri.PE4,           // FS
        peri.DMA1_CH0,      // DMA
        tx_buffer,          // Buffer
        Irqs,               // IRQs
        sai_tx_config,      // Config
    );

    let mut sai_rx_config = sai_tx_config.clone();
    sai_rx_config.mode = Mode::Master;
    sai_rx_config.tx_rx = TxRx::Receiver;

    let rx_buffer: &mut [u32] = unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(RX_BUFFER);
        buf.initialize_all_copied(0);
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let (_sai2_sub_block_tx, sai2_sub_block_rx) 
        = hal::sai::split_subblocks(peri.SAI2);
    let mut sai_rx = sai::Sai::new_asynchronous(
        sai2_sub_block_rx,  // SubBlock
        peri.PE12,          // SCK
        peri.PE11,          // SD
        peri.PE13,          // FS
        peri.DMA1_CH1,      // DMA
        rx_buffer,          // Buffer
        Irqs,               // IRQs
        sai_rx_config,      // Config
    );

    sai_rx.start().unwrap();

    let mut buf = [0u32; HALF_DMA_BUFFER_SIZE];

    loop {
        // write() must be called before read() to start the master(tx)
        sai_tx.write(&buf).await.unwrap();
        sai_rx.read(&mut buf).await.unwrap();
    }

    //fft_analyzer::exit()
}
