#![no_main]
#![no_std]

use fft_analyzer as _; // global logger + panicking-behavior + memory layout
use defmt::*;
use embassy_stm32 as hal;
use hal::{*, time::*, sai::*};
use embassy_executor::{self, Spawner};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Sender, Receiver},
};
use embassy_time::Instant;
use grounded::uninit::GroundedArrayCell;

mod fft;
mod complex;
use fft::Fft;
use complex::Complex;

const SAMPLE_RATE: u32 = 48_000;
const BLOCK_SIZE: usize = 512 * 4;
const HALF_DMA_BUFFER_SIZE: usize = BLOCK_SIZE * 2; // 2ch
const DMA_BUFFER_SIZE: usize = HALF_DMA_BUFFER_SIZE * 2; // 2 half blocks
type Frame = [u32; HALF_DMA_BUFFER_SIZE];

#[unsafe(link_section = ".sram1")]
static mut TX_BUFFER: GroundedArrayCell<u32, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();
#[unsafe(link_section = ".sram1")]
static mut RX_BUFFER: GroundedArrayCell<u32, DMA_BUFFER_SIZE> = GroundedArrayCell::uninit();
#[unsafe(link_section = ".sram1")]
static mut FFT_BUFFER: GroundedArrayCell<Complex, BLOCK_SIZE> = GroundedArrayCell::uninit();

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => dma::InterruptHandler<hal::peripherals::DMA1_CH0>;
    DMA1_STREAM1 => dma::InterruptHandler<hal::peripherals::DMA1_CH1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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
        // Target: 12.288MHz (48kHz * 256). Exact integer ratio is not possible on H753 PLL3,
        // so pick a close integer combo: VCO = 344MHz (8MHz * 43), P div = 28 → 12.2857MHz (~0.02% error).
        config.rcc.pll3 = Some(Pll {
            source: PllSource::HSE,    // 8MHz
            prediv: PllPreDiv::DIV1,   // 8MHz
            mul: PllMul::MUL43,        // 8 * 43 = 344MHz (within 150–420MHz range)
            divp: Some(PllDiv::DIV28), // 344 / 28 ≈ 12.2857MHz ≈ 12.288MHz target
            divq: None,
            divr: None,
        });

        config.rcc.sys = Sysclk::PLL1_P;            // 400MHz
        config.rcc.mux.sai1sel = hal::rcc::mux::Saisel::PLL3_P;  // ~12.286MHz for SAI1
        config.rcc.mux.sai23sel = hal::rcc::mux::Saisel::PLL3_P; // ~12.286MHz for SAI2/3
        config.rcc.ahb_pre = AHBPrescaler::DIV2;    // 200MHz
        config.rcc.apb1_pre = APBPrescaler::DIV2;   // 100MHz
        config.rcc.apb2_pre = APBPrescaler::DIV2;   // 100MHz
        config.rcc.apb3_pre = APBPrescaler::DIV2;   // 100MHz
        config.rcc.apb4_pre = APBPrescaler::DIV2;   // 100MHz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }

    let peri = hal::init(config);

    // SAI configuration:
    //  Target 48 kHz frame, stereo, 24-bit: use 64 bit clocks per frame (32 bits per channel).
    //  Kernel clock ≈12.286 MHz (PLL3_P). To get SCK ≈ 3.072 MHz (48 kHz * 64),
    //  use the smallest divider: DIV1 (raw MCKDIV=0 → divide by 2*(0+1)=2).
    //  This yields ~6.14 MHz on SCK pin; SAI internally halves for LRCLK, giving 48 kHz frame.

    let mut sai_tx_config = sai::Config::default();
    sai_tx_config.mode = Mode::Master;
    sai_tx_config.tx_rx = TxRx::Transmitter;
    sai_tx_config.sync_output = true;
    sai_tx_config.clock_strobe = ClockStrobe::Falling;
    sai_tx_config.data_size = DataSize::Data24;
    sai_tx_config.bit_order = BitOrder::MsbFirst;
    sai_tx_config.slot_size = SlotSize::Channel32;
    sai_tx_config.frame_length = 64;
    sai_tx_config.frame_sync_active_level_length = sai::word::U7(32);
    sai_tx_config.frame_sync_polarity = FrameSyncPolarity::ActiveHigh;
    sai_tx_config.fifo_threshold = FifoThreshold::Empty;

    let tx_buffer: &mut [u32] = unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(TX_BUFFER);
        buf.initialize_all_copied(0);
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let (sai1_sub_block_tx, _sai1_sub_block_rx) = hal::sai::split_subblocks(peri.SAI1);
    let sai_tx = sai::Sai::new_asynchronous(
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
    sai_rx_config.fifo_threshold = FifoThreshold::Half; // Start DMA transfer when FIFO is half full to allow ping-pong buffering

    let rx_buffer: &mut [u32] = unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(RX_BUFFER);
        buf.initialize_all_copied(0);
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let (_sai2_sub_block_tx, sai2_sub_block_rx) = hal::sai::split_subblocks(peri.SAI2);
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

    // Channel
    static START_FFT_ANALYSIS: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
    let sender = START_FFT_ANALYSIS.sender();
    let receiver = START_FFT_ANALYSIS.receiver();

    spawner
        .spawn(unwrap!(pass_through_audio(sai_rx, sai_tx, sender)));
    
    spawner
        .spawn(unwrap!(analyze_fft(receiver)));

}

#[embassy_executor::task]
async fn pass_through_audio(
    mut sai_rx: sai::Sai<'static, hal::peripherals::SAI2, u32>, 
    mut sai_tx: sai::Sai<'static, hal::peripherals::SAI1, u32>,
    sender: Sender<'static, CriticalSectionRawMutex, bool, 2>,
) {
    let mut buf: Frame = [0; HALF_DMA_BUFFER_SIZE];
    let fft_buffer: &mut [Complex] = unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(FFT_BUFFER);
        buf.initialize_all_copied(Complex::new(0.0, 0.0));
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    loop {
        match sai_rx.read(&mut buf).await {
            Ok(()) => {}
            Err(sai::Error::Overrun) => {
                warn!("SAI RX overrun, dropping frame");
                continue;
            }
            Err(e) => {
                warn!("SAI RX error: {:?}", e);
                continue;
            }
        }
        
        if let Err(e) = sai_tx.write(&buf).await {
            warn!("SAI TX error: {:?}", e);
            continue;
        }
        
        for i in 0..BLOCK_SIZE {
            let left_sample = (((buf[i * 2] as i32) << 9) >> 9) as f32 / (1 << 23) as f32;
            fft_buffer[i] = Complex::new(left_sample, 0.0);
        }

        sender.clear();
        sender.send(true).await;
    }
}

#[embassy_executor::task]
async fn analyze_fft(
    receiver: Receiver<'static, CriticalSectionRawMutex, bool, 2>,
) {
    let fft_buffer: &mut [Complex] = unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(FFT_BUFFER);
        buf.initialize_all_copied(Complex::new(0.0, 0.0));
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let mut fft = Fft::new();
    fft.setup(BLOCK_SIZE);

    println!("Starting FFT analysis loop");
    loop {
        let _ = receiver.receive().await;

        fft.process(fft_buffer);

        let spectrum = &fft_buffer[..(BLOCK_SIZE / 2)];
        let (max_idx, max_lvl) = spectrum.iter().enumerate().fold(
            (0usize, f32::MIN),
            |(i_a, a), (i_b, &b)| {
                if b.norm() > a {
                    (i_b, b.norm())
                } else {
                    (i_a, a)
                }
            },
        );

        let max_hz = (max_idx as f32) * (SAMPLE_RATE as f32) / (BLOCK_SIZE as f32);
        let now = Instant::now().as_millis() as u64;
        println!("{} : fft peak: {} Hz (norm {})", now, max_hz, max_lvl);
    }
}