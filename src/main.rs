#![no_main]
#![no_std]

use fft_analyzer as _; // global logger + panicking-behavior + memory layout
use defmt::*;
use embassy_stm32 as hal;
use hal::{bind_interrupts, time::*, sai::{self, *}, dma, usb};
use embassy_executor::{self, Spawner};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Sender, Receiver, TrySendError},
};
use embassy_usb::{Builder, UsbDevice};
use embassy_usb::driver::{Endpoint as _, EndpointIn as _, EndpointError};
use static_cell::StaticCell;
use grounded::uninit::GroundedArrayCell;

mod fft;
mod complex;
use fft::Fft;
use complex::Complex;

// const SAMPLE_RATE: u32 = 48_000;
const BLOCK_SIZE: usize = 512 * 4;
const HALF_DMA_BUFFER_SIZE: usize = BLOCK_SIZE * 2; // 2ch
const DMA_BUFFER_SIZE: usize = HALF_DMA_BUFFER_SIZE * 2; // 2 half blocks
const FFT_AVE_BUFFER_SIZE: usize = BLOCK_SIZE / 2; // Only need half the bins (positive frequencies)

const PACKET_SIZE: usize = 1026;
const BULK_CHUNK: usize = 64;           // FS Bulk max packet size
const BINS_PER_PACKET: usize = 1024;    // 2 bytes for seq and rest for bins
const QUEUE_DEPTH: usize = 8;
const SEND_FREQUENCY: u16 = 15;

pub type UsbDriver = hal::usb::Driver<'static, hal::peripherals::USB_OTG_FS>;
type BulkIn = <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn;

#[derive(Clone, Copy)]
pub struct SpectrumPacket {
    pub seq: u16,
    pub bins: [u8; BINS_PER_PACKET],
}

type Frame = [u32; HALF_DMA_BUFFER_SIZE];

// DMA buffers for SAI RX/TX
//   These are placed in SRAM1 to ensure they are in a memory region accessible by the DMA controller
//   and not affected by cache issues.
//   The size is set to accommodate ping-pong buffering for continuous audio streaming.
#[unsafe(link_section = ".sram1")]
static TX_BUFFER: StaticCell<[u32; DMA_BUFFER_SIZE]> = StaticCell::new();
#[unsafe(link_section = ".sram1")]
static RX_BUFFER: StaticCell<[u32; DMA_BUFFER_SIZE]> = StaticCell::new();

static mut FFT_BUFFER: GroundedArrayCell<Complex, BLOCK_SIZE> = GroundedArrayCell::uninit();
static mut FFT_AVE_BUFFER: GroundedArrayCell<f32, FFT_AVE_BUFFER_SIZE> = GroundedArrayCell::uninit();

// USB Bulk buffers
//   These are placed in SRAM1 to ensure they are in a memory region accessible by the USB peripheral
//   and not affected by cache issues.
#[unsafe(link_section = ".sram1")]
static USB_EP_OUT_BUF: StaticCell<[u8; 512]> = StaticCell::new();
#[unsafe(link_section = ".sram1")]
static USB_DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
#[unsafe(link_section = ".sram1")]
static USB_CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
#[unsafe(link_section = ".sram1")]
static USB_BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
#[unsafe(link_section = ".sram1")]
static USB_CTRL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

// Channel
static START_FFT_ANALYSIS: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
static SEND_SPECTRUM: Channel<CriticalSectionRawMutex, SpectrumPacket, QUEUE_DEPTH> = Channel::new();

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 => dma::InterruptHandler<hal::peripherals::DMA1_CH0>;
    DMA1_STREAM1 => dma::InterruptHandler<hal::peripherals::DMA1_CH1>;
    OTG_FS => usb::InterruptHandler<hal::peripherals::USB_OTG_FS>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    println!("--- Initialize SRAM1 ---");
    let tx_buffer = TX_BUFFER.init([0; DMA_BUFFER_SIZE]);
    let rx_buffer = RX_BUFFER.init([0; DMA_BUFFER_SIZE]);
    let ep_out_buffer = USB_EP_OUT_BUF.init([0; 512]);
    let device_descriptor = USB_DEVICE_DESC.init([0; 256]);
    let config_descriptor = USB_CONFIG_DESC.init([0; 256]);
    let bos_descriptor = USB_BOS_DESC.init([0; 256]);
    let control_buf = USB_CTRL_BUF.init([0; 64]);

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
            mul: PllMul::MUL192,        // PLL mul: 4MHz * 192 = 768MHz
            divp: Some(PllDiv::DIV2),   // PLL div for P clock (SYSCLK): 768MHz / 2 = 384MHz
            divq: Some(PllDiv::DIV16),  // PLL div for Q clock (USB)   : 768MHz / 16 = 48MHz
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

        config.rcc.sys = Sysclk::PLL1_P;            // 384MHz
        config.rcc.mux.sai1sel = hal::rcc::mux::Saisel::PLL3_P;  // ~12.286MHz for SAI1
        config.rcc.mux.sai23sel = hal::rcc::mux::Saisel::PLL3_P; // ~12.286MHz for SAI2/3
        config.rcc.mux.usbsel = hal::rcc::mux::Usbsel::PLL1_Q;    // 48MHz for USB
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

    // Initialize FFT buffer to zero
    unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(FFT_BUFFER);
        buf.initialize_all_copied(Complex::new(0.0, 0.0));
        let ave_buf = &mut *core::ptr::addr_of_mut!(FFT_AVE_BUFFER);
        ave_buf.initialize_all_copied(0.0);
    };

    // USB Bulk initialization
    let mut usb_cfg = hal::usb::Config::default();
    usb_cfg.vbus_detection = false;

    let usb_driver = hal::usb::Driver::new_fs(
        peri.USB_OTG_FS,
        Irqs,
        peri.PA12,
        peri.PA11,
        ep_out_buffer,
        usb_cfg,
    );

    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("Panasonic");
    config.product = Some("FFT Analyzer");
    config.serial_number = Some("0001");
    config.max_power = 500; // mA
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        usb_driver,
        config,
        device_descriptor,
        config_descriptor,
        bos_descriptor,
        control_buf,
    );

    // Vendor Bulk IN endpoint (class=0xFF, subclass=0, protocol=0)
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0, 0, None);
    let bulk_ep_in = alt.endpoint_bulk_in(None, 64);
    drop(alt);
    drop(interface);
    drop(function);
    let usb = builder.build();

    // SAI start
    sai_rx.start().unwrap();

    // Channel
    let sender = START_FFT_ANALYSIS.sender();
    let receiver = START_FFT_ANALYSIS.receiver();

    spawner.spawn(unwrap!(usb_task(usb)));
    spawner.spawn(unwrap!(bulk_tx_task(bulk_ep_in)));
    spawner.spawn(unwrap!(pass_through_audio(sai_rx, sai_tx, sender)));
    spawner.spawn(unwrap!(analyze_fft(receiver)));
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
            let sample = (((buf[i * 2] as i32) << 9) >> 9) as f32 / (1 << 23) as f32;
            fft_buffer[i] = Complex::new(sample, 0.0);
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
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };
    let fft_ave_buffer: &mut [f32] = unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(FFT_AVE_BUFFER);
        let (ptr, len) = buf.get_ptr_len();
        core::slice::from_raw_parts_mut(ptr, len)
    };

    let mut fft = Fft::new();
    fft.setup(BLOCK_SIZE);

    println!("Starting FFT analysis loop");
    let mut seq: u16 = 0;
    loop {
        let _ = receiver.receive().await;

        fft.process(fft_buffer);

        let spectrum = &fft_buffer[..(BLOCK_SIZE / 2)];
        for i in 0..spectrum.len() {
            // Simple moving average with alpha = 0.5
            fft_ave_buffer[i] = 0.5 * fft_ave_buffer[i] + 0.5 * spectrum[i].norm();
        }
        if seq % SEND_FREQUENCY == 0 {
            let _ = enqueue_spectrum(seq / SEND_FREQUENCY, fft_ave_buffer);
        }
        seq = seq.wrapping_add(1);
    }
}

fn enqueue_spectrum(seq: u16, spectrum: &[f32]) -> Result<(), TrySendError<SpectrumPacket>> {
    let mut pkt = SpectrumPacket {
        seq,
        bins: [0; BINS_PER_PACKET],
    };

    for i in 0..BINS_PER_PACKET {
        pkt.bins[i] = quantize(spectrum[i]);
    }

    SEND_SPECTRUM.try_send(pkt)?;
    Ok(())
}

fn quantize(v: f32) -> u8 {
    let scaled = (v * 64.0).clamp(0.0, 255.0);
    scaled as u8
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, UsbDriver>) {
    usb.run().await;
}

#[embassy_executor::task]
async fn bulk_tx_task(mut ep_in: BulkIn) {
    let mut buf = [0u8; PACKET_SIZE];
    loop {
        // Wait until the host connects and enables the endpoint
        ep_in.wait_enabled().await;
        println!("Bulk IN endpoint enabled");

        'connected: loop {
            let pkt = SEND_SPECTRUM.receive().await;

            // Assemble the packet: [seq(2)] [bins(252)]
            buf[0..2].copy_from_slice(&pkt.seq.to_le_bytes());
            for i in 0..BINS_PER_PACKET {
                buf[2 + i] = pkt.bins[i];
            }

            // Send PACKET_SIZE bytes as (PACKET_SIZE / BULK_CHUNK) × 64-byte packets
            for chunk in buf.chunks(BULK_CHUNK) {
                match ep_in.write(chunk).await {
                    Ok(()) => {}
                    Err(EndpointError::Disabled) => {
                        println!("Bulk IN disabled, waiting for reconnect");
                        break 'connected;
                    }
                    Err(EndpointError::BufferOverflow) => {
                        println!("Bulk IN buffer overflow");
                        break 'connected;
                    }
                }
            }

            // Send a zero-length packet to signal end of transfer.
            // Required because PACKET_SIZE (256) is an exact multiple of BULK_CHUNK (64).
            match ep_in.write(&[]).await {
                Ok(()) => {}
                Err(EndpointError::Disabled) => {
                    println!("Bulk IN disabled on ZLP, waiting for reconnect");
                    break 'connected;
                }
                Err(EndpointError::BufferOverflow) => {
                    println!("Bulk IN ZLP buffer overflow");
                    break 'connected;
                }
            }
        }
    }
}
