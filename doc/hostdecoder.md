# Host Decoder (Rust) 仕様・実装例

## 概要

このドキュメントは `fft-analyzer` デバイスが送信する HID Input Report（256 bytes）を Rust で受信し、`seq + offset + bins[252]` から 1024 ビンのスペクトラムフレームを再構成する実装例です。

- VID: `0x1209`
- PID: `0x0001`
- Report Size: `256`
- Header: `seq(u16 LE) + offset(u16 LE)`
- Payload: `bins[252]`
- 1フレーム: 1024 bins（`BLOCK_SIZE/2`）

---

## `Cargo.toml`

```/dev/null/Cargo.toml#L1-6
[dependencies]
hidapi = "2"
anyhow = "1"
use anyhow::{Context, Result};
use hidapi::HidApi;
use std::collections::HashMap;
use std::time::{Duration, Instant};

const VID: u16 = 0x1209;
const PID: u16 = 0x0001;

const REPORT_SIZE: usize = 256;
const HEADER_SIZE: usize = 4;
const BINS_PER_PACKET: usize = 252;
const FFT_BINS: usize = 1024;
const FRAME_TIMEOUT: Duration = Duration::from_millis(120);

#[derive(Debug)]
struct FrameAssembly {
    bins: [u8; FFT_BINS],
    received: [bool; FFT_BINS],
    received_count: usize,
    created_at: Instant,
}

impl FrameAssembly {
    fn new() -> Self {
        Self {
            bins: [0; FFT_BINS],
            received: [false; FFT_BINS],
            received_count: 0,
            created_at: Instant::now(),
        }
    }

    fn insert_packet(&mut self, offset: usize, payload: &[u8]) {
        for (i, &v) in payload.iter().enumerate() {
            let idx = offset + i;
            if idx >= FFT_BINS {
                break;
            }
            self.bins[idx] = v;
            if !self.received[idx] {
                self.received[idx] = true;
                self.received_count += 1;
            }
        }
    }

    fn is_complete(&self) -> bool {
        self.received_count == FFT_BINS
    }
}

fn main() -> Result<()> {
    let api = HidApi::new().context("failed to initialize hidapi")?;
    let device = api
        .open(VID, PID)
        .with_context(|| format!("failed to open device {:04x}:{:04x}", VID, PID))?;

    println!("opened HID device {:04x}:{:04x}", VID, PID);

    let mut frames: HashMap<u16, FrameAssembly> = HashMap::new();
    let mut report = [0u8; REPORT_SIZE];

    loop {
        let n = device.read_timeout(&mut report, 50).context("hid read failed")?;
        if n == 0 {
            gc_frames(&mut frames);
            continue;
        }
        if n != REPORT_SIZE {
            continue;
        }

        let seq = u16::from_le_bytes([report[0], report[1]]);
        let offset = u16::from_le_bytes([report[2], report[3]]) as usize;
        let bins = &report[HEADER_SIZE..(HEADER_SIZE + BINS_PER_PACKET)];

        if offset >= FFT_BINS {
            continue;
        }

        let frame = frames.entry(seq).or_insert_with(FrameAssembly::new);
        frame.insert_packet(offset, bins);

        if frame.is_complete() {
            on_frame_complete(seq, &frame.bins);
            frames.remove(&seq);
        }

        gc_frames(&mut frames);
    }
}

fn gc_frames(frames: &mut HashMap<u16, FrameAssembly>) {
    let now = Instant::now();
    frames.retain(|_, f| now.duration_since(f.created_at) <= FRAME_TIMEOUT);
}

fn on_frame_complete(seq: u16, bins: &[u8; FFT_BINS]) {
    let (max_idx, max_val) = bins
        .iter()
        .enumerate()
        .max_by_key(|(_, v)| **v)
        .map(|(i, v)| (i, *v))
        .unwrap_or((0, 0));

    // Fs=48kHz, N=2048 の換算
    let freq_hz = (max_idx as f32) * 48000.0 / 2048.0;
    println!(
        "seq={} peak_bin={} peak_val={} approx_freq={:.2}Hz",
        seq, max_idx, max_val, freq_hz
    );
}
