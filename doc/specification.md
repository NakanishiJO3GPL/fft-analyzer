# FFT Analyzer プロジェクト仕様書

## 1. 概要

本プロジェクトは、STM32H753ZI 上で動作するリアルタイム FFT 解析ファームウェアです。  
SAI + DMA で取得したオーディオデータに対して FFT を実行し、スペクトラム情報を USB HID 経由でホストへ送信します。

対象モジュール:

- `src/main.rs`: システム初期化、データフロー制御、USB 送信
- `src/fft.rs`: FFT アルゴリズム実装
- `src/complex.rs`: 複素数演算実装

---

## 2. システム構成

### 2.1 実行環境

- `#![no_std]`
- `#![no_main]`
- 非同期ランタイム: Embassy
- MCU: STM32H753ZI
- 通信: USB FS (HID クラス)
- 音声 I/O: SAI + DMA (ping-pong バッファ)

### 2.2 データフロー

1. SAI RX が DMA で連続サンプリング
2. 受信ブロックを FFT 入力バッファへ整形
3. FFT 実行 (`Fft::process`)
4. 正の周波数側スペクトラムを集計・量子化
5. HID レポートに分割して USB 送信

---

## 3. `main.rs` 仕様

### 3.1 定数・バッファ

- `BLOCK_SIZE = 512 * 4 = 2048`  
  FFT サンプル数
- `HALF_DMA_BUFFER_SIZE = BLOCK_SIZE * 2`  
  ステレオ 2ch 分の半バッファサイズ
- `DMA_BUFFER_SIZE = HALF_DMA_BUFFER_SIZE * 2`  
  ping-pong 用フル DMA サイズ
- `FFT_AVE_BUFFER_SIZE = BLOCK_SIZE / 2`  
  正周波数側ビン数
- `REPORT_SIZE = 256`  
  HID レポートサイズ
- `BINS_PER_PACKET = 252`  
  1パケットに含めるスペクトラムデータ
- `QUEUE_DEPTH = 8`  
  送信キュー深さ
- `SEND_FREQUENCY = 30`  
  送信制御に使う周波数パラメータ

DMA/USB 関連バッファは `#[unsafe(link_section = ".sram1")]` により SRAM1 配置。  
意図は DMA/USB アクセス性とキャッシュ影響低減。

### 3.2 USB HID 仕様

- Usage Page: Vendor (`0xFF00`)
- Application Collection
- 8-bit report field
- Report Count: 256 bytes
- 入力レポートのみ (`Input (Data,Var,Abs)`)

#### パケット構造 (`SpectrumPacket`)

- `seq: u16`  
  送信シーケンス番号
- `offset: u16`  
  全ビン列内の先頭オフセット
- `bins: [u8; 252]`  
  スペクトラム値（量子化済み）

### 3.3 クロック設定方針

- HSE 8MHz を基準
- PLL1:
  - SYSCLK 用 (`384MHz`)
  - USB 用 48MHz (`PLL1_Q`)
- PLL3:
  - SAI 用クロック（約 12.286MHz）
  - 目標 12.288MHz に対し整数分周近似

### 3.4 SAI 設定

- 48kHz フレーム想定
- Stereo, 24-bit データ
- フレーム長 64bit（32bit/ch）
- TX: Master Transmitter
- RX: Master Receiver
- RX FIFO: Half threshold（DMA 転送安定化）

### 3.5 タスク間通信

`embassy_sync::channel::Channel` を使用:

- `START_FFT_ANALYSIS: Channel<bool, depth=2>`  
  FFT 解析トリガ
- `SEND_SPECTRUM: Channel<SpectrumPacket, depth=8>`  
  USB 送信用キュー

---

## 4. `fft.rs` 仕様

### 4.1 目的

複素数列に対する高速フーリエ変換（FFT）を実行し、周波数成分を算出する。

### 4.2 データ構造

```text
pub struct Fft {
    n: usize,                          // 現在セットアップ済みサイズ
    r_indexes: [usize; 2048],          // in-place 並べ替え用インデックス
    w: [Complex; 2049],                // 回転因子テーブル
}
```

- `R_INDEXES_SIZE = 2048`
- `W_SIZE = 2049`

### 4.3 公開 API

- `Fft::new() -> Self`  
  初期化
- `setup(&mut self, n: usize)`  
  サイズ `n` 用にインデックス・回転因子を準備
- `process(&mut self, frames: &mut [Complex])`  
  FFT 実行（必要に応じ `setup` 再実行）

### 4.4 前提条件

- `n` は 2 のべき乗
- `n <= 2048`
- `frames.len()` は 2 のべき乗
- `frames.len() > 1` で通常処理

違反時は `assert!` / `assert_eq!` により停止。

### 4.5 アルゴリズム概要

- ビット反転順に基づくインプレース処理用インデックスを計算
- 回転因子 `W_N^k = exp(-j 2πk/N)` を前計算
- 主演算は 4-point バタフライを主軸に段階処理
- 残段に応じて 4-point または 2-point を適用
- 必要時に並べ替え（`r_indexes`）を適用

### 4.6 テスト

`#[cfg(test)]` のユニットテスト `single_sin_wave_test` を実装:

- 48kHz サンプリング、2kHz 正弦波入力
- サイズ 2048 FFT
- 最大ピークインデックス確認 (`85`)
- 周波数確認 (`1992.1875Hz`)

---

## 5. `complex.rs` 仕様

### 5.1 目的

`no_std` 環境で利用可能な軽量複素数型を提供する。

### 5.2 型定義

```text
#[derive(PartialEq, Copy, Clone, Debug, Default)]
#[repr(C)]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}
```

### 5.3 演算子

- `Add` (`a + b`)
- `Sub` (`a - b`)
- `Mul` (`a * b`)  
  `re = ar*br - ai*bi`, `im = ar*bi + ai*br`

### 5.4 メソッド

- `const fn new(re, im) -> Self`
- `norm(self) -> f32`  
  ユークリッドノルム（`hypot`）
- `from_polar(r, theta) -> Self`  
  極形式から生成
- `exp(self) -> Self`  
  複素指数 `e^(re + j im)` を計算  
  特殊値（`inf`, `nan`）に対する分岐保護あり

---

## 6. 送信データ仕様（実装準拠）

- FFT 結果は主に正周波数側（`N/2`）を対象
- 送信時に `u8` へ量子化し、`252` ビン単位で分割
- 各パケットは `seq` と `offset` を持ち、再構成を可能にする

---

## 7. 非機能要件

### 7.1 リアルタイム性

- DMA ping-pong により連続取り込み
- FFT 処理と USB 送信をチャネルで分離しスループット確保

### 7.2 メモリ制約対応

- 固定長静的バッファ中心
- ヒープ非依存（`no_std` 運用前提）

### 7.3 保守性

- FFT/複素数を独立モジュール化
- 演算ロジックは `fft.rs` に集約

---

## 8. 制約・注意事項

- FFT 最大サイズは `2048` 固定（現在実装）
- USB HID は Vendor 定義レポートであり、ホスト側で独自デコードが必要
- SAI クロックは近似値運用（完全一致ではない）
- `unsafe` を含む静的バッファ操作は初期化順序・排他を厳守すること

---

## 9. 今後の拡張候補

- 窓関数（Hann/Hamming 等）の適用
- スペクトラム平滑化・ピーク追跡の高度化
- 可変 FFT サイズ対応
- USB プロトコルのバージョニング
- ホストツールとの仕様同期（packet schema 文書化）