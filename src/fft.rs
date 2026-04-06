

use crate::complex::Complex;
use core::f32::consts::PI;

const R_INDEXES_SIZE: usize = 512 * 4;
const W_SIZE: usize = R_INDEXES_SIZE + 1;

pub struct Fft {
    n: usize,
    r_indexes: [usize; R_INDEXES_SIZE],
    w: [Complex; W_SIZE],
}

impl Fft {
    pub fn new() -> Self {
        Self {
            n: 0,
            r_indexes: [0; R_INDEXES_SIZE],
            w: [Complex::new(0.0, 0.0); W_SIZE],
        }
    }

    pub fn setup(&mut self, n: usize) {
        assert!(n <= R_INDEXES_SIZE, "n must be <= {}", R_INDEXES_SIZE);
        self.n = n;
        self.calc_bit_reversed_indexes(n);
        self.calc_w(n);
    }

    fn calc_bit_reversed_indexes(&mut self, n: usize) -> usize {
        let n_power2 = n.trailing_zeros();
        let mut r_bit = 1 << n_power2;
        let mut len = 1;
        self.r_indexes[0] = 0;

        while r_bit > 2 {
            r_bit >>= 2;
            let current_len = len;
            for j in 0..current_len {
                self.r_indexes[len + j] = self.r_indexes[j] | r_bit;
            }
            len += current_len;
            for j in 0..current_len {
                self.r_indexes[len + j] = self.r_indexes[j] | (r_bit << 1);
            }
            len += current_len;
            for j in 0..current_len {
                self.r_indexes[len + j] = self.r_indexes[j] | r_bit | (r_bit << 1);
            }
            len += current_len;
        }
        if r_bit == 2 {
            let current_len = len;
            for j in 0..current_len {
                self.r_indexes[len + j] = self.r_indexes[j] | 1;
            }
            len += current_len;
        }

        assert_eq!(len, n, "bit-reversed indexes should fill n elements");

        self.convert_indexes_as_inplace(n);
        len
    }

    fn convert_indexes_as_inplace(&mut self, len: usize) {
        let mut nums = [0usize; R_INDEXES_SIZE];
        for i in 0..len {
            nums[i] = i;
        }

        for i in 0..len {
            let r_i = self.r_indexes[i];
            let mut swapped_r_i = None;
            for j in 0..len {
                if nums[j] == r_i {
                    swapped_r_i = Some(j);
                    break;
                }
            }

            let swap_index = swapped_r_i.expect("r_i should exist in nums");
            nums.swap(i, swap_index);
            self.r_indexes[i] = swap_index;
        }
    }

    fn calc_w(&mut self, n: usize) -> usize {
        assert!(n + 1 <= W_SIZE, "w capacity exceeded");

        let mut len = 0;
        self.w[len] = Complex { re: 1.0, im: 0.0 };
        len += 1;

        // case N mod 4 == 0
        let q = n >> 2;
        let h = n >> 1;
        for i in 1..q {
            self.w[len] = self.calc_part_w(n, i);
            len += 1;
        }
        // W_N^N/4 = -i
        self.w[len] = Complex { re: 0.0, im: -1.0 };
        len += 1;
        // N/4 to N/2
        for i in q + 1..h {
            let tmp = self.w[i - q];
            self.w[len] = Complex { re: tmp.im, im: -tmp.re };
            len += 1;
        }
        // W_N^N/2 = -1
        self.w[len] = Complex { re: -1.0, im: 0.0 };
        len += 1;
        // N/2 to N
        for i in h + 1..n {
            let tmp = self.w[i - h];
            self.w[len] = Complex { re: -tmp.re, im: -tmp.im };
            len += 1;
        }
        self.w[len] = Complex { re: 1.0, im: 0.0 };
        len += 1;

        len
    }

    fn calc_part_w(&self, n: usize, seq: usize) -> Complex {
        // Separate e^(-2πi) to n
        Complex::new(
            0.0,
            -2.0 * PI / (n as f32) * (seq as f32)
        ).exp()
    }

    pub fn process(&mut self, frames: &mut [Complex]) {
        let len = frames.len();
        let len_power2 = len.trailing_zeros();

        assert_eq!(len, 1 << len_power2, "len of frames should be 2^x");

        if len <= 1 {
            return;
        }

        if len != self.n {
            self.setup(len);
        }

        self.inner_process(frames);
    }

    fn inner_process(&mut self, x: &mut [Complex]) {
        let n = x.len();

        let mut calc_n = n;
        let mut calc_w_bit = 0;

        while calc_n > 4 {
            let before_n = calc_n;
            calc_n >>= 2;
            for i in 0..calc_n {
                let w_i = i << calc_w_bit;
                let (w1, w2, w3) = (self.w[w_i], self.w[w_i << 1], self.w[w_i * 3]);
                for begin_i in (0..n).step_by(before_n) {
                    let i0 = begin_i + i;
                    let i1 = i0 + calc_n;
                    let i2 = i1 + calc_n;
                    let i3 = i2 + calc_n;

                    let xi0_plus_xi2 = x[i0] + x[i2];
                    let xi0_minus_xi2 = x[i0] - x[i2];
                    let xi1_plus_xi3 = x[i1] + x[i3];
                    let xi1_minus_xi3 = x[i1] - x[i3];
                    let xi1_minus_xi3_i = Complex::new(-xi1_minus_xi3.im, xi1_minus_xi3.re);

                    x[i0] =  xi0_plus_xi2  + xi1_plus_xi3;
                    x[i1] = (xi0_minus_xi2 - xi1_minus_xi3_i) * w1;
                    x[i2] = (xi0_plus_xi2  - xi1_plus_xi3)    * w2;
                    x[i3] = (xi0_minus_xi2 + xi1_minus_xi3_i) * w3;
                }
            }
            calc_w_bit += 2;
        }

        if calc_n == 4 {
            for i0 in (0..n).step_by(calc_n) {
                let i1 = i0 + 1;
                let i2 = i1 + 1;
                let i3 = i2 + 1;

                let xi0_plus_xi2 = x[i0] + x[i2];
                let xi0_minus_xi2 = x[i0] - x[i2];
                let xi1_plus_xi3 = x[i1] + x[i3];
                let xi1_minus_xi3 = x[i1] - x[i3];
                let xi1_minus_xi3_i = Complex::new(-xi1_minus_xi3.im, xi1_minus_xi3.re);

                x[i0] = xi0_plus_xi2  + xi1_plus_xi3;
                x[i1] = xi0_minus_xi2 - xi1_minus_xi3_i;
                x[i2] = xi0_plus_xi2  - xi1_plus_xi3;
                x[i3] = xi0_minus_xi2 + xi1_minus_xi3_i;
            }
        }
        else {
            // calc_n = 2
            for i in (0..n).step_by(calc_n) {
                let even_i = i;
                let odd_i = i + 1;
                let x_even = x[even_i] + x[odd_i];
                let x_odd  = x[even_i] - x[odd_i];
                x[even_i] = x_even;
                x[odd_i]  = x_odd;
            }

            for (i, &r) in self.r_indexes[..self.n].iter().enumerate() {
                x.swap(i, r);
            }
        }
    }
}

//------------------------------
// Unit test
//------------------------------
#[cfg(test)]
mod tests {
    use defmt::println;
    use micromath::F32Ext;
    use super::*;

    #[test]
    fn single_sin_wave_test() {
        let frame_size: usize = 2048;
        let sampling_rate = 48000;
        let hz = 2000;

        let mut sin_curve = (0..frame_size)
            .map(|x| (x as f32) * 2.0 * PI * (hz as f32) / (sampling_rate as f32))
            .map(|x| Complex::new(x.sin(), 0.0))
            .collect::<Vec<Complex>>();

        let size = frame_size.next_power_of_two();
        let mut fft = Fft::new();

        fft.setup(size);
        fft.process(&mut sin_curve);

        let result = sin_curve;
        let spectrum = &result[..(size/2)];
        let (max_index, max) = spectrum.iter().enumerate()
            .fold((0, f32::MIN), |(i_a, a), (i_b, &b)| {
                if b.norm() > a {
                    (i_b, b.norm())
                }
                else {
                    (i_a, a)
                }
            });
        let max_hz = (max_index as f32) * (sampling_rate as f32) / (size as f32);

        println!("max_index: {:?}, max: {:?}, max_hz: {:?}", max_index, max, max_hz);
        assert_eq!(max_index, 85);
        assert_eq!(max_hz, 1992.1875);
    }

}
