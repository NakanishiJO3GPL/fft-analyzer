use micromath::F32Ext;
use core::ops::{Add, Sub, Mul};

#[derive(PartialEq, Copy, Clone, Debug, Default)]
#[repr(C)]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}

impl Add for Complex {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            re: self.re + other.re,
            im: self.im + other.im,
        }
    }
}

impl Sub for Complex {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            re: self.re - other.re,
            im: self.im - other.im,
        }
    }
}

impl Mul for Complex {
    type Output = Self;
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }
}

impl Complex {
    #[inline]
    pub const fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    #[inline]
    pub fn norm(self) -> f32 {
        self.re.hypot(self.im)
    }

    #[inline]
    pub fn from_polar(r: f32, theta: f32) -> Self {
        Self::new(r * theta.cos(), r * theta.sin())
    }

    #[inline]
    pub fn exp(self) -> Self {
        let Complex { re, mut im } = self;

        if re.is_infinite() {
            if re < 0.0 {
                if !im.is_finite() {
                    return Self::new(0.0, 0.0);
                }
            } else if im == 0.0 || !im.is_finite() {
                if im.is_infinite() {
                    im = f32::NAN;
                }
                return Self::new(re, im);
            }
        }
        else if re.is_nan() && im == 0.0 {
            return self;
        }

        Self::from_polar(re.exp(), im)
    }
}
