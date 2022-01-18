use std::{
    iter::Sum,
    ops::{Add, Div, Mul, Sub},
};

#[derive(Debug, Clone, Copy)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

/// Clamps an f32 value to [0, 1], mutiplies it by 255 and casts it to u8.
/// Returns 0 if `f` is NaN.
fn saturate_cast_u8(f: f32) -> u8 {
    if f > 1.0 {
        255
    } else if f >= 0.0 {
        (f * 255.0) as u8
    } else {
        0
    }
}

impl Color {
    pub fn new(r: f32, g: f32, b: f32) -> Color {
        Color { r, g, b }
    }
    #[rustfmt::skip]
    pub fn from_xyz(x: f32, y: f32, z: f32) -> Color {
        Color {
            r:  3.240479 * x - 1.537150 * y - 0.498535 * z,
            g: -0.969256 * x + 1.875991 * y + 0.041556 * z,
            b:  0.055648 * x - 0.204043 * y + 1.057311 * z,
        }
    }
    pub fn black() -> Color {
        Color::new(0.0, 0.0, 0.0)
    }
    pub fn white() -> Color {
        Color::new(1.0, 1.0, 1.0)
    }
    pub const ONE: Color = Self {
        r: 1.0,
        g: 1.0,
        b: 1.0,
    };
    pub fn gray(level: f32) -> Color {
        Color::new(level, level, level)
    }
    pub fn rgb(r: u8, g: u8, b: u8) -> Color {
        Color::new(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0)
    }
    pub fn gamma_encode(&self) -> Self {
        Color::new(self.r.sqrt(), self.g.sqrt(), self.b.sqrt())
    }
    pub fn is_black(&self) -> bool {
        self.r <= 0.0 && self.g <= 0.0 && self.b <= 0.0
    }
    pub fn to_u8(&self) -> [u8; 3] {
        [
            saturate_cast_u8(self.r),
            saturate_cast_u8(self.g),
            saturate_cast_u8(self.b),
        ]
    }
    pub fn has_nan(&self) -> bool {
        self.r.is_nan() || self.g.is_nan() || self.b.is_nan()
    }
    /// Returns true iff all RGB components are finite and free of NaNs.
    pub fn is_finite(&self) -> bool {
        self.r.is_finite() && self.g.is_finite() && self.b.is_finite()
    }

    pub fn average(color_vec: &Vec<Self>) -> Self {
        match color_vec.len() {
            0 => Self::black(),
            num => color_vec.iter().copied().sum::<Self>() * (1.0 / num as f32),
        }
    }

    pub fn scale_down_by(self, n: u32) -> Self {
        match n {
            0 => panic!(),
            _ => self * (1.0 / n as f32),
        }
    }
    /// Component-wise (per RGB channel) division.
    pub fn cw_div(&self, other: Self) -> Self {
        Color::new(self.r / other.r, self.g / other.g, self.b / other.b)
    }
    /// Component-wise sqrt (per RGB channel).
    pub fn cw_sqrt(&self) -> Self {
        Color::new(self.r.sqrt(), self.g.sqrt(), self.b.sqrt())
    }
    /// Component-wise max.
    pub fn cw_max(&self, x: f32) -> Self {
        Color::new(self.r.max(x), self.g.max(x), self.b.max(x))
    }
    /// Component-wise (per RGB channel) subtraction, with negative result clamped to 0.
    pub fn capped_subtract(&self, other: &Self) -> Self {
        Color::new(
            (self.r - other.r).max(0.0),
            (self.g - other.g).max(0.0),
            (self.b - other.b).max(0.0),
        )
    }
}

impl std::ops::Add for Color {
    type Output = Color;
    fn add(self, rhs: Self) -> Self {
        Color::new(self.r + rhs.r, self.g + rhs.g, self.b + rhs.b)
    }
}

impl std::ops::AddAssign for Color {
    fn add_assign(&mut self, rhs: Self) {
        self.r += rhs.r;
        self.g += rhs.g;
        self.b += rhs.b;
    }
}

impl std::ops::Sub for Color {
    type Output = Color;
    fn sub(self, rhs: Self) -> Self::Output {
        Color::new(self.r - rhs.r, self.g - rhs.g, self.b - rhs.b)
    }
}

impl std::ops::Mul<f32> for Color {
    type Output = Color;
    fn mul(self, s: f32) -> Self {
        Color::new(self.r * s, self.g * s, self.b * s)
    }
}

impl std::ops::Mul<Color> for f32 {
    type Output = Color;
    fn mul(self, c: Color) -> Color {
        c * self
    }
}

impl std::ops::Mul for Color {
    type Output = Color;
    fn mul(self, rhs: Color) -> Self::Output {
        Color::new(self.r * rhs.r, self.g * rhs.g, self.b * rhs.b)
    }
}

impl std::fmt::Display for Color {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let precision = f.precision().unwrap_or(2);
        write!(
            f,
            "rgb({:.precision$}, {:.precision$}, {:.precision$})",
            self.r,
            self.g,
            self.b,
            precision = precision
        )
    }
}

impl std::fmt::LowerHex for Color {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "#{:02x}{:02x}{:02x}",
            (self.r * 255.0) as u8,
            (self.g * 255.0) as u8,
            (self.b * 255.0) as u8
        )
    }
}

impl Sum for Color {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Color::black(), |c0, c1| c0 + c1)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct XYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl XYZ {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    pub fn all(x: f32) -> Self {
        Self::new(x, x, x)
    }
    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn from_color(c: Color) -> Self {
        Self::from_rgb(c.r, c.g, c.b)
    }

    pub fn from_rgb(r: f32, g: f32, b: f32) -> Self {
        Self {
            x: 0.41245330 * r + 0.35757984 * g + 0.18042262 * b,
            y: 0.21267127 * r + 0.71515972 * g + 0.07216883 * b,
            z: 0.01933384 * r + 0.11919363 * g + 0.95022693 * b,
        }
    }
    pub fn is_black(&self) -> bool {
        self.x <= 0.0 && self.y <= 0.0 && self.z <= 0.0
    }
    pub fn to_color(&self) -> Color {
        Color::from_xyz(self.x, self.y, self.z)
    }
    pub fn sqrt(self) -> Self {
        Self::new(self.x.sqrt(), self.y.sqrt(), self.z.sqrt())
    }
}

impl Add for XYZ {
    type Output = XYZ;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for XYZ {
    type Output = XYZ;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul for XYZ {
    type Output = XYZ;
    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }
}

impl Div for XYZ {
    type Output = XYZ;
    fn div(self, rhs: Self) -> Self::Output {
        Self::new(self.x / rhs.x, self.y / rhs.y, self.z / rhs.z)
    }
}

impl Mul<f32> for XYZ {
    type Output = XYZ;
    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<XYZ> for f32 {
    type Output = XYZ;
    fn mul(self, rhs: XYZ) -> Self::Output {
        rhs * self
    }
}
