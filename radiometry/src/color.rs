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
}

impl std::ops::Add for Color {
    type Output = Color;
    fn add(self, rhs: Self) -> Self {
        Color::new(self.r + rhs.r, self.g + rhs.g, self.b + rhs.b)
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

impl std::ops::Div<f32> for Color {
    type Output = Color;
    fn div(self, rhs: f32) -> Self::Output {
        Color::new(self.r / rhs, self.g / rhs, self.b / rhs)
    }
}

impl std::fmt::Display for Color {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "rgb({:.2}, {:.2}, {:.2})", self.r, self.g, self.b)
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