pub struct Interval {
    pub min: f32,
    pub max: f32,
}

pub const ONE_PLUS_EPSILON: f32 = 1.0 + f32::EPSILON;
pub const ONE_MINUS_EPSILON: f32 = 1.0 - f32::EPSILON;

/// Computes the linear interpolation between `a` and `b`: (0, 1) -> (a, b).
pub fn lerp<T, U>(a: T, b: T, t: f32) -> T
where
    T: Copy + std::ops::Sub<T, Output = U>,
    U: Copy + std::ops::Mul<f32, Output = U> + std::ops::Add<T, Output = T>,
{
    (b - a) * t + a
}

pub fn barycentric_lerp<T, U>(values: (T, T, T), bc_coeffs: (f32, f32, f32)) -> T
where
    T: Copy + std::ops::Sub<T, Output = U>,
    U: Copy + std::ops::Mul<f32, Output = U> + std::ops::Add<T, Output = T> + std::ops::Add<U, Output = U>
{
    let (a, b, c) = values;
    let (bc0, bc1, _) = bc_coeffs;
    //   bc0 * a + bc1 * b + (1 - bc0 - bc1) * c
    // = bc0 * (a-c) + bc1 * (b-c) + c
    (a - c) * bc0 + (b - c) * bc1 + c
}

/// Represents a non-empty interval on the real-number axis.
/// Any `Interval`s covers at least 1, and doesn't differentiate between open/closed intervals.
impl Interval {
    pub fn new(a: f32, b: f32) -> Self {
        assert!(!a.is_nan());
        assert!(!b.is_nan());
        let (a, b) = min_max(a, b);
        Interval { min: a, max: b }
    }

    pub fn length(&self) -> f32 {
        assert!(self.max >= self.min);
        self.max - self.min
    }

    pub fn contains(&self, x: f32) -> bool {
        x >= self.min && x <= self.max
    }

    pub fn as_pair(&self) -> (f32, f32) {
        (self.min, self.max)
    }
}

pub fn min_max(a: f32, b: f32) -> (f32, f32) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

pub trait Inside
where
    Self: std::cmp::PartialOrd + Sized + Copy,
{
    fn inside(self, interval: (Self, Self)) -> bool {
        let (left, right) = interval;
        left <= self && self <= right
    }
}

impl Inside for f32 {}

#[macro_export]
macro_rules! assert_le {
    ($left:expr, $right:expr) => {
        if $left > $right {
            panic!(
                "less-than assertion failed: {} < {} (values: {} vs. {})",
                stringify!($left),
                stringify! {$right},
                $left,
                $right
            )
        }
    };
}

#[macro_export]
macro_rules! assert_lt {
    ($left:expr, $right:expr) => {
        if $left >= $right {
            panic!(
                "less-than assertion failed: {} < {} (values: {} vs. {})",
                stringify!($left),
                stringify! {$right},
                $left,
                $right
            )
        }
    };
}
