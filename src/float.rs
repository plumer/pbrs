pub struct Interval {
    pub min: f32,
    pub max: f32,
}

pub const ONE_PLUS_EPSILON: f32 = 1.0 + f32::EPSILON;
pub const ONE_MINUS_EPSILON: f32 = 1.0 - f32::EPSILON;

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
