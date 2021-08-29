/// Represents intervals on the real-number axis. Any `Interval`s covers at least 1 point.
/// There is no difference between open/closed intervals.
pub struct Interval {
    pub min: f32,
    pub max: f32,
}

/// 1 + \eps for f32. Multiply it to a `f32` number to get a larger one than is small as possible.
pub const ONE_PLUS_EPSILON: f32 = 1.0 + f32::EPSILON;
pub const ONE_MINUS_EPSILON: f32 = 1.0 - f32::EPSILON;

/// Computes the linear interpolation between `a` and `b`: (0, 1) -> (a, b).
/// 
/// This function also works if `a` and `b` are not "Scalable" by themselves - as long as `a-b` can
/// be scaled by a `f32`, and the difference can be added to either `a` or `b` to get back `T` then
/// `lerp` can be used.
/// - Although `Point3` can't be scaled, but the difference type `Vec3` can, and point + vector is
///   a point, so `lerp` can be used on 2 points.
/// - `lerp` can be used on `Vec3`s as well - easier to understand.
/// - If `std::time::Duration` could be scaled by `f32`, then `std::time::Instant` can be `lerp`ed.
#[allow(dead_code)]
pub fn lerp<T, U>(a: T, b: T, t: f32) -> T
where
    T: Copy + std::ops::Sub<T, Output = U>,
    U: Copy + std::ops::Mul<f32, Output = U> + std::ops::Add<T, Output = T>,
{
    (b - a) * t + a
}

/// Computes the barycentric interpolation given 3 attribute values and 3 barycentric coordinates.
/// The attribute can be of various types. If types can be `lerp`ed, then there's a great chance 
/// that they can be `barycentric_lerp`ed.
///
/// One more constraint on difference type (type of `T - T`): `U + U -> U`. If only this constraint
/// is unsatisfied, then there's a design problem with types `T` and `U`.
pub fn barycentric_lerp<T, U>(values: (T, T, T), bc_coeffs: (f32, f32, f32)) -> T
where
    T: Copy + std::ops::Sub<T, Output = U>,
    U: Copy
        + std::ops::Mul<f32, Output = U>
        + std::ops::Add<T, Output = T>
        + std::ops::Add<U, Output = U>,
{
    let (a, b, c) = values;
    let (bc0, bc1, _) = bc_coeffs;
    //   bc0 * a + bc1 * b + (1 - bc0 - bc1) * c
    // = bc0 * (a-c) + bc1 * (b-c) + c
    (a - c) * bc0 + (b - c) * bc1 + c
}

/// Returns the length of other leg of the triangle given the hypotenuse and a known one.
#[allow(dead_code)]
pub fn cathetus(hypot: f32, other: f32) -> f32 {
    (hypot.powi(2) - other.powi(2)).max(0.0).sqrt()
}

/// Represents intervals on the real-number axis. Any `Interval`s covers at least 1 point.
/// There is no difference between open/closed intervals.
impl Interval {
    /// Constructs an `Interval` with `a` and `b` being the endpoint.
    /// A comparison is made to determine which one is lesser / greater.
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

    /// Returns the left and right ends as a pair.
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
                "Assertion failed: {} <= {} (values: {} vs. {})",
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
                "Assertion failed: {} < {} (values: {} vs. {})",
                stringify!($left),
                stringify! {$right},
                $left,
                $right
            )
        }
    };
}

#[macro_export]
macro_rules! assert_gt {
    ($left:expr, $right:expr) => {
        if $left <= $right {
            panic!(
                "Assertion failed: {} > {} (values: {} vs. {})",
                stringify!($left),
                stringify! {$right},
                $left,
                $right
            )
        }
    };
}

#[macro_export]
macro_rules! assert_ge {
    ($left:expr, $right:expr) => {
        if $left < $right {
            panic!(
                "Assertion failed: {} >= {} (values: {} vs. {})",
                stringify!($left),
                stringify! {$right},
                $left,
                $right
            )
        }
    };
}