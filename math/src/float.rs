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

#[allow(dead_code)]
pub trait Float : Sized {
    /// Returns the length of other leg of the triangle given the hypotenuse and a known one.
    fn cathetus(self, other: Self) -> Self;
    /// Computes `x / y` if y is nonzero; returns `None` if y is zero.
    fn try_divide(self, divisor: Self) -> Option<Self>;
    /// Evaluates the polynomial c0 + c1 * x + c2 * x^2 + ... + cn * x^n. The coefficients should be
    /// given in increasing order of powers.
    fn polynomial<const N: usize>(self, coeffs: [Self; N]) -> Self;
}

impl Float for f32 {
    /// Computes the other side of the right-angle side given the hypotenuse.
    /// Returns 0.0 if the hypotenuse (self) is shorter than the right-angle side.
    /// ```
    /// use math::float::Float;
    /// assert_eq!(1.0f32.cathetus(0.6), 0.8);
    /// assert_eq!(1.0f32.cathetus(-0.6), 0.8);
    /// ```
    fn cathetus(self, other: f32) -> f32 {
        (self.powi(2) - other.powi(2)).max(0.0).sqrt()
    }
    
    /// Computes `x / y` if y is nonzero; returns `None` if y is zero.
    /// ```
    /// use math::float::Float;
    /// assert_eq!(1.0f32.try_divide(0.0), None);
    /// assert_eq!(1.0f32.try_divide(2.5), Some(0.4));
    /// assert_eq!(0.0f32.try_divide(0.0), None);
    /// assert_eq!(0.0f32.try_divide(2.5), Some(0.0));
    /// ```
    fn try_divide(self, divisor: Self) -> Option<Self> {
        if divisor == 0.0 {
            None
        } else {
            Some(self / divisor)
        }
    }

    /// Evaluates the polynomial c0 + c1 * x + c2 * x^2 + ... + cn * x^n. The coefficients should be
    /// given in increasing order of powers.
    /// ```
    /// let x = 0.6;
    /// let fx = x.polynomial([4, 2, 1]);
    /// assert!((fx - (4.0 + 2 * 0.6 + 0.6*0.6)).abs() < 1e-6);
    /// ```
    fn polynomial<const N: usize>(self, coeffs: [Self; N]) -> Self {
        // a + b * x + c * x^2 + d * x^3
        // = a + x * (b + x * (c + d * x))
        coeffs.iter().rev().fold(0.0, |d, c| d * self + c)
    }
}

/// Divides the given `interval` evenly into `count` pieces and returns the midpoint of each piece
/// together with the spacing between adjacent midpoints.
pub fn linspace(interval: (f32, f32), count: i32) -> (Vec<f32>, f32) {
    let (a, b) = interval;
    (
        (0..count)
            .into_iter()
            .map(|i| (i as f32 + 0.5) / count as f32 * (b - a) + a)
            .collect::<Vec<_>>(),
        (b - a) / count as f32,
    )
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
                stringify!($right),
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
                stringify!($right),
                $left,
                $right
            )
        }
    };
}
