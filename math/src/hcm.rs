use core::convert::TryFrom;
use std::{
    fmt,
    ops::{Add, AddAssign, Div, Index, IndexMut, Mul, Neg, Sub},
};

pub fn vec3(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(x, y, z)
}

pub fn point3(x: f32, y: f32, z: f32) -> Point3 {
    Point3::new(x, y, z)
}

pub use glam::Vec4;

/// Represents a 3D vector. Each component is a `f32` number.
/// Components can be accessed using `v.x` `v.y` `v.z`,
/// or indices `v[i]` where i is 0, 1, or 2.
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Copy, Clone)]
pub struct Point3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[deprecated]
#[derive(Debug, Clone, Copy)]
pub struct Radian(pub f32);

#[deprecated]
#[derive(Debug, Clone, Copy)]
pub struct Degree(pub f32);

impl fmt::Display for Vec3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(2);
        write!(
            f,
            "({:.p$}, {:.p$}, {:.p$})",
            self.x,
            self.y,
            self.z,
            p = precision
        )
    }
}
impl fmt::Display for Point3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let precision = f.precision().unwrap_or(2);
        write!(
            f,
            "[{:.p$}, {:.p$}, {:.p$}]",
            self.x,
            self.y,
            self.z,
            p = precision
        )
    }
}

impl Vec3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3 { x, y, z }
    }
    pub fn as_triple(self) -> (f32, f32, f32) {
        (self.x, self.y, self.z)
    }
    pub fn as_vec4(self) -> Vec4 {
        Vec4::new(self.x, self.y, self.z, 0.0)
    }
    pub const X: Vec3 = Self::new(1.0, 0.0, 0.0);
    pub const Y: Vec3 = Self::new(0.0, 1.0, 0.0);
    pub const Z: Vec3 = Self::new(0.0, 0.0, 1.0);
    pub const ZERO: Vec3 = Self::new(0.0, 0.0, 0.0);

    pub fn dot(self, v: Vec3) -> f32 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }
    pub fn cross(self, v: Vec3) -> Vec3 {
        // x1 y1 z1
        // x2 y2 z2
        // i  j  k
        Vec3::new(
            self.y * v.z - self.z * v.y,
            self.z * v.x - self.x * v.z,
            self.x * v.y - self.y * v.x,
        )
    }

    pub fn norm_squared(self) -> f32 {
        self.dot(self)
    }
    pub fn norm(self) -> f32 {
        f32::sqrt(self.norm_squared())
    }
    pub fn is_zero(self) -> bool {
        self.norm_squared() == 0.0
    }

    /// Returns a normalized (unit-length) `self` vector.
    /// Panics if the vector length is zero, NaN or infinite.
    pub fn hat(self) -> Vec3 {
        let norm2 = self.norm_squared();
        assert!(norm2 != 0.0 && norm2.is_finite());
        let inv_sqrt = 1.0 / self.norm();
        self * inv_sqrt
    }
    pub fn try_hat(self) -> Option<Self> {
        let inv_length = 1.0 / self.norm();
        (inv_length.is_finite() && inv_length != 0.0).then(|| inv_length * self)
    }

    /// Chooses from `self` or `-self`, whichever faces a surface having given `normal`.
    pub fn facing(self, normal: Self) -> Self {
        if self.dot(normal).is_sign_negative() {
            self
        } else {
            -self
        }
    }

    /// Projects `self` onto `other`. Both vectors can be arbitrary finite length.
    /// ```
    /// let a = math::hcm::vec3(1.0, 2.5, 0.0);
    /// let b = math::hcm::vec3(0.6, 0.0, 0.0);
    /// let c = b - b.projected_onto(a);
    /// assert!(c.dot(a).abs() < f32::EPSILON, "c = {}, a = {}", c, a);
    ///
    /// let a = math::hcm::vec3(0.19, -0.00, 0.98);
    /// let b = math::hcm::vec3(-9762.44, -17.83, 1851.39);
    /// let c = b - b.projected_onto(a);
    /// assert!(c.dot(a).abs() < f32::EPSILON, "c = {}, a = {}", c, a);
    /// ```
    pub fn projected_onto(self, other: Self) -> Self {
        self.dot(other) * other / other.norm_squared()
    }

    // Returns the index to the element with minimum magnitude.
    pub fn abs_min_dimension(self) -> usize {
        let abs = [self.x.abs(), self.y.abs(), self.z.abs()];
        let res = if abs[0] < abs[1] { 0 } else { 1 };
        let res = if abs[res] < abs[2] { res } else { 2 };
        res
    }

    pub fn max_dimension(self) -> usize {
        let res = if self.x > self.y { 0 } else { 1 };
        if self[2] > self[res] {
            2
        } else {
            res
        }
    }

    pub fn has_nan(self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, other: Self) -> Vec3 {
        Vec3::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}
impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}
impl Add<Point3> for Vec3 {
    type Output = Point3;
    fn add(self, other: Point3) -> Point3 {
        Point3::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, other: Self) -> Vec3 {
        Vec3::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}
impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}
impl Index<usize> for Vec3 {
    type Output = f32;
    fn index(&self, i: usize) -> &f32 {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("invalid index"),
        }
    }
}
impl IndexMut<usize> for Vec3 {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("invalid index"),
        }
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Mul<Vec3> for f32 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}
impl Div<f32> for Vec3 {
    type Output = Self;
    fn div(self, s: f32) -> Vec3 {
        Vec3::new(self.x / s, self.y / s, self.z / s)
    }
}

// Implementation of Points
impl Point3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Point3 {
        Point3 { x, y, z }
    }
    pub const ORIGIN: Point3 = Point3::new(0.0, 0.0, 0.0);
    pub fn as_triple(self) -> (f32, f32, f32) {
        (self.x, self.y, self.z)
    }
    pub fn with_x(self, x: f32) -> Self {
        Self { x, ..self }
    }
    pub fn with_y(self, y: f32) -> Self {
        Self { y, ..self }
    }
    pub fn with_z(self, z: f32) -> Self {
        Self { z, ..self }
    }

    pub fn distance_to(self, p: Self) -> f32 {
        (self - p).norm()
    }
    pub fn squared_distance_to(self, p: Self) -> f32 {
        (self - p).norm_squared()
    }
    pub fn has_nan(self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }
    pub fn as_vec4(self) -> Vec4 {
        Vec4::new(self.x, self.y, self.z, 1.0)
    }
}

impl Add<Vec3> for Point3 {
    type Output = Point3;
    fn add(self, v: Vec3) -> Point3 {
        Point3::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}

impl Sub for Point3 {
    type Output = Vec3;
    fn sub(self, from: Point3) -> Vec3 {
        Vec3::new(self.x - from.x, self.y - from.y, self.z - from.z)
    }
}
impl Sub<Vec3> for Point3 {
    type Output = Point3;
    fn sub(self, t: Vec3) -> Point3 {
        Point3::new(self.x - t.x, self.y - t.y, self.z - t.z)
    }
}
impl Index<usize> for Point3 {
    type Output = f32;
    fn index(&self, i: usize) -> &f32 {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("invalid index"),
        }
    }
}
impl IndexMut<usize> for Point3 {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("invalid index"),
        }
    }
}

// Explicit conversion between Vec3 and Point3.
// -------------------------------------------------------------------------------------------------
impl From<Vec3> for Point3 {
    fn from(v: Vec3) -> Self {
        Point3::new(v.x, v.y, v.z)
    }
}

impl From<Point3> for Vec3 {
    fn from(p: Point3) -> Self {
        Vec3::new(p.x, p.y, p.z)
    }
}

impl From<Vec4> for Vec3 {
    fn from(v4: Vec4) -> Self {
        Vec3::new(v4.x, v4.y, v4.z)
    }
}

impl TryFrom<Vec4> for Point3 {
    type Error = &'static str;
    fn try_from(value: Vec4) -> Result<Self, Self::Error> {
        if value.w == 1.0 {
            Ok(Point3::new(value.x, value.y, value.z))
        } else if value.w == 0.0 {
            Err("homogeneous coordinate is zero")
        } else {
            let w = value.w;
            Ok(Point3::new(value.x / w, value.y / w, value.z / w))
        }
    }
}

/// ------------------------------------------------------------------------------------------------
/// Mat3: implements m * m, m * v, m + m, m - m
#[derive(Debug, Clone, Copy)]
pub struct Mat3 {
    pub cols: [Vec3; 3],
}

#[allow(dead_code)]
impl Mat3 {
    pub const ZERO: Self = Self {
        cols: [Vec3::ZERO; 3],
    };
    pub const IDENTITY: Self = Self {
        cols: [Vec3::X, Vec3::Y, Vec3::Z],
    };
    pub fn from_cols(v0: Vec3, v1: Vec3, v2: Vec3) -> Self {
        Self { cols: [v0, v1, v2] }
    }
    pub fn nonuniform_scale(s: Vec3) -> Self {
        let mut mat = Self::IDENTITY;
        mat.cols[0][0] = s[0];
        mat.cols[1][1] = s[1];
        mat.cols[2][2] = s[2];
        mat
    }
    pub fn scaler(s: f32) -> Self {
        let mut mat = Self::IDENTITY;
        mat.cols[0][0] = s;
        mat.cols[1][1] = s;
        mat.cols[2][2] = s;
        mat
    }
    pub fn rotater_x(angle: crate::Angle) -> Self {
        let (sin_t, cos_t) = angle.sin_cos();
        let rot_y = Vec3::new(0.0, cos_t, sin_t);
        let rot_z = Vec3::new(0.0, -sin_t, cos_t);

        Mat3::from_cols(Vec3::X, rot_y, rot_z)
    }

    pub fn rotater_y(angle: crate::Angle) -> Self {
        let (sin_t, cos_t) = angle.sin_cos();
        let rot_x = Vec3::new(cos_t, 0.0, -sin_t);
        let rot_z = Vec3::new(sin_t, 0.0, cos_t);

        Mat3::from_cols(rot_x, Vec3::Y, rot_z)
    }

    pub fn rotater_z(angle: crate::Angle) -> Self {
        let (sin_t, cos_t) = angle.sin_cos();
        let rot_x = Vec3::new(cos_t, sin_t, 0.0);
        let rot_y = Vec3::new(-sin_t, cos_t, 0.0);

        Mat3::from_cols(rot_x, rot_y, Vec3::Z)
    }
    pub fn rotater(axis: Vec3, angle: crate::Angle) -> Self {
        let mut mat = Self::IDENTITY;
        let (sin_t, cos_t) = angle.sin_cos();
        for i in 0..3 {
            let mut base = Vec3::ZERO;
            base[i] = 1.0;
            let vc = base.dot(axis) * axis / axis.dot(axis);
            let v1 = base - vc;
            let v2 = v1.cross(axis.hat());
            mat.cols[i] = vc + v1 * cos_t + v2 * sin_t;
        }
        mat
    }
    pub fn transpose(&self) -> Self {
        let mut mat = Self::ZERO;
        for i in 0..3 {
            for j in 0..3 {
                mat.cols[i][j] = self.cols[j][i];
            }
        }
        mat
    }
    pub fn frobenius_norm_squared(&self) -> f32 {
        (0..3).map(|i| self.cols[i].norm_squared()).sum()
    }
}

impl Mul for Mat3 {
    type Output = Mat3;
    fn mul(self, m: Self) -> Mat3 {
        let mut mat = Mat3::ZERO;
        for c in 0..3 {
            // TODO verify the correctness
            mat.cols[c] = mat.cols[c] + self * m.cols[c];
        }
        mat
    }
}

impl Mul<Vec3> for Mat3 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        self.cols[0] * v[0] + self.cols[1] * v[1] + self.cols[2] * v[2]
    }
}

impl Mul<f32> for Mat3 {
    type Output = Mat3;
    fn mul(self, f: f32) -> Mat3 {
        Self::from_cols(self.cols[0] * f, self.cols[1] * f, self.cols[2] * f)
    }
}

impl Sub for Mat3 {
    type Output = Mat3;
    fn sub(self, rhs: Mat3) -> Self::Output {
        Self::from_cols(
            self.cols[0] - rhs.cols[0],
            self.cols[1] - rhs.cols[1],
            self.cols[2] - rhs.cols[2],
        )
    }
}

// TODO Quaternion
// -------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub struct Mat4 {
    pub cols: [Vec4; 4],
}

#[allow(dead_code)]
impl Mat4 {
    pub const ZERO: Mat4 = Mat4 {
        cols: [Vec4::ZERO; 4],
    };
    pub const IDENTITY: Mat4 = Mat4 {
        cols: [Vec4::X, Vec4::Y, Vec4::Z, Vec4::W],
    };
    pub fn translater(t: Vec3) -> Mat4 {
        let mut mat = Self::IDENTITY;
        mat.cols[3] = Vec4::new(t.x, t.y, t.z, 1.0);
        mat
    }
    pub fn nonuniform_scale(s: Vec3) -> Mat4 {
        let mut mat = Self::IDENTITY;
        mat.cols[0][0] = s[0];
        mat.cols[1][1] = s[1];
        mat.cols[2][2] = s[2];
        mat
    }
    pub fn scaler(s: f32) -> Mat4 {
        let mut mat = Self::IDENTITY;
        mat.cols[0][0] = s;
        mat.cols[1][1] = s;
        mat.cols[2][2] = s;
        mat
    }
    pub fn rotater(axis: Vec3, angle: crate::Angle) -> Mat4 {
        let mut mat = Self::IDENTITY;
        let (sin_t, cos_t) = angle.sin_cos();
        for i in 0..3 {
            let mut base = Vec3::ZERO;
            base[i] = 1.0;
            let vc = base.dot(axis) * axis / axis.dot(axis);
            let v1 = base - vc;
            let v2 = v1.cross(axis.hat());
            mat.cols[i] = (vc + v1 * cos_t + v2 * sin_t).as_vec4();
        }
        mat
    }
    pub fn transpose(&self) -> Mat4 {
        let mut mat = Self::ZERO;
        for i in 0..4 {
            for j in 0..4 {
                mat.cols[i][j] = self.cols[j][i];
            }
        }
        mat
    }
    pub fn orientation(&self) -> Mat3 {
        Mat3::from_cols(
            self.cols[0].into(),
            self.cols[1].into(),
            self.cols[2].into(),
        )
    }
}

impl Mul<Vec4> for Mat4 {
    type Output = Vec4;
    fn mul(self, v: Vec4) -> Vec4 {
        self.cols[0] * v[0] + self.cols[1] * v[1] + self.cols[2] * v[2] + self.cols[3] * v[3]
    }
}

impl Mul for Mat4 {
    type Output = Mat4;
    fn mul(self, m: Self) -> Mat4 {
        let mut mat = Mat4::ZERO;
        for c in 0..4 {
            // TODO verify the correctness and implement AddAssign trait.
            mat.cols[c] = mat.cols[c] + self * m.cols[c];
        }
        mat
    }
}

impl Mul<Vec3> for Mat4 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        let v4 = self.cols[0] * v[0] + self.cols[1] * v[1] + self.cols[2] * v[2];
        Vec3::new(v4.x, v4.y, v4.z)
    }
}

impl Mul<Point3> for Mat4 {
    type Output = Point3;
    fn mul(self, p: Point3) -> Self::Output {
        let v4 = self * p.as_vec4();
        if v4.w == 1.0 {
            Point3::new(v4.x, v4.y, v4.z)
        } else {
            Point3::new(v4.x / v4.w, v4.y / v4.w, v4.z / v4.w)
        }
    }
}

// Mod-level functions
#[allow(dead_code)]
pub fn normalize(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(x, y, z).hat()
}

/// Computes a pair of unit-vectors that forms a orthonormal matrix with `v`.
/// ```
/// use math::hcm::{Vec3, Mat3, make_coord_system};
/// let v0 = Vec3::new(0.3, 0.4, -0.6).hat();
/// let (v1, v2) = make_coord_system(v0);
///
/// let basis = Mat3::from_cols(v0, v1, v2);
/// // basis * basis^T should be identity.
/// let diff_to_eye = basis * basis.transpose() - Mat3::IDENTITY;
/// assert!(diff_to_eye.frobenius_norm_squared() < f32::EPSILON);
/// ```
pub fn make_coord_system(v: Vec3) -> (Vec3, Vec3) {
    let i0 = v.abs_min_dimension();
    let (i1, i2) = ((i0 + 1) % 3, (i0 + 2) % 3);
    let mut v1 = Vec3::ZERO;
    // v = [x, y, z] -> [x, 0, z], v1 = [-z, 0, x]
    v1[i1] = v[i2];
    v1[i2] = -v[i1];
    assert!(v1.dot(v).abs() < f32::EPSILON);
    let v2 = v.cross(v1);
    (v1.hat(), v2.hat())
}

pub fn reflect(normal: Vec3, wi: Vec3) -> Vec3 {
    let perp = wi.dot(normal) * normal / normal.norm_squared();
    let parallel = wi - perp;
    wi - 2.0 * parallel
}

pub enum Refract {
    FullReflect(Vec3),
    Transmit(Vec3),
}

pub use Refract::FullReflect;
pub use Refract::Transmit;

/// Refracts incident light `wi` with regard to `normal`.
/// - `normal` is assumed to be unit-length and forms an acute angle with `wi`.
/// - `ni` and `no` are refraction indices.
/// If `ni`/`no` > 1 (e.g., from water to air), there is a chance of full reflection.
pub fn refract(normal: Vec3, wi: Vec3, ni_over_no: f32) -> Refract {
    let wi = wi.hat();
    let normal = normal.hat();
    let cos_theta_i = wi.dot(normal);
    crate::assert_ge!(cos_theta_i, 0.0);
    let sin2_theta_i = (1.0 - cos_theta_i.powi(2)).max(0.0);
    // sin_i * ni = sin_o * no => sin_o = sin_i * ni_over_no
    let sin2_theta_o = sin2_theta_i * ni_over_no.powi(2);
    if sin2_theta_o >= 1.0 {
        FullReflect(reflect(normal, wi))
    } else {
        let cos_theta_o = (1.0 - sin2_theta_o).sqrt();
        let refracted = ni_over_no * -wi + (ni_over_no * cos_theta_i - cos_theta_o) * normal;
        Transmit(refracted)
    }
}

/// Computes a unit-vector on a unit-sphere given longitude and latitude values.
///
/// The computed vector is (0, 0, 1) rotated `theta` radians away from the z-axis and then rotates
/// around the z-axis with angle `phi`. Note that sin(theta) and cos(theta) values are passed in
/// as usually the trigonometry values are more directly available rather than the angle itself.
pub fn spherical_direction(sin_theta: f32, cos_theta: f32, phi: crate::Angle) -> Vec3 {
    let (cos_phi, sin_phi) = phi.sin_cos();
    Vec3::new(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta)
}

#[macro_export]
macro_rules! assert_close {
    ($left:expr, $right:expr) => {
        if ($left - $right).norm_squared() > 1e-4 {
            panic!(
                "Assertion failed: Close({}, {}) values: {} vs. {}, dist = {}",
                stringify!($left),
                stringify!(right),
                $left,
                $right,
                ($left - $right).norm()
            )
        }
    };
}

#[cfg(test)]
mod test {
    type Vec3 = super::Vec3;
    #[test]
    fn test_reflect() {
        let normal = Vec3::Y;
        let wi = Vec3::new(2.0, 1.0, 0.5);
        let wo = Vec3::new(-2.0, 1.0, -0.5);
        let reflect_wi = super::reflect(normal, wi);
        eprintln!("reflect_wi = {}", reflect_wi);
        assert!((reflect_wi - wo).norm_squared() < f32::EPSILON);
    }
    #[test]
    fn test_refract() {
        let normal = Vec3::Y * 6.0;
        let wi = Vec3::new(1.0, 1.0, 0.0).hat();
        let wo = Vec3::new(-0.5, -0.5 * 3.0f32.sqrt(), 0.0);
        let refract_wo = super::refract(normal, wi, 0.5f32.sqrt());
        match refract_wo {
            super::FullReflect(_) => panic!(""),
            super::Transmit(v) => {
                assert!((wo - v).norm_squared() < f32::EPSILON, "{} vs {}", v, wo)
            }
        }

        // The critical angle for a "glass"-to-air IOR of 2.0 is 30 degrees.
        // One corresponding incident direction is (0.5, sqrt(0.75), 0.0).
        let full_reflect_wi = Vec3::new(0.51, 0.75f32.sqrt(), 0.0).hat();
        let transmit_wi = Vec3::new(0.49, 0.75f32.sqrt(), 0.0).hat();
        assert!(matches!(
            super::refract(normal, full_reflect_wi, 2.0),
            super::FullReflect(_)
        ));
        assert!(matches!(
            super::refract(normal, transmit_wi, 2.0),
            super::Transmit(_)
        ));
    }
}
