use std::{f32::consts::PI, fmt, ops::{Add, Div, Index, IndexMut, Mul, Neg, Sub}};


/// Represents a 3D vector. Each component is a `f32` number.
/// Components can be accessed using `v.x` `v.y` `v.z`,
/// or indices `v[i]` where i is 0, 1, or 2.
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Debug, Copy, Clone)]
pub struct Point3 {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct Radian(pub f32);

#[derive(Debug, Clone, Copy)]
pub struct Degree(pub f32);

impl From<Degree> for Radian {
    fn from(d: Degree) -> Self {
        let Degree(deg) = d;
        Radian(deg * std::f32::consts::PI / 180.0)
    }
}
impl Degree {
    pub fn to_radian(self) -> Radian {
        let Degree(d) = self;
        Radian(d * PI / 180.0)
    }
}

impl fmt::Display for Degree {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}deg", self.0)
    }
}
impl fmt::Display for Radian {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}rad", self.0)
    }
}

impl fmt::Display for Vec3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x, self.y, self.z)
    }
}
impl fmt::Display for Point3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{}, {}, {}]", self.x, self.y, self.z)
    }
}

impl Vec3 {
    pub fn new(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3 { x, y, z }
    }
    pub fn xbase() -> Vec3 {
        Self::new(1.0, 0.0, 0.0)
    }
    pub fn ybase() -> Vec3 {
        Self::new(0.0, 1.0, 0.0)
    }
    pub fn zbase() -> Vec3 {
        Self::new(0.0, 0.0, 1.0)
    }
    pub fn zero() -> Vec3 {
        Self::new(0.0, 0.0, 0.0)
    }

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
    pub fn hat(self) -> Vec3 {
        if self.is_zero() {
            eprintln!("normalizing zero vector");
            self
        } else {
            let inv_sqrt = 1.0 / self.norm();
            self * inv_sqrt
        }
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, other: Self) -> Vec3 {
        Vec3::new(self.x + other.x, self.y + other.y, self.z + other.z)
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
            _ => panic!("invalid index")
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

pub fn normalize(v: Vec3) -> Vec3 {
    v.hat()
}

// Implementation of Points
impl Point3 {
    pub fn new(x: f32, y: f32, z: f32) -> Point3 {
        Point3 { x, y, z }
    }
    pub fn origin() -> Point3 {
        Point3::new(0.0, 0.0, 0.0)
    }

    pub fn distance_to(self, p: Self) -> f32 {
        (self - p).norm()
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

#[derive(Debug, Clone, Copy)]
struct Vec4 {
    x: f32,
    y: f32,
    z: f32,
    w: f32,
}

impl fmt::Display for Vec4 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {}, {})", self.x, self.y, self.z, self.w)
    }
}

impl Vec4 {
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Vec4 {
        Vec4 { x, y, z, w }
    }
    pub fn zero() -> Vec4 {
        Vec4::new(0.0, 0.0, 0.0, 0.0)
    }
}

impl From<Vec3> for Vec4 {
    fn from(v3: Vec3) -> Self {
        Vec4::new(v3.x, v3.y, v3.z, 0.0)
    }
}

impl From<Point3> for Vec4 {
    fn from(v3: Point3) -> Self {
        Vec4::new(v3.x, v3.y, v3.z, 1.0)
    }
}

impl Add for Vec4 {
    type Output = Vec4;
    fn add(self, rhs: Self) -> Self {
        Self::new(
            self.x + rhs.x,
            self.y + rhs.y,
            self.z + rhs.z,
            self.w + rhs.w,
        )
    }
}

impl Sub for Vec4 {
    type Output = Vec4;
    fn sub(self, rhs: Self) -> Self {
        Self::new(
            self.x - rhs.x,
            self.y - rhs.y,
            self.z - rhs.z,
            self.w - rhs.w,
        )
    }
}

impl Mul<f32> for Vec4 {
    type Output = Vec4;
    fn mul(self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s, self.w * s)
    }
}

impl Div<f32> for Vec4 {
    type Output = Vec4;
    fn div(self, divisor: f32) -> Self {
        let s = 1.0 / divisor;
        Self::new(self.x * s, self.y * s, self.z * s, self.w * s)
    }
}

impl Index<usize> for Vec4 {
    type Output = f32;
    fn index(&self, i: usize) -> &f32 {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            3 => &self.w,
            _ => panic!("invalid index"),
        }
    }
}

impl IndexMut<usize> for Vec4 {
    fn index_mut(&mut self, i: usize) -> &mut f32 {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            3 => &mut self.w,
            _ => panic!("invalid index"),
        }
    }
}

#[derive(Debug,Clone,Copy)]
pub struct Mat4 {
    cols: [Vec4; 4],
}

#[allow(dead_code)]
impl Mat4 {
    pub fn zero() -> Mat4 {
        Mat4 {
            cols: [Vec4::zero(); 4],
        }
    }
    pub fn identity() -> Mat4 {
        let mut mat = Self::zero();
        for i in 0..4 {
            mat.cols[i][i] = 1.0
        }
        mat
    }
    pub fn translater(t: Vec3) -> Mat4 {
        let mut mat = Self::identity();
        mat.cols[3] = Vec4::new(t.x, t.y, t.z, 1.0);
        mat
    }
    pub fn nonuniform_scale(s : Vec3) -> Mat4 {
        let mut mat = Self::identity();
        mat.cols[0][0] = s[0];
        mat.cols[1][1] = s[1];
        mat.cols[2][2] = s[2];
        mat
    }
    pub fn scaler(s: f32) -> Mat4 {
        let mut mat = Self::identity();
        mat.cols[0][0] = s;
        mat.cols[1][1] = s;
        mat.cols[2][2] = s;
        mat
    }
    pub fn rotater(axis: Vec3, angle: Radian) -> Mat4 {
        let Radian(rad) = angle;
        let mut mat = Self::identity();
        let cos_t = f32::cos(rad);
        let sin_t = f32::sin(rad);
        for i in 0..3 {
            let mut base = Vec3::zero();
            base[i] = 1.0;
            let vc = base.dot(axis) * axis / axis.dot(axis);
            let v1 = base - vc;
            let v2 = v1.cross(axis.hat());
            mat.cols[i] = Vec4::from(vc + v1 * cos_t + v2 * sin_t);
        }
        mat
    }
    pub fn transpose(&self) -> Mat4 {
        let mut mat = Self::zero();
        for i in 0..4 {
            for j in 0..4 {
               mat.cols[i][j] = self.cols[j][i]; 
            }
        }
        mat
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
        let mut mat = Mat4::zero();
        for c in 0..4 {
            mat.cols[c] = self * m.cols[c];
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
        let v4 = self * Vec4::from(p);
        if v4.w == 1.0 {
            Point3::new(v4.x, v4.y, v4.z)
        } else {
            Point3::new(v4.x / v4.w, v4.y / v4.w, v4.z / v4.w)
        }
    }
}