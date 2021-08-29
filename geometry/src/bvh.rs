use std::fmt::{Debug, Display, Formatter, Result};

use crate::ray::Ray;
use math::{
    float::min_max,
    hcm::{Point3, Vec3},
};

/// 3D bounding-box type. Boundary check is half-open (`[min, max)`) on all axes.
/// - Build one from 2 `Point3`s;
/// - Expand it by `b.union()` or `union(b1, b2)`;
/// - Check if it `contains()` a point or `encloses()` another box, or `intersects()` with a `Ray`.
#[derive(Debug, Clone, Copy)]
pub struct BBox {
    min: Point3,
    max: Point3,
}

impl BBox {
    pub fn empty() -> BBox {
        BBox {
            min: Point3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            max: Point3::new(-f32::INFINITY, -f32::INFINITY, -f32::INFINITY),
        }
    }
    pub fn new(p0: Point3, p1: Point3) -> BBox {
        let (xmin, xmax) = min_max(p0.x, p1.x);
        let (ymin, ymax) = min_max(p0.y, p1.y);
        let (zmin, zmax) = min_max(p0.z, p1.z);
        BBox {
            min: Point3::new(xmin, ymin, zmin),
            max: Point3::new(xmax, ymax, zmax),
        }
    }

    pub fn union(self, p: Point3) -> BBox {
        let mut result = self;
        for i in 0..3 {
            result.min[i] = self.min[i].min(p[i]);
            result.max[i] = self.max[i].max(p[i]);
        }
        result
    }

    pub fn midpoint(self) -> Point3 {
        (self.max - self.min) * 0.5 + self.min
    }

    pub fn diag(&self) -> Vec3 {
        self.max - self.min
    }

    #[allow(dead_code)]
    pub fn all_corners(&self) -> [Point3; 8] {
        let mut res = [Point3::origin(); 8];

        for i in 0..8 {
            for axis in 0..3 {
                res[i][axis] = if i & (1 << axis) == 0 {
                    self.min[axis]
                } else {
                    self.max[axis]
                };
            }
        }

        res
    }
    pub fn min(&self) -> Point3 {
        self.min
    }

    /// Computes the surface area of the bounding box.
    pub fn area(&self) -> f32 {
        let Vec3 { x, y, z } = self.diag();
        if x.is_sign_positive() && y.is_sign_positive() && z.is_sign_positive() {
            (x * y + y * z + z * x) * 2.0
        } else {
            0.0
        }
    }

    pub fn intersect(&self, r: &Ray) -> bool {
        let (mut t_min, mut t_max) = (0.0f32, r.t_max);
        for axis in 0..3 {
            let inv_dir = 1.0 / r.dir[axis];
            let t0 = (self.min[axis] - r.origin[axis]) * inv_dir;
            let t1 = (self.max[axis] - r.origin[axis]) * inv_dir;
            let (t0, t1) = min_max(t0, t1);
            // Shrinks [t_min, t_max] by intersecting it with [t0, t1].
            t_min = t_min.max(t0);
            t_max = t_max.min(t1);
            if t_max < t_min {
                return false;
            }
        }
        return true;
    }

    pub fn encloses(&self, other: Self) -> bool {
        for axis in 0..3 {
            if self.min[axis] > other.min[axis] {
                return false;
            }
            if self.max[axis] < other.max[axis] {
                return false;
            }
        }
        true
    }
    #[allow(dead_code)]
    pub fn contains(&self, p: Point3) -> bool {
        for axis in 0..3 {
            if self.min[axis] > p[axis] {
                return false;
            }
            if self.max[axis] < p[axis] {
                return false;
            }
        }
        true
    }
}

impl Display for BBox {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "box[{} -> {}]", self.min, self.max)
    }
}

pub fn union(b0: BBox, b1: BBox) -> BBox {
    b0.union(b1.min).union(b1.max)
}
