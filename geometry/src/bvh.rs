use std::fmt::{Debug, Display, Formatter, Result};

use crate::ray::Ray;
use math::hcm::{Point3, Vec3};

/// 3D bounding-box type. Boundary check is half-open (`[min, max)`) on all axes.
/// - Build one from 2 `Point3`s;
/// - Expand it by `b.union()` or `union(b1, b2)`;
/// - Check if it `contains()` a point or `encloses()` another box, or `intersects()` with a `Ray`.
#[derive(Debug, Clone, Copy)]
pub struct BBox {
    min: glam::Vec3A,
    max: glam::Vec3A,
}

impl BBox {
    pub fn empty() -> BBox {
        BBox {
            min: glam::Vec3A::splat(f32::INFINITY),
            max: glam::Vec3A::splat(-f32::INFINITY),
        }
    }
    pub fn has_nan(&self) -> bool {
        self.min.is_nan() || self.max.is_nan()
    }
    pub fn new(p0: Point3, p1: Point3) -> BBox {
        let p0 = glam::Vec3A::new(p0.x, p0.y, p0.z);
        let p1 = glam::Vec3A::new(p1.x, p1.y, p1.z);
        BBox {
            min: p0.min(p1),
            max: p0.max(p1),
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
        let p = (self.max - self.min) * 0.5 + self.min;
        Point3::new(p.x, p.y, p.z)
    }

    pub fn diag(&self) -> Vec3 {
        let d = self.max - self.min;
        Vec3::new(d.x, d.y, d.z)
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
        Point3::new(self.min.x, self.min.y, self.min.z)
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

        let (dirx, diry, dirz) = r.dir.as_triple();
        let (ox, oy, oz) = r.origin.as_triple();

        use glam::Vec3A;
        let dir = Vec3A::new(dirx, diry, dirz);
        let o = Vec3A::new(ox, oy, oz);

        let t0 = (self.min - o) / dir;
        let t1 = (self.max - o) / dir;
        let t_low = t0.min(t1).max_element().max(0.0);
        let t_high = t0.max(t1).min_element().min(r.t_max);
        let unstable = t_low <= t_high;

        unstable
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
    BBox {
        min: b0.min.min(b1.min),
        max: b0.max.max(b1.max),
    }
}

#[cfg(test)]
mod test {
    use crate::bvh::BBox;
    use crate::ray::Ray;
    use math::hcm::{point3, vec3};
    #[test]
    fn stable_box_test() {
        let bbox = BBox::new(
            point3(-17.027893, 16.054487, 3.048435),
            point3(-16.821342, 23.935837, 12.119018),
        );
        let ray = Ray::new(
            point3(0.0, 23.0, 30.0),
            vec3(-0.82937318, -0.08858252, -0.87960643),
        );

        assert!(bbox.intersect(&ray));
    }
}
