use crate::hcm::{Point3, Vec3};
use crate::ray::Ray;

#[derive(Debug, Clone, Copy)]
pub struct Interaction {
    pos: Point3,
    ray_t: f32,
    normal: Vec3,
    albedo: Vec3,
}

#[allow(dead_code)]
pub struct BBox {
    p_min: Point3,
    p_max: Point3,
}

pub trait Shape {
    fn intersect(&self, r: &Ray) -> Option<Interaction>;
    fn bbox(&self) -> BBox;
}

pub struct Sphere {
    center: Point3,
    radius: f32,
}

impl Sphere {
    pub fn new(center: Point3, radius: f32) -> Sphere {
        Sphere{center, radius}
    }
}

impl Shape for Sphere {
    fn bbox(&self) -> BBox {
        BBox {
            p_min: Point3::new(-self.radius, -self.radius, -self.radius),
            p_max: Point3::new(self.radius, self.radius, self.radius),
        }
    }
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        // r = o + td
        // sphere: (p-c)(p-c) = radius^2
        // (td + o - c)^2 = radius^2
        // t^2 d^2 + (o-c)^2 + 2t d * (o-c) = radius^2
        // delta = 4(d*(o-c))^2 - 4d^2((o-c)^2 - radius^2)
        let Ray { origin, dir, .. } = r.clone();

        let dir = dir.hat();
        
        let f = origin - self.center; // vector connecting the sphere center to ray origin.
        let q = f.dot(dir) * dir; // r.o + q gives the point closest to sphere center.
        let delta = self.radius * self.radius - (f - q).norm_squared();
        let (t_low, t_high) = if delta < 0.0 {
            return None;
        } else {
            let c = f.norm_squared() - self.radius * self.radius;
            let neg_b = -dir.dot(f);
            let t0 = neg_b + f32::signum(neg_b) * f32::sqrt(delta);
            let t1 = c / t0;
            if t0 < t1 {
                (t0, t1)
            } else {
                (t1, t0)
            }
        };
        // Keeps only the roots that are within [0, r.t_max).
        let truncated_roots = (r.truncated_t(t_low), r.truncated_t(t_high));
        let best_t: Option<f32> = match truncated_roots {
            (None, None) => None,
            (None, Some(high)) => Some(high),
            (Some(low), _) => Some(low), // It doesn't matter if the greater one is valid or not.
        };
        // print!("{:?}", best_t);
        match best_t {
            None => None,
            Some(t) => Some(Interaction {
                pos: origin + dir * t,
                ray_t: t,
                normal: origin + dir * t - self.center,
                albedo: Vec3::zero(),
            }),
        }
    }
}
