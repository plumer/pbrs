use crate::hcm;

#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: hcm::Point3,
    pub dir: hcm::Vec3,
    pub t_max: f32
}

impl Ray {
    pub fn new(origin: hcm::Point3, dir : hcm::Vec3) -> Ray {
        Ray{origin, dir, t_max: f32::INFINITY}
    }
    pub fn set_extent(&mut self, t_max: f32) {
        self.t_max = t_max;
    }
    /// Returns `None` if the given `t` is outside the ray's extent [0.0, `r.t_max`).
    /// `Some(t)` otherwise.
    pub fn truncated_t(&self, t: f32) -> Option<f32> {
        if t < f32::EPSILON || t >= self.t_max {
            None
        } else {
            Some(t)
        }
    }

    pub fn position_at(&self, t: f32) -> hcm::Point3 {
        self.origin + t * self.dir
    }
}