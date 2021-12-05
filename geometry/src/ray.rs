use std::fmt::{Display, Formatter, Result};

use math::hcm;

/// Represents a ray:
///
///   origin + t * direction
///
/// where t is positive.
///
/// The extent of the ray is by default infinite, but can be set to a positive number in order to
/// accelerate intersection tests.
///
/// A `Ray` object can be used to intersect a `Shape`, a `BBox`, and an `Instance`. Please see their
/// respective documentation for details.
#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: hcm::Point3,
    pub dir: hcm::Vec3,
    pub t_max: f32,
}

impl Ray {
    pub fn new(origin: hcm::Point3, dir: hcm::Vec3) -> Self {
        Ray {
            origin,
            dir,
            t_max: f32::INFINITY,
        }
    }
    pub fn set_extent(&mut self, t_max: f32) {
        self.t_max = t_max;
    }

    pub fn with_extent(self, t_max: f32) -> Self {
        Ray { t_max, ..self }
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

impl Display for Ray {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let precision = f.precision().unwrap_or(2);
        write!(
            f,
            "{:.precision$} + t{:.precision$}",
            self.origin,
            self.dir,
            precision = precision
        )
    }
}
