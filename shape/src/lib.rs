mod blas;
mod simple;

pub use blas::*;
pub use simple::*;

use std::fmt::{Display, Formatter, Result};

use geometry::bvh::BBox;
use geometry::ray::Ray;
use math::hcm::{Point3, Vec3};

#[derive(Debug, Clone, Copy)]
pub struct Interaction {
    pub pos: Point3,
    pub ray_t: f32,
    pub uv: (f32, f32),
    pub normal: Vec3,
    pub dpdu: Vec3,
    // pub dpdv: Vec3
}

impl Interaction {
    pub fn new(pos: Point3, ray_t: f32, uv: (f32, f32), normal: Vec3) -> Interaction {
        Interaction {
            pos,
            ray_t,
            uv,
            normal,
            dpdu: Vec3::zero(),
        }
    }
    pub fn with_dpdu(self, dpdu: Vec3) -> Interaction {
        Self { dpdu, ..self }
    }
}

impl Display for Interaction {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let (u, v) = self.uv;
        write!(
            f,
            "pos = {}, t = {:.2}, uv = ({:.2}, {:.2}), normal = {}",
            self.pos, self.ray_t, u, v, self.normal
        )
    }
}

/// Represents the characteristics of a shape: has a bounding box, and can interact with a ray.
/// - See `simple.rs` for basic shape implementations: `Sphere`, `Quad`, and `Cuboid`.
/// - See `blas.rs` for aggregated shapes: `IsoBlas` and `TriangleMesh`.
pub trait Shape: Send + Sync {
    fn intersect(&self, r: &Ray) -> Option<Interaction>;
    fn bbox(&self) -> BBox;
    fn summary(&self) -> String;
}
