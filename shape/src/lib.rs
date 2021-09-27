mod blas;
mod simple;
mod interaction;

pub use blas::*;
pub use simple::*;

use geometry::bvh::BBox;
use geometry::ray::Ray;

pub use interaction::Interaction;

/// Represents the characteristics of a shape: has a bounding box, and can interact with a ray.
/// - See `simple.rs` for basic shape implementations: `Sphere`, `Quad`, and `Cuboid`.
/// - See `blas.rs` for aggregated shapes: `IsoBlas` and `TriangleMesh`.
pub trait Shape: Send + Sync {
    fn intersect(&self, r: &Ray) -> Option<Interaction>;
    fn bbox(&self) -> BBox;
    fn summary(&self) -> String;
}
