mod blas;
mod simple;

use geometry::bvh::BBox;
use geometry::interaction::Interaction;
use geometry::ray::Ray;

pub use blas::*;
pub use simple::*;

/// Represents the characteristics of a shape: has a bounding box, and can interact with a ray.
/// - See `simple.rs` for basic shape implementations: `Sphere`, `Quad`, and `Cuboid`.
/// - See `blas.rs` for aggregated shapes: `IsoBlas` and `TriangleMesh`.
pub trait Shape: Send + Sync {
    fn summary(&self) -> String;
    fn bbox(&self) -> BBox;
    fn intersect(&self, r: &Ray) -> Option<Interaction>;
    fn occludes(&self, r: &Ray) -> bool;
}
