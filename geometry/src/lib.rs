/// Defines the `BBox` bounding-box type.
pub mod bvh;
pub mod camera;
pub mod ray;
pub mod bxdf;
pub mod microfacet;
pub mod fourier;
pub mod transform;
pub mod interaction;

pub use interaction::Interaction;
pub use transform::{AffineTransform, RigidBodyTransform, InstanceTransform};