/// Defines the `BBox` bounding-box type.
pub mod bvh;
pub mod bxdf;
pub mod camera;
pub mod fourier;
pub mod interaction;
pub mod microfacet;
pub mod ray;
pub mod transform;

pub use interaction::Interaction;
pub use transform::{AffineTransform, InstanceTransform, RigidBodyTransform};

use math::hcm;

pub fn compute_normals(
    positions: &Vec<hcm::Point3>, indices: &Vec<(usize, usize, usize)>,
) -> Vec<hcm::Vec3> {
    let mut normals = Vec::new();
    normals.resize(positions.len(), hcm::Vec3::ZERO);
    for ijk in indices.iter() {
        let (i, j, k) = *ijk;
        let p0 = positions[i];
        let p1 = positions[j];
        let p2 = positions[k];
        let n = (p1 - p0).cross(p2 - p0);
        normals[i] += n;
        normals[j] += n;
        normals[k] += n;
    }
    normals.iter().map(|n| n.hat()).collect()
}
