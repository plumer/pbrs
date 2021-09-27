use geometry::bxdf::Omega;
use geometry::ray::Ray;
use math::{hcm::{Mat3, Point3, Vec3}};
use std::fmt::{Display, Formatter, Result};

/// Contains geometric information on a ray-surface intersection:
///  - `pos`: position of intersection
///  - `ray_t`: t-value of the ray at the intersection.
///  - `normal`: normal vector of the surface.
///  - `uv`, `dpdu`: shape-specific definition of 
#[derive(Debug, Clone, Copy)]
pub struct Interaction {
    pub pos: Point3,
    pub ray_t: f32,
    pub uv: (f32, f32),
    pub normal: Vec3,
    // pub dpdv: Vec3
    tbn_frame: Mat3,
}

impl Interaction {
    pub fn new(pos: Point3, ray_t: f32, uv: (f32, f32), normal: Vec3) -> Interaction {
        Interaction {
            pos,
            ray_t,
            uv,
            normal,
            tbn_frame: Mat3::zero(),
        }
    }
    
    pub fn tangent(&self) -> Vec3 {
        self.tbn_frame.cols[0]
    }

    /// Builds the tangent-bitangent-normal frame with the given tangent and existing normal.
    pub fn with_dpdu(self, dpdu: Vec3) -> Interaction {
        let normal = self.normal.hat();
        let bitangent = (normal.cross(dpdu)).hat();
        let dpdu = bitangent.cross(normal);
        
        let det = dpdu.cross(bitangent).dot(normal);
        assert!((det - 1.0).abs() < 1e-4, "{}", det);
        Self {
            tbn_frame: Mat3::from_vectors(dpdu, bitangent, normal),
            ..self
        }
    }
    pub fn world_to_local(self, world: Vec3) -> Omega {
        let cols = self.tbn_frame.cols;
        assert!(self.has_valid_frame());
        Omega::normalize(cols[0].dot(world), cols[1].dot(world), cols[2].dot(world))
    }

    pub fn local_to_world(self, local: Omega) -> Vec3 {
        let cols = self.tbn_frame.cols;
        assert!(self.has_valid_frame());
        local.x() * cols[0] + local.y() * cols[1] + local.z() * cols[2]
    }

    pub fn spawn_ray(&self, dir: Vec3) -> Ray {
        let out_normal = dir.dot(self.normal).signum() * self.normal;
        Ray::new(self.pos + out_normal * 0.001, dir)
    }
    
    pub fn has_valid_frame(&self) -> bool {
        let cols = self.tbn_frame.cols;
        let det = cols[0].cross(cols[1]).dot(cols[2]);
        (det - 1.0).abs() < 1e-4
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
