use crate::hcm;
use crate::ray::Ray;
use crate::shape::Interaction;
use crate::{hcm::Vec3, image::Color};
pub trait Material : Sync + Send{
    /// Computes the scattering of a ray on a given surface interation.
    /// Returns the scattered ray and radiance carried.
    fn scatter(&self, wi: Vec3, isect: &Interaction) -> (Ray, Color);
    fn emission(&self) -> Color {
        Color::black()
    }
}

pub struct Lambertian {
    pub albedo: Color,
}

impl Lambertian {
    pub fn new(albedo: Color) -> Self {
        Lambertian{albedo}
    }
}
pub struct Metal {
    pub albedo: Color,
    pub fuzziness: f32,
}
impl Metal {
    pub fn new(albedo: Color, fuzziness: f32) -> Self {
        Self{albedo, fuzziness}
    }
}
pub struct Dielectric {
    pub refract_index: f32,
}
impl Dielectric {
    pub fn new(refract_index: f32) -> Self {
        Self{refract_index}
    }
}

// Utility functions
// ------------------------------------------------------------------------------------------------

pub fn uniform_sphere() -> Vec3 {
    let u = rand::random::<f32>();
    let v = rand::random::<f32>();

    let theta = 2.0 * std::f32::consts::PI * u;
    let phi = (2.0 * v - 1.0).acos();
    Vec3::new(
        phi.sin() * theta.cos(),
        phi.sin() * theta.sin(),
        2.0 * v - 1.0,
    )
}

pub fn uniform_hemisphere() -> Vec3 {
    let u = rand::random::<f32>();
    let v = rand::random::<f32>();

    let theta = 2.0 * std::f32::consts::PI * u;
    let phi = v.acos();
    Vec3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), v)
}

// Implements the Material trait for muliple materials.
// ------------------------------------------------------------------------------------------------

impl Material for Lambertian {
    fn scatter(&self, _: Vec3, isect: &Interaction) -> (Ray, Color) {
        let h = uniform_hemisphere();
        let (nx, ny) = hcm::make_coord_system(isect.normal);
        let wo = isect.normal + nx * h.x + ny * h.y + isect.normal * h.z;
        let ray_out = Ray::new(isect.pos, wo);
        (ray_out, self.albedo)
    }
}

impl Material for Metal {
    fn scatter(&self, wi: Vec3, isect: &Interaction) -> (Ray, Color) {
        let reflected = hcm::reflect(isect.normal, wi);
        let ray_out = Ray::new(isect.pos, reflected + uniform_sphere() * self.fuzziness);
        (ray_out, self.albedo)
    }
}

impl Material for Dielectric {
    fn scatter(&self, wi: Vec3, isect: &Interaction) -> (Ray, Color) {
        let wi = wi.hat();
        let (outward_normal, ni_over_no, cosine) = if isect.normal.dot(wi) < 0.0 {
            // Ray hits the inside of the surface.
            (-isect.normal, self.refract_index, -isect.normal.dot(wi))
        } else {
            (isect.normal, 1.0 / self.refract_index, isect.normal.dot(wi))
        };
        let reflected = hcm::reflect(outward_normal, wi);
        let (mut wo, reflect_pr) = match hcm::refract(outward_normal, wi, ni_over_no) {
            hcm::Transmit(wt) => (wt, schlick(cosine, self.refract_index)),
            hcm::FullReflect(_) => (reflected, 1.0),
        };
        if rand::random::<f32>() < reflect_pr {
            wo = reflected;
        }
        (Ray::new(isect.pos, wo), Color::white())
    }
}

fn schlick(cosine: f32, ref_index: f32) -> f32 {
    let r0 = (1.0 - ref_index) / (1.0 + ref_index);
    let r0 = r0 * r0;
    r0 + (1.0 - r0) * (1.0 - cosine).powi(5)
}

#[cfg(test)]
mod test {
    #[test]
    fn test_random_unit_sphere() {
        for _ in 0..64 {
            let v = super::uniform_sphere();
            let diff = v.norm_squared() - 1.0;
            assert!(diff.abs() < f32::EPSILON * 3.0, "{}", v.norm_squared());
        }
    }

    #[test]
    fn test_random_unit_hemisphere() {
        for _ in 0..64 {
            let v = super::uniform_hemisphere();
            let diff = v.norm_squared() - 1.0;
            assert!(diff.abs() < f32::EPSILON * 3.0, "{}", v.norm_squared());
        }
    }
}
