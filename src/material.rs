use crate::ray::Ray;
use crate::shape::Interaction;
use texture::{self, *};
use geometry::bxdf::{self, BxDF};
use geometry::microfacet;
use math::hcm::{self, Vec3};
use radiometry::color::Color;
use std::sync::Arc;
pub trait Material: Sync + Send {
    /// Computes the scattering of a ray on a given surface interation.
    /// Returns the scattered ray and radiance carried.
    ///
    /// If the returned color is black, then there's a possibility that the
    /// material is emissive.
    fn scatter(&self, wi: Vec3, isect: &Interaction) -> (Ray, Color);

    /// Computes the scattering of a ray on a given surface interaction.
    /// Returns the scattered ray and modulated radiance (BSDF value).
    /// A 2D random variable is needed for most surfaces.
    fn bxdfs_at(&self, isect: &Interaction) -> Vec<Box<dyn BxDF>>;

    fn emission(&self) -> Color {
        Color::black()
    }
    fn summary(&self) -> String;
}

#[derive(Clone)]
pub struct Lambertian {
    pub albedo: Arc<dyn Texture>,
}

impl Lambertian {
    pub fn textured(albedo: Arc<dyn Texture>) -> Self {
        Lambertian { albedo }
    }

    pub fn solid(c: Color) -> Self {
        let albedo = Arc::new(texture::Solid::new(c));
        Self::textured(albedo)
    }
}
pub struct Metal {
    pub albedo: Color,
    pub fuzziness: f32,
}
impl Metal {
    pub fn new(albedo: Color, fuzziness: f32) -> Self {
        Self { albedo, fuzziness }
    }
}

pub struct Glossy {
    mf_refl: bxdf::MicrofacetReflection,
}

impl Glossy {
    pub fn new(albedo: Color, roughness: f32) -> Self {
        let alpha = geometry::microfacet::MicrofacetDistrib::roughness_to_alpha(roughness);
        let distrib = geometry::microfacet::MicrofacetDistrib::trowbridge_reitz(alpha, alpha);
        let mf_refl = bxdf::MicrofacetReflection::new(albedo, distrib, bxdf::Fresnel::Nop);
        Self { mf_refl }
    }
}

pub struct Mirror {
    pub albedo: Color,
}
impl Mirror {
    pub fn new(albedo: Color) -> Self {
        Self { albedo }
    }
}

pub struct Plastic {
    pub diffuse: Color,
    pub specular: Color,
    pub roughness: f32,
    pub remap_roughness: bool,
}

pub struct Dielectric {
    pub refract_index: f32,
    pub reflect: Color,
    pub transmit: Color,
}
impl Dielectric {
    pub fn new(refract_index: f32) -> Self {
        Self {
            refract_index,
            reflect: Color::white(),
            transmit: Color::white(),
        }
    }
    pub fn with_colors(self, reflect: Color, transmit: Color) -> Self {
        Self {
            refract_index: self.refract_index,
            reflect,
            transmit,
        }
    }
}

pub enum Roughness {
    Iso(f32),
    UV((f32, f32)),
}

pub struct Uber {
    pub kd: Arc<dyn Texture>,
    pub ks: Arc<dyn Texture>,
    pub kr: Option<Arc<dyn Texture>>,
    pub kt: Option<Arc<dyn Texture>>,
    pub rough: Roughness,
    pub eta: f32,
    pub opacity: f32,
    pub remap_roughness: bool,
}

pub struct Substrate {
    pub kd: Arc<dyn Texture>,
    pub ks: Arc<dyn Texture>,
    pub rough: Roughness,
    pub remap_roughness: bool,
}

pub struct DiffuseLight {
    emit: Color,
}

impl DiffuseLight {
    pub fn new(emit: Color) -> Self {
        DiffuseLight { emit }
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

// Implements the Material trait for various materials.
// ------------------------------------------------------------------------------------------------

impl Material for Lambertian {
    fn scatter(&self, _wo: Vec3, isect: &Interaction) -> (Ray, Color) {
        // let rnd2 = (rand::random::<f32>(), rand::random::<f32>());
        // let (r, f, pr) = self.sample_bsdf(wo, isect, rnd2);
        // assert!(matches!(pr, Prob::Density(_)));
        // assert!(pr.density() > 0.0);
        // return (r, f * r.dir.dot(isect.normal) / pr.density());
        //
        let h = uniform_hemisphere();
        let wo = (h.dot(isect.normal)).signum() * h;
        // let mut wo = isect.normal + uniform_sphere();
        // if wo.norm_squared() < 1e-6 {
        //     wo = isect.normal;
        // }
        let ray_out = isect.spawn_ray(wo);
        (ray_out, self.albedo.value(isect.uv, isect.pos))
    }

    fn bxdfs_at(&self, isect: &Interaction) -> Vec<Box<dyn BxDF>> {
        let point_albedo = self.albedo.value(isect.uv, isect.pos);
        let lambertian = bxdf::DiffuseReflect::lambertian(point_albedo);
        vec![Box::new(lambertian)]
    }

    fn summary(&self) -> String {
        String::from("Lambertian")
    }
}

impl Material for Metal {
    fn scatter(&self, wi: Vec3, isect: &Interaction) -> (Ray, Color) {
        let reflected = hcm::reflect(isect.normal, wi);
        let ray_out = Ray::new(isect.pos, reflected + uniform_sphere() * self.fuzziness);
        (ray_out, self.albedo)
    }
    fn summary(&self) -> String {
        format!("Metal{{albedo = {}}}", self.albedo)
    }

    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<Box<dyn BxDF>> {
        todo!()
    }
}

impl Material for Glossy {
    fn scatter(&self, _wi: Vec3, _isect: &Interaction) -> (Ray, Color) {
        todo!()
    }
    fn summary(&self) -> String {
        "Glossy".to_owned()
    }

    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<Box<dyn BxDF>> {
        todo!()
    }
}
impl Material for Mirror {
    fn scatter(&self, wi: Vec3, isect: &Interaction) -> (Ray, Color) {
        let reflected = hcm::reflect(isect.normal, wi);
        let ray_out = Ray::new(isect.pos, reflected);
        (ray_out, self.albedo)
    }
    fn summary(&self) -> String {
        format!("Mirror{{albedo = {}}}", self.albedo)
    }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<Box<dyn BxDF>> {
        let spec = bxdf::Specular::mirror(self.albedo);
        vec![Box::new(spec)]
    }

    // fn sample_bsdf(&self, wo: Vec3, isect: &Interaction, rnd2: (f32, f32)) -> (Ray, Color, Prob) {
    //     let spec = bxdf::Specular::mirror(self.albedo);
    //     let (f, wi, pr) = spec.sample(isect.world_to_local(wo), rnd2);
    //     let ray_out = isect.spawn_ray(isect.local_to_world(wi));
    //     (ray_out, f, pr)
    // }
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
        let (mut wo, reflect_pr, mut color) = match hcm::refract(outward_normal, wi, ni_over_no) {
            hcm::Transmit(wt) => (wt, schlick(cosine, self.refract_index), self.transmit),
            hcm::FullReflect(_) => (reflected, 1.0, self.reflect),
        };
        if rand::random::<f32>() < reflect_pr {
            wo = reflected;
            color = self.reflect;
        }
        (Ray::new(isect.pos, wo), color)
    }
    fn summary(&self) -> String {
        format!("Dielectric{{ior = {}}}", self.refract_index)
    }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<Box<dyn BxDF>> {
        let spec = bxdf::Specular::dielectric(self.reflect, 1.0, self.refract_index);
        vec![Box::new(spec)]
    }

    // fn sample_bsdf(&self, wo: Vec3, isect: &Interaction, rnd2: (f32, f32)) -> (Ray, Color, Prob) {
    //     let bsdf = bxdf::Specular::dielectric(self.reflect, 1.0, self.refract_index);
    //     let (f, wi, pr) = bsdf.sample(isect.world_to_local(wo), rnd2);
    //     let ray_out = isect.spawn_ray(isect.local_to_world(wi));
    //     (ray_out, f, pr)
    // }
}

impl Material for DiffuseLight {
    fn scatter(&self, wi: Vec3, isect: &Interaction) -> (Ray, Color) {
        let wo = hcm::reflect(isect.normal, wi);
        let pos = isect.normal * 0.0001 + isect.pos;
        (Ray::new(pos, wo), Color::black())
    }
    fn emission(&self) -> Color {
        self.emit
    }
    fn summary(&self) -> String {
        format!("DiffuseLight{{emit = {}}}", self.emit)
    }
    // fn sample_bsdf(&self, wo: Vec3, isect: &Interaction, _rnd2: (f32, f32)) -> (Ray, Color, Prob) {
    //     let (r, color) = self.scatter(wo, isect);
    //     (r, color, Prob::Density(std::f32::consts::FRAC_1_PI))
    // }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<Box<dyn BxDF>> {
        todo!()
    }
}

impl Material for Uber {
    fn scatter(&self, _wi: Vec3, _isect: &Interaction) -> (Ray, Color) {
        todo!()
    }
    fn summary(&self) -> String {
        String::from("uber")
    }

    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<Box<dyn BxDF>> {
        todo!()
    }
}

impl Material for Substrate {
    fn scatter(&self, _wi: Vec3, _isect: &Interaction) -> (Ray, Color) {
        todo!()
    }
    fn summary(&self) -> String {
        String::from("substrate")
    }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<Box<dyn BxDF>> {
        todo!()
    }
}

impl Material for Plastic {
    fn scatter(&self, _wi: Vec3, isect: &Interaction) -> (Ray, Color) {
        let h = uniform_hemisphere();
        let wo = (h.dot(isect.normal)).signum() * h;
        let ray_out = isect.spawn_ray(wo);
        (ray_out, self.diffuse)
    }
    fn summary(&self) -> String {
        String::from("plastic")
    }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<Box<dyn BxDF>> {
        use microfacet::MicrofacetDistrib as MFDistrib; 
        let lambertian = bxdf::DiffuseReflect::lambertian(self.diffuse);
        let alpha = MFDistrib::roughness_to_alpha(self.roughness);
        let trowbridge = MFDistrib::trowbridge_reitz(alpha, alpha);
        let mf_refl =
            bxdf::MicrofacetReflection::new(self.specular, trowbridge, bxdf::Fresnel::Nop);
        vec![Box::new(mf_refl), Box::new(lambertian)]
    }
}

fn schlick(cosine: f32, ref_index: f32) -> f32 {
    let r0 = (1.0 - ref_index) / (1.0 + ref_index);
    let r0 = r0 * r0;
    r0 + (1.0 - r0) * (1.0 - cosine).powi(5)
}

#[cfg(test)]
mod test {
    use math::hcm::{Point3, Vec3};
    use radiometry::color::Color;
    use shape::Interaction;

    use crate::material::{Lambertian, Material};

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

    #[test]
    fn test_lambertian() {
        let isect = Interaction::rayless(
            Point3::new(0.4, 0.5, 3.0),
            (0.3, 0.8),
            Vec3::new(0.36, 0.48, 0.8),
        )
        .with_dpdu(Vec3::new(-0.8, 0.6, 0.0));
        assert!(isect.has_valid_frame());
        let lambertian = Lambertian::solid(Color::white());
        let wo = Vec3::new(0.7, 0.5, 0.3);
        let (_ray, color) = lambertian.scatter(wo, &isect);
        // let (ray, f, p) = lambertian.sample_bsdf(wo, &isect, (0.8, 0.5));

        // eprintln!(
        //     "Scattered color = {}, BSDF value = {}, prob = {:?}",
        //     color, f, p
        // );
        // let v0 = Vec3::new(color.r, color.g, color.b);
        // let color1 = f * ray.dir.dot(isect.normal) / p.density();
        // let v1 = Vec3::new(color1.r, color1.g, color1.b);
        // assert!(
        //     (v0 - v1).norm_squared().abs() < 1e-5,
        //     "{:.5} vs {:.5}",
        //     v0,
        //     v1
        // )
    }
}
