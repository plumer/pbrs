use geometry::bxdf::{self, Fresnel, BXDF};
use geometry::fourier::{FourierBSDF, FourierTable};
use geometry::microfacet::MicrofacetDistrib;
use geometry::Interaction;
use geometry::{microfacet, ray::Ray};
use math::hcm::{self, Vec3};
use radiometry::color::Color;
use std::sync::Arc;
use texture::{self, *};

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
    fn bxdfs_at<'a>(&'a self, isect: &Interaction) -> Vec<BXDF<'a>>;

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
    // pub albedo: Color,
    eta_real: Color,
    eta_imag: Color,
    pub fuzziness: f32,
}
impl Metal {
    pub fn new(_albedo: Color, _fuzziness: f32) -> Self {
        panic!("Can't build from albedo. use from_ior instead")
        // Self { albedo, fuzziness }
    }
    /// Builds a metal material from index of refraction and fuzziness.
    /// Index of refraction is a per-channel complex number, expressed as η + ki.
    pub fn from_ior(eta: Color, eta_k: Color, fuzziness: f32) -> Self {
        Self {
            eta_real: eta,
            eta_imag: eta_k,
            fuzziness,
        }
    }
}

pub struct Glossy {
    mf_refl: bxdf::MicrofacetReflection,
}

impl Glossy {
    pub fn new(albedo: Color, roughness: f32) -> Self {
        let alpha = geometry::microfacet::MicrofacetDistrib::roughness_to_alpha(roughness);
        let distrib = geometry::microfacet::MicrofacetDistrib::beckmann(alpha, alpha);
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

    fn bxdfs_at(&self, isect: &Interaction) -> Vec<BXDF> {
        let point_albedo = self.albedo.value(isect.uv, isect.pos);
        let lambertian = bxdf::DiffuseReflect::lambertian(point_albedo);
        vec![lambertian.into()]
    }

    fn summary(&self) -> String {
        String::from("Lambertian")
    }
}

impl Material for Metal {
    fn scatter(&self, wi: Vec3, isect: &Interaction) -> (Ray, Color) {
        let reflected = hcm::reflect(isect.normal, wi);
        let ray_out = Ray::new(isect.pos, reflected + uniform_sphere() * self.fuzziness);
        (
            ray_out,
            Fresnel::conductor(self.eta_real, self.eta_imag).eval(isect.normal.dot(wi).abs()),
        )
    }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<BXDF> {
        let albedo = Color::white();
        let alpha = MicrofacetDistrib::roughness_to_alpha(self.fuzziness);
        let distrib = MicrofacetDistrib::beckmann(alpha, alpha);
        let fresnel = Fresnel::conductor(self.eta_real, self.eta_imag);
        vec![bxdf::MicrofacetReflection::new(albedo, distrib, fresnel).into()]
    }
    fn summary(&self) -> String {
        format!("Metal{{ior = {} + {}i}}", self.eta_real, self.eta_imag)
    }
}

impl Material for Glossy {
    fn scatter(&self, _wi: Vec3, _isect: &Interaction) -> (Ray, Color) {
        todo!()
    }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<BXDF> {
        vec![self.mf_refl.clone().into()]
    }
    fn summary(&self) -> String {
        "Glossy".to_owned()
    }
}
impl Material for Mirror {
    fn scatter(&self, wi: Vec3, isect: &Interaction) -> (Ray, Color) {
        let reflected = hcm::reflect(isect.normal, wi);
        let ray_out = Ray::new(isect.pos, reflected);
        (ray_out, self.albedo)
    }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<BXDF> {
        let spec = bxdf::Specular::mirror(self.albedo);
        vec![spec.into()]
    }
    fn summary(&self) -> String {
        format!("Mirror{{albedo = {}}}", self.albedo)
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
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<BXDF> {
        let spec = bxdf::Specular::dielectric(self.reflect, 1.0, self.refract_index);
        vec![spec.into()]
    }
    fn summary(&self) -> String {
        format!("Dielectric{{ior = {}}}", self.refract_index)
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
    // fn sample_bsdf(&self, wo: Vec3, isect: &Interaction, _rnd2: (f32, f32)) -> (Ray, Color, Prob) {
    //     let (r, color) = self.scatter(wo, isect);
    //     (r, color, Prob::Density(std::f32::consts::FRAC_1_PI))
    // }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<BXDF> {
        vec![]
    }
    fn emission(&self) -> Color {
        self.emit
    }
    fn summary(&self) -> String {
        format!("DiffuseLight{{emit = {}}}", self.emit)
    }
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

impl Material for Uber {
    fn scatter(&self, _wi: Vec3, _isect: &Interaction) -> (Ray, Color) {
        todo!()
    }
    fn bxdfs_at(&self, isect: &Interaction) -> Vec<BXDF> {
        let mut bxdfs: Vec<BXDF> = vec![];
        let transmission = Color::gray((1.0 - self.opacity).clamp(0.0, 1.0));
        if transmission.is_black() {
        } else {
            let spec_transmit = bxdf::Specular::transmit(transmission, 1.0, self.eta);
            bxdfs.push(spec_transmit.into());
        }

        let kd = self.kd.value(isect.uv, isect.pos);
        if !kd.is_black() {
            let diffuse = bxdf::DiffuseReflect::lambertian(kd);
            bxdfs.push(diffuse.into());
        }
        let ks = self.ks.value(isect.uv, isect.pos);
        if !ks.is_black() {
            let fresnel = Fresnel::dielectric(1.0, self.eta);
            let (ru, rv) = match self.rough {
                Roughness::Iso(u) => (u, u),
                Roughness::UV(uv) => uv,
            };
            let (au, av) = match self.remap_roughness {
                true => (
                    MicrofacetDistrib::roughness_to_alpha(ru),
                    MicrofacetDistrib::roughness_to_alpha(rv),
                ),
                false => (ru, rv),
            };
            let distrib = MicrofacetDistrib::beckmann(au, av);
            let spec = bxdf::MicrofacetReflection::new(ks, distrib, fresnel);
            bxdfs.push(spec.into());
        }

        if let Some(kr_tex) = &self.kr {
            let kr = kr_tex.value(isect.uv, isect.pos);
            if !kr.is_black() {
                let refl = bxdf::Specular::dielectric(kr, 1.0, self.eta);
                bxdfs.push(refl.into());
            }
        }
        if let Some(kt_tex) = &self.kt {
            let kt = kt_tex.value(isect.uv, isect.pos);
            if !kt.is_black() {
                let trans = bxdf::Specular::transmit(kt, 1.0, self.eta);
                bxdfs.push(trans.into());
            }
        }
        bxdfs
    }
    fn summary(&self) -> String {
        String::from("uber")
    }
}

pub struct Substrate {
    pub kd: Arc<dyn Texture>,
    pub ks: Arc<dyn Texture>,
    pub alpha: f32,
}

impl Substrate {
    pub fn new(
        kd: Arc<dyn Texture>, ks: Arc<dyn Texture>, roughness: f32, remap_roughness: bool,
    ) -> Self {
        let alpha = match remap_roughness {
            true => MicrofacetDistrib::roughness_to_alpha(roughness),
            false => roughness,
        };
        Self { kd, ks, alpha }
    }
}

impl Material for Substrate {
    fn scatter(&self, _wi: Vec3, _isect: &Interaction) -> (Ray, Color) {
        todo!()
    }
    fn bxdfs_at(&self, isect: &Interaction) -> Vec<BXDF> {
        /*
        if (bumpMap) Bump(bumpMap, si);
        si->bsdf = ARENA_ALLOC(arena, BSDF)(*si);
        Spectrum d = Kd->Evaluate(*si).Clamp();
        Spectrum s = Ks->Evaluate(*si).Clamp();
        Float roughu = nu->Evaluate(*si);
        Float roughv = nv->Evaluate(*si);

        if (!d.IsBlack() || !s.IsBlack()) {
            if (remapRoughness) {
                roughu = TrowbridgeReitzDistribution::RoughnessToAlpha(roughu);
                roughv = TrowbridgeReitzDistribution::RoughnessToAlpha(roughv);
            }
            MicrofacetDistribution *distrib =
                ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(roughu, roughv);
            si->bsdf->Add(ARENA_ALLOC(arena, FresnelBlend)(d, s, distrib));
        } */
        let diffuse = self.kd.value(isect.uv, isect.pos);
        let specular = self.ks.value(isect.uv, isect.pos);
        if diffuse.is_black() && specular.is_black() {
            vec![]
        } else {
            // let distrib = MicrofacetDistrib::beckmann(self.alpha, self.alpha);
            // vec![bxdf::FresnelBlend::new(diffuse, specular, distrib).into()]
            vec![bxdf::DiffuseReflect::lambertian(diffuse).into()]
        }
    }
    fn summary(&self) -> String {
        String::from("substrate")
    }
}

impl Material for Plastic {
    fn scatter(&self, _wi: Vec3, isect: &Interaction) -> (Ray, Color) {
        let h = uniform_hemisphere();
        let wo = (h.dot(isect.normal)).signum() * h;
        let ray_out = isect.spawn_ray(wo);
        (ray_out, self.diffuse)
    }
    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<BXDF> {
        use microfacet::MicrofacetDistrib as MFDistrib;
        let lambertian = bxdf::DiffuseReflect::lambertian(self.diffuse);
        let alpha = MFDistrib::roughness_to_alpha(self.roughness);
        let trowbridge = MFDistrib::beckmann(alpha, alpha);
        let mf_refl =
            bxdf::MicrofacetReflection::new(self.specular, trowbridge, bxdf::Fresnel::Nop);
        vec![mf_refl.into(), lambertian.into()]
    }
    fn summary(&self) -> String {
        String::from("plastic")
    }
}

pub struct Fourier {
    table: FourierTable,
}

impl Fourier {
    pub fn from_file(path: &str) -> Self {
        let table = FourierTable::from_file(path).unwrap();
        Self { table }
    }
}

impl<'a> crate::Material for Fourier {
    fn scatter(&self, _wi: math::hcm::Vec3, _isect: &Interaction) -> (geometry::ray::Ray, Color) {
        todo!()
    }

    fn bxdfs_at(&self, _isect: &Interaction) -> Vec<BXDF> {
        let fourier_bsdf = FourierBSDF { table: &self.table };
        vec![fourier_bsdf.into()]
    }

    fn summary(&self) -> String {
        "Fourier".to_owned()
    }
}

fn schlick(cosine: f32, ref_index: f32) -> f32 {
    let r0 = (1.0 - ref_index) / (1.0 + ref_index);
    let r0 = r0 * r0;
    r0 + (1.0 - r0) * (1.0 - cosine).powi(5)
}

#[cfg(test)]
mod test {
    use geometry::bxdf::{BxDF, Omega};
    use geometry::Interaction;
    use math::{
        float::Float,
        hcm::{Point3, Vec3},
    };
    use radiometry::color::Color;

    use super::{Lambertian, Material};

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
        let v0 = Vec3::new(color.r, color.g, color.b);
        let v1 = {
            let lambertian_bxdf = lambertian
                .bxdfs_at(&isect)
                .pop()
                .expect("At least one bxdf");
            let (f, wi, p) = lambertian_bxdf.sample(Omega(wo.hat()), (0.8, 0.5));

            eprintln!(
                "Scattered color = {}, BSDF value = {}, prob = {:?}",
                color, f, p
            );
            let color1 = f * wi.cos_theta().abs() * p.density().weak_recip();
            Vec3::new(color1.r, color1.g, color1.b)
        };

        assert!(
            (v0 - v1).norm_squared().abs() < 1e-5,
            "{:.5} vs {:.5}",
            v0,
            v1
        )
    }
}
