use math::float::Float;
use math::hcm::Vec3;

type Color = Vec3;

pub enum HemiPdf {
    Regular(f32),
    Delta(f32),
}

pub enum SmoothnessType {
    Diffuse,
    Glossy,
    Specular,
}

pub enum IntrusionType {
    Reflection,
    Transmission,
    ReflectTransmit,
}
pub struct BxDFType {
    intrusion: IntrusionType,
    smooth: SmoothnessType,
}

pub mod local {
    use super::Vec3;
    pub fn cos_theta(w: Vec3) -> f32 {
        w.z
    }
    pub fn cos_theta_sqr(w: Vec3) -> f32 {
        w.z.powi(2)
    }
    pub fn sin_theta(w: Vec3) -> f32 {
        sin_theta_sqr(w).max(0.0).sqrt()
    }
    pub fn sin_theta_sqr(w: Vec3) -> f32 {
        1.0 - cos_theta_sqr(w)
    }

    pub fn cos_phi(w: Vec3) -> f32 {
        let sin_theta = sin_theta(w);
        if sin_theta == 0.0 {
            1.0
        } else {
            (w.x / sin_theta).clamp(-1.0, 1.0)
        }
    }
    pub fn sin_phi(w: Vec3) -> f32 {
        let sin_theta = sin_theta(w);
        if sin_theta == 0.0 {
            1.0
        } else {
            (w.y / sin_theta).clamp(-1.0, 1.0)
        }
    }
}

pub fn concentric_sample_disk(uv: (f32, f32)) -> (f32, f32) {
    let x = uv.0 * 2.0 - 1.0;
    let y = uv.1 * 2.0 - 1.0;

    if x == 0.0 && y == 0.0 {
        return (0.0, 0.0);
    }
    let r = if x.abs() > y.abs() { x } else { y }.abs();
    assert_eq!(r, x.abs().max(y.abs()));
    let hypot = x.hypot(y);
    let (cos_theta, sin_theta) = (x / hypot, y / hypot);
    // println!("r = {}, hypot = {}, sin/cos = {}/{}", r, hypot, sin_theta, cos_theta);
    (r * cos_theta, r * sin_theta)
}

pub fn cos_sample_hemisphere(uv: (f32, f32)) -> Vec3 {
    let (x, y) = concentric_sample_disk(uv);
    let z = (1.0 - x * x - y * y).max(0.0).sqrt();
    Vec3::new(x, y, z)
}

pub fn cos_hemisphere_pdf(w_local: Vec3) -> f32 {
    local::cos_theta(w_local) * std::f32::consts::FRAC_1_PI
}

/// Describes the ray scattering behaviors in a probabilistic way. All vectors involved are in the
/// intersection coordinate system: +Z being the normal vector. To interact with the models, one
/// should convert the outgoing/incident directions to local coordinates.
///
/// BSDF plays a key role in the rendering integration:
/// ```
///             /
/// L_o - L_e = | L(wi) * f(wo, wi) * abscos(wi) d(wi)
///            / Sphere
/// ```
/// By Monte-Carlo integration, the integral is estimated as the expectation of the following:
/// ```
/// L_o - L_e = L(wi) * f(wo, wi) * abscos(wi) / pdf(wi)
/// ```
/// where `wi` is a randomly generated unit-length 3D vector, denoting the incident direction.
/// The `sample` method should return the sampled incident direction `wi`, the evaluated bsdf `f`,
/// and the pdf.
///
/// Method `eval(wo, wi)` returns the value of the BSDF function at given in-out angles.
///
/// Method `sample(wo, (u, v))` produces necessary values for one sample of contribution to the
/// monte-carlo integration of the rendering equation.
///
/// Useful implementations:
/// - [`SpecularReflection`], [`SpecularTransmission`], `FresnelSpecular`
pub trait BxDF {
    const FRAC_1_PI: f32 = std::f32::consts::FRAC_1_PI;
    fn get_type(&self) -> BxDFType;

    /// Evaluates the BSDF function at given in-out angles. Note that specular BSDFs always return
    /// 0. Use [`sample()`] in those cases instead.
    fn eval(&self, wo_local: Vec3, wi_local: Vec3) -> Color;

    /// Produces a possible incident direction given the outgoing direction, returning the
    /// probability density of the resulting direction and consumes a 2D random variable
    /// if the behavior is probabilistic (e.g., lambertian), and the BSDF value.
    ///
    /// Returned values from one such invocation produces values needed for one sample of
    /// contribution to the monte-carlo integration process.
    fn sample(&self, wo_local: Vec3, rnd2: (f32, f32)) -> (Vec3, HemiPdf, Color);

    fn pdf(&self, wo_local: Vec3, wi_local: Vec3) -> f32;
}

// ----------------------------

pub trait Fresnel {
    /// Computes the ratio of refracted energy.
    fn refl_coeff(&self, cos_theta_i: f32) -> f32;
}

/// Fresnel reflectivity ratio computation for dielectric materials (e.g., glass).
pub struct FresnelDielectric {
    eta_i: f32,
    eta_t: f32,
}

impl FresnelDielectric {
    pub fn new(eta_i: f32, eta_t: f32) -> Self {
        Self { eta_i, eta_t }
    }
}

impl Fresnel for FresnelDielectric {
    /// Computes ratio of reflected radiance from the surface scattering.
    /// If `cos_theta_i` is negative, then the refractive index will be inverted before calculation.
    fn refl_coeff(&self, cos_theta_i: f32) -> f32 {
        let cos_theta_i = cos_theta_i.clamp(-1.0, 1.0);
        let (eta_i, eta_t, cos_theta_i) = if cos_theta_i > 0.0 {
            (self.eta_i, self.eta_t, cos_theta_i)
        } else {
            (self.eta_t, self.eta_i, -cos_theta_i)
        };

        let sin_theta_i = 1.0.cathetus(cos_theta_i); // (1.0 - cos_theta_i.powi(2)).max(0.0).sqrt();
        let sin_theta_t = eta_i / eta_t * sin_theta_i;
        let ratio = if sin_theta_t >= 1.0 {
            1.0
        } else {
            let cos_theta_t = (1.0 - sin_theta_t.powi(2)).max(0.0).sqrt();
            // perp  /n_i cos_i - n_t cos_t\ 2  parl  /n_i cos_t - n_t cos_i\ 2
            // R_s = |---------------------|    R_p = |---------------------|
            //       \n_i cos_i + n_t cos_t/          \n_i cos_t + n_t cos_i/
            let r_perpendicular = (eta_i * cos_theta_i - eta_t * cos_theta_t)
                / (eta_i * cos_theta_i + eta_t * cos_theta_t);
            let r_parallel = (eta_t * cos_theta_i - eta_i * cos_theta_t)
                / (eta_t * cos_theta_i + eta_i * cos_theta_t);
            (r_parallel.powi(2) + r_perpendicular.powi(2)) * 0.5
        };
        ratio
    }
}

// ---------------------------

/// BSDF that represents specular reflection. All transmission energy is absorbed.
///
/// `eval()` always return black - the reflection is specular. `sample()` is more useful.
///
/// [`sample()`] returns perfect reflected direction across
/// the normal (z-axis in the local coordinate), and uses the underlying Fresnel model to determine
/// reflectivity modulation on the BSDF value.
pub struct SpecularReflection<Fr: Fresnel> {
    albedo: Color,
    fresnel: Fr,
}

impl<Fr: Fresnel> SpecularReflection<Fr> {
    pub fn new(albedo: Color, fresnel: Fr) -> Self {
        Self { albedo, fresnel }
    }
}

impl<Fr: Fresnel> BxDF for SpecularReflection<Fr> {
    fn get_type(&self) -> BxDFType {
        BxDFType {
            intrusion: IntrusionType::Reflection,
            smooth: SmoothnessType::Specular,
        }
    }

    fn eval(&self, _wo_local: Vec3, _wi_local: Vec3) -> Color {
        Color::new(0.0, 0.0, 0.0)
    }

    fn sample(&self, wo_local: Vec3, _rnd2: (f32, f32)) -> (Vec3, HemiPdf, Color) {
        let wi_local = Vec3::new(-wo_local.x, -wo_local.y, wo_local.z);
        (
            wi_local,
            HemiPdf::Delta(1.0),
            self.fresnel.refl_coeff(local::cos_theta(wi_local)) * self.albedo
                / local::cos_theta(wi_local).abs(),
        )
    }

    fn pdf(&self, wo_local: Vec3, wi_local: Vec3) -> f32 {
        0.0
    }
}

/// BSDF that represents specular transmission. All reflective energy is absorbed, which is uncommon
/// in real-life.
///
/// `eval()` always return black - the transmission is specular. `sample()` is more useful.
///
/// `sample()` computes the refracted ray using the normal (z-axis in the local coordinate). The
/// amount of energy that gets through is computed by the underlying Fresnel model. In the case of a
/// full reflection, zero vector is returned and the BSDF value is black as well.
pub struct SpecularTransmission {
    albedo: Color,
    eta_outer: f32,
    eta_inner: f32,
    fresnel: FresnelDielectric,
}

impl SpecularTransmission {
    /// Makes a new specular transmission BSDF with given
    #[rustfmt::skip]
    pub fn new(albedo: Color, eta_outer: f32, eta_inner: f32) -> Self {
        Self {
            albedo, eta_outer, eta_inner,
            fresnel: FresnelDielectric::new(eta_outer, eta_inner),
        }
    }
}

impl BxDF for SpecularTransmission {
    fn get_type(&self) -> BxDFType {
        BxDFType {
            intrusion: IntrusionType::Transmission,
            smooth: SmoothnessType::Specular,
        }
    }

    fn eval(&self, _wo_local: Vec3, _wi_local: Vec3) -> Color {
        Color::new(0.0, 0.0, 0.0)
    }

    fn sample(&self, wo_local: Vec3, _rnd2: (f32, f32)) -> (Vec3, HemiPdf, Color) {
        // Computes the normal for computing the refraction. It is flipped to the side forming an
        // acute angle with `wo_local`.
        let (eta_i, eta_t, normal) = if local::cos_theta(wo_local) > 0.0 {
            (self.eta_outer, self.eta_inner, Vec3::zbase())
        } else {
            (self.eta_inner, self.eta_outer, -Vec3::zbase())
        };

        match math::hcm::refract(normal, wo_local, eta_i / eta_t) {
            math::hcm::FullReflect(_) => (Vec3::zero(), HemiPdf::Delta(1.0), Color::zero()),
            math::hcm::Transmit(wi_local) => {
                // Transmissivity = 1 - reflectivity
                let f_tr = 1.0 - self.fresnel.refl_coeff(local::cos_theta(wi_local));
                // if transport_mode is radiance: f_t *= (eta_i / eta_t)^2
                (
                    wi_local,
                    HemiPdf::Delta(1.0),
                    (f_tr / local::cos_theta(wi_local).abs()) * self.albedo,
                )
            }
        }
    }

    fn pdf(&self, wo_local: Vec3, wi_local: Vec3) -> f32 {
        0.0
    }
}

/// BSDF representing dielectric material (e.g., glass). Ray scattering is both reflective and
/// transmissive.
pub struct FresnelSpecular {
    reflect_albedo: Color,
    transmit_albedo: Color,
    eta_a: f32,
    eta_b: f32,
    // TODO(zixun) transport mode?
}

impl FresnelSpecular {
    #[rustfmt::skip]
    /// Makes a new specular dielectric BSDF.
    /// - `reflect_albedo` and `transmit_albedo` is often the same.
    /// - `eta_a` is the IOR of the medium on the positive side of the normal vector, and `eta_b`
    ///   the opposite side.
    fn new(reflect_albedo: Color, transmit_albedo: Color, eta_a: f32, eta_b: f32) -> Self {
        Self{reflect_albedo, transmit_albedo, eta_a, eta_b}
    }
}

impl BxDF for FresnelSpecular {
    fn get_type(&self) -> BxDFType {
        BxDFType {
            intrusion: IntrusionType::ReflectTransmit,
            smooth: SmoothnessType::Specular,
        }
    }

    fn eval(&self, _wo_local: Vec3, _wi_local: Vec3) -> Color {
        Color::new(0.0, 0.0, 0.0)
    }

    fn sample(&self, wo_local: Vec3, rnd2: (f32, f32)) -> (Vec3, HemiPdf, Color) {
        let refl_coeff =
            FresnelDielectric::new(self.eta_a, self.eta_b).refl_coeff(local::cos_theta(wo_local));
        let (u, _v) = rnd2;
        if u < refl_coeff {
            // Samples a reflective direction.
            let wi_local = Vec3::new(-wo_local.x, -wo_local.y, wo_local.z);
            (
                wi_local,
                HemiPdf::Regular(refl_coeff),
                refl_coeff * self.reflect_albedo / local::cos_theta(wi_local).abs(),
            )
        } else {
            // Samples a transmissive direction.
            let (eta_i, eta_t, normal) = if local::cos_theta(wo_local) > 0.0 {
                (self.eta_a, self.eta_b, Vec3::zbase())
            } else {
                (self.eta_b, self.eta_a, -Vec3::zbase())
            };
            match math::hcm::refract(normal, wo_local, eta_i / eta_t) {
                math::hcm::FullReflect(wi_local) => {
                    (wi_local, HemiPdf::Delta(refl_coeff), Color::zero())
                }
                math::hcm::Transmit(wi_local) => {
                    // Transmissivity = 1 - reflectivity
                    let transmit_coeff = 1.0 - refl_coeff;
                    // if transport_mode is radiance: f_t *= (eta_i / eta_t)^2
                    (
                        wi_local,
                        HemiPdf::Delta(transmit_coeff),
                        (transmit_coeff / local::cos_theta(wi_local).abs()) * self.transmit_albedo,
                    )
                }
            }
        }
    }

    fn pdf(&self, wo_local: Vec3, wi_local: Vec3) -> f32 {
        0.0
    }
}

pub struct LambertianReflection {
    albedo: Color,
}

impl LambertianReflection {
    pub fn new(albedo: Color) -> Self {
        Self { albedo }
    }
}

impl BxDF for LambertianReflection {
    fn get_type(&self) -> BxDFType {
        BxDFType {
            intrusion: IntrusionType::Reflection,
            smooth: SmoothnessType::Diffuse,
        }
    }

    fn eval(&self, _wo_local: Vec3, _wi_local: Vec3) -> Color {
        self.albedo * Self::FRAC_1_PI
    }

    fn sample(&self, wo_local: Vec3, rnd2: (f32, f32)) -> (Vec3, HemiPdf, Color) {
        assert!(local::cos_theta(wo_local) >= 0.0);
        let wi_local = cos_sample_hemisphere(rnd2);
        (
            wi_local,
            HemiPdf::Regular(self.pdf(wo_local, wi_local)),
            self.eval(wo_local, wi_local),
        )
    }

    fn pdf(&self, wo_local: Vec3, wi_local: Vec3) -> f32 {
        assert!(wo_local.z * wi_local.z >= 0.0);
        local::cos_theta(wi_local) * Self::FRAC_1_PI
    }
}
