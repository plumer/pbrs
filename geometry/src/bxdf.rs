use crate::microfacet as mf;
use math::hcm::Vec3;
use radiometry::color::Color;

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

#[allow(dead_code)]
pub struct BxDFType {
    intrusion: IntrusionType,
    smooth: SmoothnessType,
}

pub mod local {
    use super::Vec3;
    pub fn cos_theta(w: Vec3) -> f32 {
        w.z
    }
    pub fn cos2_theta(w: Vec3) -> f32 {
        w.z.powi(2)
    }
    pub fn sin_theta(w: Vec3) -> f32 {
        sin2_theta(w).max(0.0).sqrt()
    }
    pub fn sin2_theta(w: Vec3) -> f32 {
        1.0 - cos2_theta(w)
    }
    pub fn tan2_theta(w: Vec3) -> f32 {
        sin2_theta(w) / cos2_theta(w)
    }

    pub fn cos_phi(w: Vec3) -> f32 {
        let xy_hypot = w.x.hypot(w.y);
        if xy_hypot == 0.0 {
            1.0
        } else {
            w.x / xy_hypot
        }
    }
    pub fn sin_phi(w: Vec3) -> f32 {
        let xy_hypot = w.x.hypot(w.y);
        if xy_hypot == 0.0 {
            0.0
        } else {
            w.y / xy_hypot
        }
    }
    pub fn cos2_phi(w: Vec3) -> f32 {
        let xy_length2 = w.x * w.x + w.y * w.y;
        if xy_length2 == 0.0 {
            1.0
        } else {
            w.x * w.x / xy_length2
        }
    }
    pub fn sin2_phi(w: Vec3) -> f32 {
        let xy_length2 = w.x * w.x + w.y * w.y;
        if xy_length2 == 0.0 {
            0.0
        } else {
            w.y * w.y / xy_length2
        }
    }

    pub fn sin_cos_phi(w: Vec3) -> (f32, f32) {
        let xy_hypot = w.x.hypot(w.y);
        if xy_hypot == 0.0 {
            (0.0, 1.0)
        } else {
            (w.x / xy_hypot, w.y / xy_hypot)
        }
    }

    pub fn same_hemisphere(w0: Vec3, w1: Vec3) -> bool {
        w0.z * w1.z >= 0.0
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
/// ```ignore
///             /
/// L_o - L_e = | L(wi) * f(wo, wi) * abscos(wi) d(wi)
///            / Sphere
/// ```
/// By Monte-Carlo integration, the integral is estimated as the expectation of the following:
/// ```ignore
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

    /// A default implementation of the `sample` method. Uses cosine-hemisphere distribution
    /// to map the 2D random variable, and uses `pdf` and `eval` to compute the return values.
    fn cosine_hemisphere_sample(&self, wo_local: Vec3, rnd2: (f32, f32)) -> (Vec3, HemiPdf, Color) {
        assert!(local::cos_theta(wo_local) >= 0.0);
        let wi_local = cos_sample_hemisphere(rnd2);
        (
            wi_local,
            HemiPdf::Regular(self.pdf(wo_local, wi_local)),
            self.eval(wo_local, wi_local),
        )
    }
}

// ----------------------------
#[derive(Debug, Clone, Copy)]
#[rustfmt::skip]
pub enum Fresnel {
    Nop,
    /// Fresnel reflectivity ratio computation for dielectric materials (e.g., glass).
    Dielectric { eta_i: f32, eta_t: f32 },
    // TODO(zixun): Conductor(f32, f32)
}

impl Fresnel {
    pub fn dielectric(eta_i: f32, eta_t: f32) -> Self {
        Self::Dielectric { eta_i, eta_t }
    }

    /// Computes the ratio of refracted energy.
    pub fn refl_coeff(&self, cos_theta_i: f32) -> f32 {
        match self {
            Self::Nop => 1.0,
            Self::Dielectric { eta_i, eta_t } => {
                let cos_theta_i = cos_theta_i.clamp(-1.0, 1.0);
                let (eta_i, eta_t, cos_theta_i) = if cos_theta_i > 0.0 {
                    (eta_i, eta_t, cos_theta_i)
                } else {
                    (eta_t, eta_i, -cos_theta_i)
                };

                let sin_theta_i = (1.0 - cos_theta_i.powi(2)).max(0.0).sqrt();
                let sin_theta_t = eta_i / eta_t * sin_theta_i;
                let ratio = if sin_theta_t >= 1.0 {
                    1.0
                } else {
                    let cos_theta_t = (1.0 - sin_theta_t.powi(2)).max(0.0).sqrt();
                    //       perpendicular           2        parallel                2
                    //       /n_i cos_i - n_t cos_t\          /n_i cos_t - n_t cos_i\
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
pub struct SpecularReflection {
    albedo: Color,
    fresnel: Fresnel,
}

impl SpecularReflection {
    pub fn dielectric(albedo: Color, eta_i: f32, eta_t: f32) -> Self {
        Self {
            albedo,
            fresnel: Fresnel::Dielectric { eta_i, eta_t },
        }
    }
}

impl BxDF for SpecularReflection {
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

    fn pdf(&self, _wo_local: Vec3, _wi_local: Vec3) -> f32 {
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
    fresnel: Fresnel,
}

impl SpecularTransmission {
    /// Makes a new specular transmission BSDF with given
    #[rustfmt::skip]
    pub fn new(albedo: Color, eta_outer: f32, eta_inner: f32) -> Self {
        Self {
            albedo, eta_outer, eta_inner,
            fresnel: Fresnel::dielectric(eta_outer, eta_inner),
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
            math::hcm::FullReflect(_) => (Vec3::zero(), HemiPdf::Delta(1.0), Color::black()),
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

    fn pdf(&self, _wo_local: Vec3, _wi_local: Vec3) -> f32 {
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
    #[allow(dead_code)]
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
            Fresnel::dielectric(self.eta_a, self.eta_b).refl_coeff(local::cos_theta(wo_local));
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
                    (wi_local, HemiPdf::Delta(refl_coeff), Color::black())
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

    fn pdf(&self, _wo_local: Vec3, _wi_local: Vec3) -> f32 {
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
        self.cosine_hemisphere_sample(wo_local, rnd2)
    }

    fn pdf(&self, wo_local: Vec3, wi_local: Vec3) -> f32 {
        assert!(wo_local.z * wi_local.z >= 0.0);
        cos_hemisphere_pdf(wi_local)
    }
}

// Oren-Nayar
pub struct OrenNayar {
    albedo: Color,
    coeff_a: f32,
    coeff_b: f32,
}

impl OrenNayar {
    pub fn new(albedo: Color, sigma: math::hcm::Degree) -> Self {
        let math::hcm::Radian(sigma_rad) = sigma.to_radian();
        let sigma_sqr = sigma_rad.powi(2);
        let coeff_a = 1.0 - (sigma_sqr / (2.0 * (sigma_sqr + 0.33)));
        let coeff_b = 0.45 * sigma_sqr / (sigma_sqr + 0.09);
        Self {
            albedo,
            coeff_a,
            coeff_b,
        }
    }
}

impl BxDF for OrenNayar {
    fn get_type(&self) -> BxDFType {
        BxDFType {
            intrusion: IntrusionType::Reflection,
            smooth: SmoothnessType::Diffuse,
        }
    }

    fn eval(&self, wo_local: Vec3, wi_local: Vec3) -> Color {
        let sin_theta_i = local::sin_theta(wi_local);
        let sin_theta_o = local::sin_theta(wo_local);
        let (sin_phi_i, cos_phi_i) = local::sin_cos_phi(wi_local);
        let (sin_phi_o, cos_phi_o) = local::sin_cos_phi(wo_local);
        let delta_cos_phi = (cos_phi_i * cos_phi_o + sin_phi_i * sin_phi_o).max(0.0);
        let abs_cos_theta_i = local::cos_theta(wi_local).abs();
        let abs_cos_theta_o = local::cos_theta(wo_local).abs();
        let (sin_alpha, tan_beta) = if abs_cos_theta_i > abs_cos_theta_o {
            (sin_theta_o, sin_theta_i / abs_cos_theta_i)
        } else {
            (sin_theta_i, sin_theta_o / abs_cos_theta_o)
        };
        self.albedo
            * std::f32::consts::FRAC_1_PI
            * (self.coeff_a + self.coeff_b * delta_cos_phi * sin_alpha * tan_beta)
    }

    fn sample(&self, wo_local: Vec3, rnd2: (f32, f32)) -> (Vec3, HemiPdf, Color) {
        self.cosine_hemisphere_sample(wo_local, rnd2)
    }

    fn pdf(&self, wo_local: Vec3, wi_local: Vec3) -> f32 {
        assert!(wo_local.z * wi_local.z >= 0.0);
        cos_hemisphere_pdf(wi_local)
    }
}

/// Implements a general microfacet-based BRDF using Torrance-Sparrow model.
pub struct MicrofacetReflection {
    albedo: Color,
    distrib: mf::MicrofacetDistrib,
    fresnel: Fresnel,
}

impl MicrofacetReflection {
    pub fn new(albedo: Color, distrib: mf::MicrofacetDistrib, fresnel: Fresnel) -> Self {
        Self {
            albedo,
            distrib,
            fresnel,
        }
    }
}

impl BxDF for MicrofacetReflection {
    fn get_type(&self) -> BxDFType {
        BxDFType {
            intrusion: IntrusionType::Reflection,
            smooth: SmoothnessType::Glossy,
        }
    }

    fn eval(&self, wo_local: Vec3, wi_local: Vec3) -> Color {
        let cos_theta_o = local::cos_theta(wo_local).abs();
        let cos_theta_i = local::cos_theta(wi_local).abs();
        let wh_local = wo_local + wi_local;
        if cos_theta_o == 0.0 || cos_theta_i == 0.0 || wh_local.is_zero() {
            return Color::black();
        }
        let wh_local = wh_local.hat();
        let wh_local = wh_local * wh_local.z.signum();
        let refl_coeff = self.fresnel.refl_coeff(wi_local.dot(wh_local));
        println!("fresnel cos_theta = {}, refl coeff = {}", wi_local.dot(wh_local), refl_coeff);
        self.albedo * self.distrib.d(wh_local) * self.distrib.g(wo_local, wi_local) //* refl_coeff
            // / (4.0 * cos_theta_o * cos_theta_i)
    }

    fn sample(&self, wo_local: Vec3, rnd2: (f32, f32)) -> (Vec3, HemiPdf, Color) {
        // Samples microfacet normal wh and reflected direction wi.
        let wh_local = self.distrib.sample_wh(wo_local, rnd2);
        let wi_local = math::hcm::reflect(wh_local, wo_local);
        {
            let bisector = (wi_local + wo_local).hat();
            println!("similarity: {}", bisector.dot(wh_local));
        }
        if !local::same_hemisphere(wo_local, wi_local) {
            return (Vec3::zero(), HemiPdf::Regular(0.0), Color::black());
        }
        // Computes pdf of wi_local for microfacet reflection.
        // The pdf of wi_local is converted from that of wh_local and the following identities (that
        // only holds in the reflection coordinate using wo_local as the normal):
        //  - dw = sin(theta_w) * d theta * d phi
        //  - theta_h * 2 = theta_i
        let pdf = self.distrib.pdf(wo_local, wh_local) / (4.0 * wo_local.dot(wh_local));
        (
            wi_local,
            HemiPdf::Regular(pdf),
            self.eval(wo_local, wi_local),
        )
    }

    fn pdf(&self, wo_local: Vec3, wi_local: Vec3) -> f32 {
        if !local::same_hemisphere(wo_local, wi_local) {
            println!("not same hemisphere");
            return 0.0;
        }
        let wh_local = wo_local + wi_local;
        if wh_local.is_zero() {
            println!("degenerate");
            return 0.0;
        }
        self.distrib.pdf(wo_local, wh_local.hat()) / (4.0 * wo_local.dot(wh_local))
    }
}
