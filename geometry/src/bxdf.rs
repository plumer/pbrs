use crate::microfacet as mf;
use math::hcm::Vec3;
use math::float::Float;
use radiometry::color::Color;

/// A wrapper of `Vec3` representing unit-length vectors.
///
/// In describing ray scattering behavior on a surface, one often uses the lower-case greek letter
/// omega (ω) to represent the directions. One convention is that the normal vector is assumed to be
/// (0, 0, 1) (+z axis). The `Omega` class takes this convention and provides geometric semantics
/// in the reflection coordinate.
///
/// Any direction (2-DOF assuming unit-length), `w`, is described by 2 values:
///  - theta, the angle between `w` and the normal;
///  - phi, the angle of `w`'s projection onto the xy-plane from the x-axis.
///
/// `Omega` provides fast computation of trigonometric functions over theta and phi, assuming that
/// the vectors are unit-length:
///  - `cos_theta()`, `cos2_theta()`, `sin2_theta()`, `tan2_theta()`, `sin_theta()`
///     (in decreasing reliability)
///  - `sin_phi()`, `cos_phi()`, `sin2_phi()`, `cos2_phi()` (similar reliability)
///  - `same_hemisphere()`, checking if two `Omega`s are on the same side of the normal.
///  - `x()`, `y()`, `z()` accessors; `dot()`, `refract()`, `reflect()` (wrapper)
///  - `bisector()`, computes `(w0 + w1).hat()` or `None` in the degenerate case (w0 + w1 == 0).
#[derive(Clone, Copy)]
pub struct Omega(pub Vec3);
pub enum RefractResult {
    FullReflect(Omega),
    Transmit(Omega),
}

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

impl Omega {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self(Vec3::new(x, y, z))
    }
    pub fn normalize(x: f32, y: f32, z: f32) -> Self {
        Self(Vec3::new(x, y, z).hat())
    }
    pub fn normal() -> Self {
        Self(Vec3::zbase())
    }
    pub fn cos_theta(self) -> f32 {
        self.0.z
    }
    pub fn cos2_theta(self) -> f32 {
        self.0.z.powi(2)
    }
    pub fn sin_theta(self) -> f32 {
        self.sin2_theta().max(0.0).sqrt()
    }
    pub fn sin2_theta(self) -> f32 {
        1.0 - self.cos2_theta()
    }
    pub fn tan2_theta(self) -> f32 {
        self.sin2_theta() / self.cos2_theta()
    }

    pub fn cos_phi(self) -> f32 {
        let Self(Vec3 { x, y, z: _ }) = self;
        x.try_divide(x.hypot(y)).unwrap_or(1.0)
    }
    pub fn sin_phi(self) -> f32 {
        let Self(Vec3 { x, y, z: _ }) = self;
        y.try_divide(x.hypot(y)).unwrap_or(0.0)
    }
    pub fn cos2_phi(self) -> f32 {
        let Self(Vec3 { x, y, z: _ }) = self;
        (x * x).try_divide(x * x + y * y).unwrap_or(1.0)
    }
    pub fn sin2_phi(self) -> f32 {
        let Self(Vec3 { x, y, z: _ }) = self;
        (y * y).try_divide(x * x + y * y).unwrap_or(0.0)
    }

    pub fn sin_cos_phi(self) -> (f32, f32) {
        let Self(Vec3 { x, y, z: _ }) = self;
        let xy_hypot = x.hypot(y);
        if xy_hypot == 0.0 {
            (0.0, 1.0)
        } else {
            (x / xy_hypot, y / xy_hypot)
        }
    }

    /// Returns true if 2 vectors are on the same side of the surface.
    /// The normal of the surface is implicitly (0, 0, 1).
    pub fn same_hemisphere(w0: Omega, w1: Omega) -> bool {
        w0.cos_theta() * w1.cos_theta() >= 0.0
    }

    pub fn dot(self, other: Self) -> f32 {
        self.0.dot(other.0)
    }

    pub fn x(self) -> f32 {
        self.0.x
    }
    pub fn y(self) -> f32 {
        self.0.y
    }
    pub fn z(self) -> f32 {
        self.0.z
    }

    pub fn refract(normal: Omega, wi: Omega, ni_over_no: f32) -> RefractResult {
        let Omega(normal) = normal;
        let Omega(wi) = wi;
        match math::hcm::refract(normal, wi, ni_over_no) {
            math::hcm::FullReflect(wo) => RefractResult::FullReflect(Omega(wo)),
            math::hcm::Transmit(wo) => RefractResult::Transmit(Omega(wo)),
        }
    }

    pub fn reflect(normal: Omega, wi: Omega) -> Omega {
        let wo = math::hcm::reflect(normal.0, wi.0);
        Omega(wo)
    }

    pub fn bisector(w0: Omega, w1: Omega) -> Option<Omega> {
        let mid = w0.0 + w1.0;
        if mid.is_zero() {
            None
        } else {
            Some(Omega(mid.hat()))
        }
    }

    /// Flips the direction if it is opposite to the given normal.
    pub fn face_forward(self, normal: Self) -> Self {
        if self.dot(normal).is_sign_negative() {
            Self(-self.0)
        } else {
            self
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

pub fn cos_sample_hemisphere(uv: (f32, f32)) -> Omega {
    let (x, y) = concentric_sample_disk(uv);
    let z = (1.0 - x * x - y * y).max(0.0).sqrt();
    Omega::new(x, y, z)
}

pub fn cos_hemisphere_pdf(w: Omega) -> f32 {
    w.cos_theta() * std::f32::consts::FRAC_1_PI
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
    fn eval(&self, wo: Omega, wi: Omega) -> Color;

    /// Produces a possible incident direction given the outgoing direction, returning the
    /// probability density of the resulting direction and consumes a 2D random variable
    /// if the behavior is probabilistic (e.g., lambertian), and the BSDF value.
    ///
    /// Returned values from one such invocation produces values needed for one sample of
    /// contribution to the monte-carlo integration process.
    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Omega, HemiPdf, Color);

    fn pdf(&self, wo: Omega, wi: Omega) -> f32;

    /// A default implementation of the `sample` method. Uses cosine-hemisphere distribution
    /// to map the 2D random variable, and uses `pdf` and `eval` to compute the return values.
    fn cosine_hemisphere_sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Omega, HemiPdf, Color) {
        assert!(wo.cos_theta() >= 0.0);
        let wi = cos_sample_hemisphere(rnd2);
        (wi, HemiPdf::Regular(self.pdf(wo, wi)), self.eval(wo, wi))
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

    fn eval(&self, _wo: Omega, _wi: Omega) -> Color {
        Color::new(0.0, 0.0, 0.0)
    }

    fn sample(&self, wo: Omega, _rnd2: (f32, f32)) -> (Omega, HemiPdf, Color) {
        let wi = Omega::new(-wo.x(), -wo.y(), wo.z());
        let fr_refl = self.fresnel.refl_coeff(wi.cos_theta());
        (
            wi,
            HemiPdf::Delta(1.0),
            fr_refl * self.albedo / wi.cos_theta().abs(),
        )
    }

    fn pdf(&self, _wo: Omega, _wi: Omega) -> f32 {
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

    fn eval(&self, _wo: Omega, _wi: Omega) -> Color {
        Color::new(0.0, 0.0, 0.0)
    }

    fn sample(&self, wo: Omega, _rnd2: (f32, f32)) -> (Omega, HemiPdf, Color) {
        // Computes the normal for computing the refraction. It is flipped to the side forming an
        // acute angle with `wo`.
        let (eta_i, eta_t, normal) = if wo.cos_theta() > 0.0 {
            (self.eta_outer, self.eta_inner, Vec3::zbase())
        } else {
            (self.eta_inner, self.eta_outer, -Vec3::zbase())
        };

        match Omega::refract(Omega(normal), wo, eta_i / eta_t) {
            RefractResult::FullReflect(_) => (
                Omega::new(0.0, 0.0, 0.0),
                HemiPdf::Delta(1.0),
                Color::black(),
            ),
            RefractResult::Transmit(wi) => {
                // Transmissivity = 1 - reflectivity
                let f_tr = 1.0 - self.fresnel.refl_coeff(wi.cos_theta());
                // if transport_mode is radiance: f_t *= (eta_i / eta_t)^2
                (
                    wi,
                    HemiPdf::Delta(1.0),
                    (f_tr / wi.cos_theta().abs()) * self.albedo,
                )
            }
        }
    }

    fn pdf(&self, _wo: Omega, _wi: Omega) -> f32 {
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

    fn eval(&self, _wo: Omega, _wi: Omega) -> Color {
        Color::new(0.0, 0.0, 0.0)
    }

    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Omega, HemiPdf, Color) {
        let refl_coeff = Fresnel::dielectric(self.eta_a, self.eta_b).refl_coeff(wo.cos_theta());
        let (u, _v) = rnd2;
        if u < refl_coeff {
            // Samples a reflective direction.
            let wi = Omega::new(-wo.x(), -wo.y(), wo.z());
            (
                wi,
                HemiPdf::Regular(refl_coeff),
                refl_coeff * self.reflect_albedo / wi.cos_theta().abs(),
            )
        } else {
            // Samples a transmissive direction.
            let (eta_i, eta_t, normal) = if wo.cos_theta() > 0.0 {
                (self.eta_a, self.eta_b, Vec3::zbase())
            } else {
                (self.eta_b, self.eta_a, -Vec3::zbase())
            };
            match Omega::refract(Omega(normal), wo, eta_i / eta_t) {
                RefractResult::FullReflect(wi) => (wi, HemiPdf::Delta(refl_coeff), Color::black()),
                RefractResult::Transmit(wi) => {
                    // Transmissivity = 1 - reflectivity
                    let transmit_coeff = 1.0 - refl_coeff;
                    // if transport_mode is radiance: f_t *= (eta_i / eta_t)^2
                    (
                        wi,
                        HemiPdf::Delta(transmit_coeff),
                        (transmit_coeff / wi.cos_theta().abs()) * self.transmit_albedo,
                    )
                }
            }
        }
    }

    fn pdf(&self, _wo: Omega, _wi: Omega) -> f32 {
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

    fn eval(&self, _wo: Omega, _wi: Omega) -> Color {
        self.albedo * Self::FRAC_1_PI
    }

    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Omega, HemiPdf, Color) {
        self.cosine_hemisphere_sample(wo, rnd2)
    }

    fn pdf(&self, wo: Omega, wi: Omega) -> f32 {
        assert!(wo.z() * wi.z() >= 0.0);
        cos_hemisphere_pdf(wi)
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

    fn eval(&self, wo: Omega, wi: Omega) -> Color {
        let sin_theta_i = wi.sin_theta();
        let sin_theta_o = wo.sin_theta();
        let (sin_phi_i, cos_phi_i) = wi.sin_cos_phi();
        let (sin_phi_o, cos_phi_o) = wo.sin_cos_phi();
        let delta_cos_phi = (cos_phi_i * cos_phi_o + sin_phi_i * sin_phi_o).max(0.0);
        let abs_cos_theta_i = wi.cos_theta().abs();
        let abs_cos_theta_o = wo.cos_theta().abs();
        let (sin_alpha, tan_beta) = if abs_cos_theta_i > abs_cos_theta_o {
            (sin_theta_o, sin_theta_i / abs_cos_theta_i)
        } else {
            (sin_theta_i, sin_theta_o / abs_cos_theta_o)
        };
        self.albedo
            * std::f32::consts::FRAC_1_PI
            * (self.coeff_a + self.coeff_b * delta_cos_phi * sin_alpha * tan_beta)
    }

    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Omega, HemiPdf, Color) {
        self.cosine_hemisphere_sample(wo, rnd2)
    }

    fn pdf(&self, wo: Omega, wi: Omega) -> f32 {
        assert!(wo.z() * wi.z() >= 0.0);
        cos_hemisphere_pdf(wi)
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

    fn eval(&self, wo: Omega, wi: Omega) -> Color {
        let cos_theta_o = wo.cos_theta().abs();
        let cos_theta_i = wi.cos_theta().abs();
        let wh = Omega::bisector(wo, wi);
        if cos_theta_o == 0.0 || cos_theta_i == 0.0 || wh.is_none() {
            return Color::black();
        }
        let wh = wh.unwrap().face_forward(Omega::normal());
        let refl_coeff = self.fresnel.refl_coeff(wi.dot(wh));
        println!(
            "fresnel cos_theta = {}, refl coeff = {}",
            wi.dot(wh),
            refl_coeff
        );
        self.albedo * self.distrib.d(wh) * self.distrib.g(wo, wi) //* refl_coeff
                                                                  // / (4.0 * cos_theta_o * cos_theta_i)
    }

    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Omega, HemiPdf, Color) {
        // Samples microfacet normal wh and reflected direction wi.
        let wh = self.distrib.sample_wh(wo, rnd2);
        let wi = Omega::reflect(wh, wo);
        if !Omega::same_hemisphere(wo, wi) {
            return (
                Omega::new(0.0, 0.0, 0.0),
                HemiPdf::Regular(0.0),
                Color::black(),
            );
        }
        // Computes pdf of wi for microfacet reflection.
        // The pdf of wi is converted from that of wh and the following identities (that
        // only holds in the reflection coordinate using wo as the normal):
        //  - dw = sin(theta_w) * d theta * d phi
        //  - theta_h * 2 = theta_i
        let pdf = self.distrib.pdf(wo, wh) / (4.0 * wo.dot(wh));
        (wi, HemiPdf::Regular(pdf), self.eval(wo, wi))
    }

    fn pdf(&self, wo: Omega, wi: Omega) -> f32 {
        if !Omega::same_hemisphere(wo, wi) {
            return 0.0;
        }
        if let Some(wh) = Omega::bisector(wo, wi) {
            self.distrib.pdf(wo, wh) / (4.0 * wo.dot(wh))
        } else {
            0.0
        }
    }
}
