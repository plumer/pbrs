use std::f32::consts::FRAC_1_PI;

use crate::microfacet::{self as mf};
use math::float::Float;
use math::hcm::Vec3;
use math::prob::Prob;
use radiometry::color::{Color, XYZ};

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

#[derive(Debug)]
pub enum IntrusionType {
    Reflection,
    Transmission,
    Hybrid,
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

    /// Tesselates the +Z hemisphere into N x 4N cell, returning the direction at the center of
    /// each cell and `d_theta, d_phi`.
    pub fn tesselate_hemi(count: i32) -> (Vec<Self>, (f32, f32)) {
        use math::float::linspace;
        use std::f32::consts::{FRAC_PI_2, PI};
        let (thetas, d_theta) = linspace((0.0, FRAC_PI_2), count);
        let (phis, d_phi) = linspace((0.0, PI * 2.0), count * 4);

        let mut vectors = vec![];
        for theta in thetas.into_iter() {
            for phi in phis.iter().copied() {
                let (sin_theta, cos_theta) = theta.sin_cos();
                let wo = math::hcm::spherical_direction(sin_theta, cos_theta, math::new_rad(phi));

                assert!(wo.norm_squared().dist_to(1.0) < 1e-3);
                vectors.push(Self(wo));
            }
        }
        (vectors, (d_theta, d_phi))
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
    w.cos_theta() * FRAC_1_PI
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
/// - `MatteReflection`, `MicrofacetReflection`
pub trait BxDF {
    /// Evaluates the BSDF function at given in-out angles. Note that specular BSDFs always return
    /// 0. Use [`sample()`] in those cases instead.
    fn eval(&self, wo: Omega, wi: Omega) -> Color;

    /// Produces a possible incident direction given the outgoing direction, returning the
    /// probability density of the resulting direction and consumes a 2D random variable
    /// if the behavior is probabilistic (e.g., lambertian), and the BSDF value.
    ///
    /// Returned values from one such invocation produces values needed for one sample of
    /// contribution to the monte-carlo integration process.
    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Color, Omega, Prob);

    fn prob(&self, wo: Omega, wi: Omega) -> Prob;

    /// A default implementation of the `sample` method. Uses cosine-hemisphere distribution
    /// to map the 2D random variable, and uses `pdf` and `eval` to compute the return values.
    fn cosine_hemisphere_sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Color, Omega, Prob) {
        assert!(wo.cos_theta() >= 0.0);
        let wi = cos_sample_hemisphere(rnd2);
        (self.eval(wo, wi), wi, self.prob(wo, wi))
    }
}

// ----------------------------
#[derive(Debug, Clone, Copy)]
#[rustfmt::skip]
pub enum Fresnel {
    Nop,
    /// Fresnel reflectivity ratio computation for dielectric materials (e.g., glass).
    Dielectric { eta_front: f32, eta_back: f32 },
    Conductor {eta_i: XYZ, eta_t: XYZ, k: XYZ}
}

impl Fresnel {
    pub fn dielectric(eta_front: f32, eta_back: f32) -> Self {
        Self::Dielectric {
            eta_front,
            eta_back,
        }
    }

    /// Computes the ratio of refracted energy.
    pub fn refl_coeff(&self, cos_theta_i: f32) -> f32 {
        match self {
            Self::Nop => 1.0,
            Self::Dielectric {
                eta_front: eta_i,
                eta_back: eta_t,
            } => {
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
            Self::Conductor { .. } => panic!(),
        }
    }

    pub fn eval(&self, cos_theta_i: f32) -> Color {
        match self {
            Self::Nop | Self::Dielectric { .. } => Color::gray(self.refl_coeff(cos_theta_i)),
            /*    cosThetaI = Clamp(cosThetaI, -1, 1);
            Spectrum eta = etat / etai;
            Spectrum etak = k / etai;

            Float cosThetaI2 = cosThetaI * cosThetaI;
            Float sinThetaI2 = 1. - cosThetaI2;
            Spectrum eta2 = eta * eta;
            Spectrum etak2 = etak * etak;

            Spectrum t0 = eta2 - etak2 - sinThetaI2;
            Spectrum a2plusb2 = Sqrt(t0 * t0 + 4 * eta2 * etak2);
            Spectrum t1 = a2plusb2 + cosThetaI2;
            Spectrum a = Sqrt(0.5f * (a2plusb2 + t0));
            Spectrum t2 = (Float)2 * cosThetaI * a;
            Spectrum Rs = (t1 - t2) / (t1 + t2);

            Spectrum t3 = cosThetaI2 * a2plusb2 + sinThetaI2 * sinThetaI2;
            Spectrum t4 = t2 * sinThetaI2;
            Spectrum Rp = Rs * (t3 - t4) / (t3 + t4);

            return 0.5 * (Rp + Rs); */
            Self::Conductor { eta_i, eta_t, k } => {
                let eta = *eta_t / *eta_i;
                let eta2 = eta * eta;
                let etak = *k / *eta_i;
                let etak2 = etak * etak;
                let cos2_theta_i = cos_theta_i.clamp(-1.0, 1.0).powi(2);
                let sin2_theta_i = 1.0 - cos2_theta_i;

                let t0 = eta2 - etak2 - XYZ::all(sin2_theta_i);
                let a2_plus_b2 = (t0 * t0 + 4.0 * eta2 * etak2).sqrt();
                let t1 = a2_plus_b2 + XYZ::all(cos2_theta_i);
                let a = ((a2_plus_b2 + t0) * 0.5).sqrt();
                let t2 = 2.0 * a * cos_theta_i;
                let ratio_s = (t1 - t2) / (t1 + t2);

                let t3 = cos2_theta_i * a2_plus_b2 + XYZ::all(sin2_theta_i.powi(2));
                let t4 = t2 * sin2_theta_i;
                let ratio_p = ratio_s * (t3 - t4) / (t3 + t4);

                ((ratio_s + ratio_p) * 0.5).to_color()
            }
        }
    }
}

pub struct Specular {
    fresnel: Fresnel,
    albedo: Color,
    intrusion: IntrusionType,
}
impl Specular {
    pub fn mirror(albedo: Color) -> Self {
        Self {
            fresnel: Fresnel::Nop,
            albedo,
            intrusion: IntrusionType::Reflection,
        }
    }

    /// BSDF representing dielectric material (e.g., glass). Ray scattering is both reflective and
    /// transmissive.
    pub fn dielectric(albedo: Color, eta_outer: f32, eta_inner: f32) -> Self {
        Self {
            fresnel: Fresnel::dielectric(eta_outer, eta_inner),
            albedo,
            intrusion: IntrusionType::Hybrid,
        }
    }

    fn reflect(&self, wo: Omega) -> (Omega, Color) {
        let wi = Omega::new(-wo.x(), -wo.y(), wo.z());
        let fr_refl = self.fresnel.eval(wi.cos_theta());
        (wi, fr_refl * self.albedo / wi.cos_theta().abs())
    }

    fn refract(&self, wo: Omega, eta_front: f32, eta_back: f32) -> (Omega, Color) {
        // Computes the normal for computing the refraction. It is flipped to the side forming an
        // acute angle with `wo`.
        let (eta_i, eta_t, normal) = if wo.cos_theta() > 0.0 {
            (eta_front, eta_back, Vec3::zbase())
        } else {
            (eta_back, eta_front, -Vec3::zbase())
        };

        match Omega::refract(Omega(normal), wo, eta_i / eta_t) {
            RefractResult::FullReflect(_) => (Omega::new(0.0, 0.0, 0.0), Color::black()),
            RefractResult::Transmit(wi) => {
                // Transmissivity = 1 - reflectivity
                let f_tr = 1.0 - self.fresnel.refl_coeff(wi.cos_theta());
                // if transport_mode is radiance: f_t *= (eta_i / eta_t)^2
                (wi, (f_tr / wi.cos_theta().abs()) * self.albedo)
            }
        }
    }
}

impl BxDF for Specular {
    fn eval(&self, _wo: Omega, _wi: Omega) -> Color {
        return Color::black();
    }

    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Color, Omega, Prob) {
        use Fresnel::*;
        use IntrusionType::*;
        match (&self.intrusion, self.fresnel) {
            // Perfect reflection.
            (Reflection, _) => {
                let (wi, color) = self.reflect(wo);
                (color, wi, Prob::Mass(1.0))
            }
            // Perfect refraction (no reflection, which is unintuitive...).
            (
                Transmission,
                Dielectric {
                    eta_front,
                    eta_back,
                },
            ) => {
                let (wi, color) = self.refract(wo, eta_front, eta_back);
                (color, wi, Prob::Mass(1.0))
            }
            // Both reflective and refractive (seen in most dielectrics).
            (
                Hybrid,
                Dielectric {
                    eta_front,
                    eta_back,
                },
            ) => {
                let refl_coeff = self.fresnel.refl_coeff(wo.cos_theta());
                if rnd2.0 < refl_coeff {
                    let (wi, color) = self.reflect(wo);
                    (color, wi, Prob::Mass(refl_coeff))
                } else {
                    let (wi, color) = self.refract(wo, eta_front, eta_back);
                    (color, wi, Prob::Mass(1.0 - refl_coeff))
                }
            }
            _ => panic!("Unmatched combo: {:?}, {:?}", self.intrusion, self.fresnel),
        }
    }

    fn prob(&self, _wo: Omega, _wi: Omega) -> Prob {
        Prob::Mass(0.0)
    }
}

#[derive(Debug, Clone, Copy)]
enum MatteModel {
    Lambertian,
    OrenNayar { coeff_a: f32, coeff_b: f32 },
}

#[derive(Debug, Clone)]
pub struct DiffuseReflect {
    albedo: Color,
    model: MatteModel,
}

impl DiffuseReflect {
    pub fn lambertian(albedo: Color) -> Self {
        Self {
            albedo,
            model: MatteModel::Lambertian,
        }
    }
    /// `sigma`: standard deviation of the microfacet orientation angle
    pub fn oren_nayar(albedo: Color, sigma: math::Angle) -> Self {
        let sigma_sqr = sigma.to_rad().powi(2);
        let coeff_a = 1.0 - (sigma_sqr / (2.0 * (sigma_sqr + 0.33)));
        let coeff_b = 0.45 * sigma_sqr / (sigma_sqr + 0.09);
        Self {
            albedo,
            model: MatteModel::OrenNayar { coeff_a, coeff_b },
        }
    }
}

impl BxDF for DiffuseReflect {
    fn eval(&self, wo: Omega, wi: Omega) -> Color {
        match self.model {
            MatteModel::Lambertian => self.albedo * FRAC_1_PI,
            MatteModel::OrenNayar { coeff_a, coeff_b } => {
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
                self.albedo * FRAC_1_PI * (coeff_a + coeff_b * delta_cos_phi * sin_alpha * tan_beta)
            }
        }
    }
    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Color, Omega, Prob) {
        assert!(wo.cos_theta() >= 0.0);
        let wi = cos_sample_hemisphere(rnd2);
        (self.eval(wo, wi), wi, self.prob(wo, wi))
    }

    fn prob(&self, wo: Omega, wi: Omega) -> Prob {
        if wo.z() * wi.z() >= 0.0 {
            Prob::Density(cos_hemisphere_pdf(wi))
        } else {
            Prob::Density(0.0)
        }
    }
}

#[derive(Debug, Clone)]
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
    fn eval(&self, wo: Omega, wi: Omega) -> Color {
        let cos_theta_o = wo.cos_theta().abs();
        let cos_theta_i = wi.cos_theta().abs();
        let wh = Omega::bisector(wo, wi);
        if cos_theta_o == 0.0 || cos_theta_i == 0.0 || wh.is_none() {
            return Color::black();
        }
        let wh = wh.unwrap().face_forward(Omega::normal());
        let refl = self.fresnel.eval(wi.dot(wh));
        // println!("fresnel cos_theta = {}, refl coeff = {}", wi.dot(wh), refl);
        self.albedo * self.distrib.d(wh) * self.distrib.g(wo, wi) * refl
            / (4.0 * cos_theta_o * cos_theta_i)
    }

    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Color, Omega, Prob) {
        // Samples microfacet normal wh and reflected direction wi.
        let wh = self.distrib.sample_wh(wo, rnd2);
        let wi = Omega::reflect(wh, wo);
        if !Omega::same_hemisphere(wo, wi) {
            return (Color::black(), Omega::normal(), Prob::Density(0.0));
        }
        // Computes pdf of wi for microfacet reflection.
        // The pdf of wi is converted from that of wh and the following identities (that
        // only holds in the reflection coordinate using wo as the normal):
        //  - dw = sin(theta_w) * d theta * d phi
        //  - theta_h * 2 = theta_i
        let pdf = self.distrib.pdf(wo, wh) / (4.0 * wo.dot(wh));
        (self.eval(wo, wi), wi, Prob::Density(pdf))
    }

    fn prob(&self, wo: Omega, wi: Omega) -> Prob {
        if !Omega::same_hemisphere(wo, wi) {
            return Prob::Density(0.0);
        }
        if let Some(wh) = Omega::bisector(wo, wi) {
            Prob::Density(self.distrib.pdf(wo, wh) / (4.0 * wo.dot(wh)))
        } else {
            Prob::Density(0.0)
        }
    }
}
