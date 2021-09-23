use crate::bxdf::Omega;

/// Microfacet distribution functions. Can be modelled by either Beckmann-Spizzichino or Trowbridge-
/// Reitz functions.
///
/// Models the micro-structure of many rough surfaces (including metas, plastic, frosted glass),
/// by providing the distribution of microfacet normals as a continuous hemisphere distribution.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub enum MicrofacetDistrib {
    Beckmann { alpha_x: f32, alpha_y: f32 },
    TrowbridgeReitz { alpha_x: f32, alpha_y: f32 },
}

impl MicrofacetDistrib {
    pub fn roughness_to_alpha(roughness: f32) -> f32 {
        let x = roughness.max(1e-3).ln();
        1.62142
            + 0.819955 * x
            + 0.1734 * x * x
            + 0.0171201 * x * x * x
            + 0.000640711 * x * x * x * x
    }
    pub fn beckmann(alpha_x: f32, alpha_y: f32) -> Self {
        Self::Beckmann { alpha_x, alpha_y }
    }
    pub fn trowbridge_reitz(alpha_x: f32, alpha_y: f32) -> Self {
        Self::TrowbridgeReitz { alpha_x, alpha_y }
    }

    /// Differential area of microfacets of normal `wh`.
    /// Should satisfy
    /// ```ignore
    ///  integrate(hemisphere, D(wh) cos_theta(wh) d(wh)) = 1
    /// ```
    pub fn d(&self, wh: Omega) -> f32 {
        let tan2_theta = wh.tan2_theta();
        let cos4_theta = wh.cos2_theta().powi(2);

        if tan2_theta.is_infinite() {
            0.0
        } else {
            assert!(!cos4_theta.is_infinite());
            match self {
                Self::Beckmann { alpha_x, alpha_y } => {
                    let x = wh.cos2_phi() / alpha_x.powi(2) + wh.sin2_phi() / alpha_y.powi(2);
                    (x * -tan2_theta).exp()
                        / (std::f32::consts::PI * alpha_x * alpha_y * cos4_theta)
                }
                Self::TrowbridgeReitz { alpha_x, alpha_y } => {
                    let e = wh.cos2_phi() / alpha_x.powi(2) + wh.sin2_phi() / alpha_y.powi(2);
                    ((1.0 + e * tan2_theta).powi(2)
                        * (std::f32::consts::PI * alpha_x * alpha_y * cos4_theta))
                        .recip()
                }
            }
        }
    }

    /// Measures invisible masked microfacet area, per visibile microfacet area, or in math:
    ///
    /// A-(w) / (A+(w) - A-(w))
    fn lambda(&self, w: Omega) -> f32 {
        let abs_tan_theta = w.tan2_theta().sqrt().abs();
        if abs_tan_theta.is_infinite() {
            0.0
        } else {
            match self {
                Self::Beckmann { alpha_x, alpha_y } => {
                    let alpha =
                        (w.cos2_phi() * alpha_x.powi(2) + w.sin2_phi() * alpha_y.powi(2)).sqrt();
                    let a = (alpha * abs_tan_theta).recip();
                    if a >= 1.6 {
                        0.0
                    } else {
                        (1.0 - 1.259 * a + 0.396 * a * a) / (3.535 * a + 2.181 * a * a)
                    }
                }
                Self::TrowbridgeReitz { alpha_x, alpha_y } => {
                    let alpha2 = w.cos2_phi() * alpha_x.powi(2) + w.sin2_phi() * alpha_y.powi(2);
                    let alpha2_tan2_theta = alpha2 * w.tan2_theta();
                    (-1.0 + (1.0 + alpha2_tan2_theta).sqrt()) * 0.5
                }
            }
        }
    }

    /// Masking-shadowing function, giving the fraction of microfacets that is visible from angle
    /// `w`. Usually this is a function of the normal of the microfacet as well, but we assume
    /// the indenpendence here. This implies the following property:
    /// ```ignore
    /// integrate(g1(w) * max(0.0, dot(w, wh)) * diffarea(wh) d(wh) in hemisphere) = cos_theta(w)
    /// ```
    /// In math it is closely related to lambda:
    ///
    /// [A+(w) - A-(w)] / A+(w)
    pub fn g1(&self, w: Omega) -> f32 {
        (1.0 + self.lambda(w)).recip()
    }

    /// Measures the fraction microfacets visible from both `wo` and `wi` angles.
    /// Usually the assumption that G(wo, wi) = G1(wo) * G1(wi) does not hold. A more accurate
    /// method is (1.0 + Lambda(wo) + Lambda(wi)) ^ {-1}
    pub fn g(&self, wo: Omega, wi: Omega) -> f32 {
        (1.0 + self.lambda(wo) + self.lambda(wi)).recip()
    }

    pub fn pdf(&self, wo: Omega, wh: Omega) -> f32 {
        #[cfg(sample_visible_area)]
        {
            self.d(wh) * self.g1(wo) * wo.dot(wh).abs() / wo.cos_theta().abs()
        }
        #[cfg(not(sample_visible_area))]
        {
            self.d(wh) * wo.cos_theta().abs()
        }
    }

    #[cfg(not(sample_visible_area))]
    pub fn sample_wh(&self, wo: Omega, rnd2: (f32, f32)) -> Omega {
        use std::f32::consts::{FRAC_PI_2, PI};

        let (u, v) = rnd2;
        match self {
            Self::Beckmann { alpha_x, alpha_y } => {
                let (tan2_theta, phi) = if alpha_x == alpha_y {
                    let log_sample = (1.0 - u).ln();
                    assert!(log_sample.is_finite());
                    // if log_sample.is_infinite() {
                    //     log_sample = 0.0;
                    // }
                    (-alpha_x.powi(2) * log_sample, v * 2.0 * PI)
                } else {
                    let log_sample = (1.0 - u).ln();
                    assert!(log_sample.is_finite());
                    let mut phi = (alpha_y / alpha_x * (2.0 * PI * v + FRAC_PI_2).tan()).atan();
                    if v >= 0.5 {
                        phi += PI;
                    }
                    let (sin_phi, cos_phi) = phi.sin_cos();
                    let alpha2 = (cos_phi / alpha_x).powi(2) + (sin_phi / alpha_y).powi(2);
                    (-log_sample / alpha2, phi)
                };
                let cos_theta = (1.0 + tan2_theta).sqrt().recip();
                let sin_theta = cos_theta * tan2_theta.sqrt();
                let wh =
                    math::hcm::spherical_direction(sin_theta, cos_theta, math::hcm::Radian(phi));
                Omega(wh).face_forward(wo)
            }
            Self::TrowbridgeReitz { alpha_x, alpha_y } => {
                todo!("{} / {}", alpha_x, alpha_y)
            }
        }
    }
}
