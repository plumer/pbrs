use crate::bxdf::local;
use math::hcm::Vec3;

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub enum MicrofacetDistrib {
    Beckmann { alpha_x: f32, alpha_y: f32 },
    TrowbridgeReitz { alpha_x: f32, alpha_y: f32 },
}

impl MicrofacetDistrib {
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
    pub fn d(&self, wh: Vec3) -> f32 {
        let tan2_theta = local::tan2_theta(wh);
        let cos4_theta = local::cos2_theta(wh).powi(2);

        if tan2_theta.is_infinite() {
            0.0
        } else {
            assert!(!cos4_theta.is_infinite());
            match self {
                Self::Beckmann { alpha_x, alpha_y } => {
                    let x = local::cos2_phi(wh) / alpha_x.powi(2)
                        + local::sin2_phi(wh) / alpha_y.powi(2);
                    (x * -tan2_theta).exp()
                        / (std::f32::consts::PI * alpha_x * alpha_y * cos4_theta)
                }
                Self::TrowbridgeReitz { alpha_x, alpha_y } => {
                    let e = local::cos2_phi(wh) / alpha_x.powi(2)
                        + local::sin2_phi(wh) / alpha_y.powi(2);
                    ((1.0 + e * tan2_theta).powi(2)
                        * (std::f32::consts::PI * alpha_x * alpha_y * cos4_theta))
                        .recip()
                }
            }
        }
    }

    fn lambda(&self, w_local: Vec3) -> f32 {
        let abs_tan_theta = local::tan2_theta(w_local).sqrt().abs();
        if abs_tan_theta.is_infinite() {
            0.0
        } else {
            match self {
                Self::Beckmann { alpha_x, alpha_y } => {
                    let alpha = (local::cos2_phi(w_local) * alpha_x.powi(2)
                        + local::sin2_phi(w_local) * alpha_y.powi(2))
                    .sqrt();
                    let a = (alpha * abs_tan_theta).recip();
                    if a >= 1.6 {
                        0.0
                    } else {
                        (1.0 - 1.259 * a + 0.396 * a * a) / (3.535 * a + 2.181 * a * a)
                    }
                }
                Self::TrowbridgeReitz { alpha_x, alpha_y } => {
                    let alpha2 = local::cos2_phi(w_local) * alpha_x.powi(2)
                        + local::sin2_phi(w_local) * alpha_y.powi(2);
                    let alpha2_tan2_theta = alpha2 * local::tan2_theta(w_local);
                    (-1.0 + (1.0 + alpha2_tan2_theta).sqrt()) * 0.5
                }
            }
        }
    }

    fn g1(&self, w_local: Vec3) -> f32 {
        (1.0 + self.lambda(w_local)).recip()
    }

    fn g(&self, wo_local: Vec3, wi_local: Vec3) -> f32 {
        (1.0 + self.lambda(wo_local) + self.lambda(wi_local)).recip()
    }

    fn pdf(&self, wo_local: Vec3, wh_local: Vec3) -> f32 {
        #[cfg(sample_visible_area)]
        {
            self.d(wh_local) * self.g1(wo_local) * wo_local.dot(wh_local).abs()
                / local::cos_theta(wo_local).abs()
        }
        #[cfg(not(sample_visible_area))]
        {
            self.d(wh_local) * local::cos_theta(wo_local).abs()
        }
    }
}
