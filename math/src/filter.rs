use std::f32::consts::PI;

use crate::float::Float;

#[derive(Debug, Clone, Copy)]
pub struct UV {
    pub u: f32,
    pub v: f32,
}

impl UV {
    pub fn midpoint() -> Self {
        UV { u: 0.5, v: 0.5 }
    }
    pub fn as_pair(self) -> (f32, f32) {
        (self.u, self.v)
    }
}

#[allow(non_snake_case)]
pub enum Filter {
    Box { radius: UV },
    Triangle { radius: UV },
    Gaussian { radius: UV, alpha: f32 },
    MitchellNetravali { radius: UV, B: f32, C: f32 },
    LanczosSinc { radius: UV, tau: f32 },
}

impl Filter {
    pub fn eval(&self, offset_to_center: UV) -> f32 {
        let (ox, oy) = offset_to_center.as_pair();
        match *self {
            Self::Box { .. } => 1.0,
            Self::Triangle { radius } => {
                let (rx, ry) = radius.as_pair();
                (rx - ox.abs()).max(0.0) * (ry - oy.abs()).max(0.0)
            }
            Self::Gaussian { radius, alpha } => {
                let (rx, ry) = radius.as_pair();
                let gx = (-alpha * ox * ox) - (-alpha * rx * rx).exp();
                let gy = (-alpha * oy * oy) - (-alpha * ry * ry).exp();
                gx.max(0.0) * gy.max(0.0)
            }
            Self::MitchellNetravali { radius, B, C } => {
                let (rx, ry) = radius.as_pair();
                Self::mitchell_netravali_1d(ox / rx, B, C)
                    * Self::mitchell_netravali_1d(oy / ry, B, C)
            }
            Self::LanczosSinc { radius, tau } => {
                let windowed_sinc = |x: f32, r: f32| {
                    let x = x.abs();
                    if x > r {
                        0.0
                    } else {
                        Self::sinc(x / tau) * Self::sinc(x)
                    }
                };
                windowed_sinc(ox, radius.u) * windowed_sinc(oy, radius.v)
            }
        }
    }

    pub fn sinc(x: f32) -> f32 {
        let x = x.abs();
        if x < 1e-5 {
            1.0
        } else {
            (PI * x).sin() / (PI * x)
        }
    }

    pub fn mitchell_netravali_1d(x: f32, b: f32, c: f32) -> f32 {
        let x = (2.0 * x).abs();
        (1.0 / 6.0)
            * if x > 1.0 {
                x.polynomial([
                    8.0 * b + 24.0 * c,
                    -12.0 * b - 48.0 * c,
                    6.0 * b + 30.0 * c,
                    -b - 6.0 * c,
                ])
            } else {
                x.polynomial([
                    6.0 - 2.0 * b,
                    0.0,
                    -18.0 + 12.0 * b + 6.0 * c,
                    12.0 - 9.0 * b - 6.0 * c,
                ])
            }
    }
}