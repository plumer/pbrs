use crate::bxdf::{BxDF, Omega};
use itertools::Itertools;
use math::{
    float::{Fallback, Float, Inside},
    prob::Prob,
};
use radiometry::color::Color;

use std::{
    f64::consts::{FRAC_1_PI, PI},
    io::Read,
};

/*
  File format description:

  This is the file format generated by the material designer of the paper

  'A Comprehensive Framework for Rendering Layered Materials' by
  Wenzel Jakob, Eugene D'Eon, Otto Jakob and Steve Marschner
  Transactions on Graphics (Proceedings of SIGGRAPH 2014)

  A standalone Python plugin for generating such data files is available
  on GitHub: https://github.com/wjakob/layerlab

  This format specifies an isotropic BSDF expressed in a Spline x Fourier
  directional basis. It begins with a header of the following type:

 struct Header {
     uint8_t identifier[7];     // Set to 'SCATFUN'
     uint8_t version;           // Currently version is 1
     uint32_t flags;            // 0x01: file contains a BSDF, 0x02: uses harmonic extrapolation
     int nMu;                   // Number of samples in the elevational discretization

     int nCoeffs;               // Total number of Fourier series coefficients stored in the file
     int mMax;                  // Coeff. count for the longest series occurring in the file
     int nChannels;             // Number of color channels (usually 1 or 3)
     int nBases;                // Number of BSDF basis functions (relevant for texturing)

     int nMetadataBytes;        // Size of descriptive metadata that follows the BSDF data
     int nParameters;           // Number of textured material parameters
     int nParameterValues;      // Total number of BSDF samples for all textured parameters
     float eta;                 // Relative IOR through the material (eta(bottom) / eta(top))

     float alpha[2];            // Beckmann-equiv. roughness on the top (0) and bottom (1) side
     float unused[2];           // Unused fields to pad the header to 64 bytes
 };

  Due to space constraints, two features are not currently implemented in PBRT,
  namely texturing and harmonic extrapolation (though it would be straightforward
  to port them from Mitsuba.)
*/

#[derive(Debug, Clone, Copy)]
#[repr(C, packed)]
struct FourierBsdfHeader {
    pub identifier: [u8; 7],
    pub version: u8,
    pub flags: u32,
    pub n_mu: i32,
    pub n_coeffs: i32,
    pub m_max: i32,
    pub n_channels: i32,
    pub n_bases: i32,
    pub n_metadata_bytes: i32,
    pub n_parameters: i32,
    pub n_parameter_values: i32,
    pub eta: f32,

    pub alpha: [f32; 2],
    pub unused: [f32; 2],
}

fn read_header(buffer: &[u8]) -> Result<FourierBsdfHeader, std::io::Error> {
    assert_eq!(std::mem::size_of::<FourierBsdfHeader>(), 64);

    let mut header: FourierBsdfHeader = unsafe { std::mem::zeroed() };
    unsafe {
        let header_slice = std::slice::from_raw_parts_mut(&mut header as *mut _ as *mut u8, 64);
        let mut buffer: &[u8] = &buffer;
        buffer.read_exact(header_slice).unwrap();
    }

    // Verification
    let identifier = buffer[0..7].iter().map(|&c| c as char).collect::<String>();
    assert_eq!(identifier, "SCATFUN");
    assert_eq!(header.version, 1);
    let flags = header.flags;
    assert_eq!(flags, 1);
    assert!(header.eta.is_finite());
    assert!(header.alpha[0].is_finite());
    assert!(header.alpha[1].is_finite());
    // println!("read structure: {:#?}", header);

    Ok(header)
}

#[derive(Default, Debug)]
pub struct FourierTable {
    pub m_max: usize,
    pub n_channels: usize, // Either 1 (monochromatic) or 3 (luminance, red and blue)
    // pub n_mu: usize,
    pub mu: Vec<f32>, // Set of discretized zenith angles (theta, angle between w and +Z)
    pub cdf: Vec<f32>,
    a0: Vec<f32>, // A cache of order-0 coefficients.
    pub a_offset: Vec<i32>,
    // A lookup table for the number of terms (m) to sum up a_k*cos(phi k). The order m can be
    // determined by querying m_lookup using 2 cos_theta (aka mu) values.
    pub m_lookup: Vec<i32>,
    pub a: Vec<f32>,
    pub recip: Vec<f32>,
}

impl FourierTable {
    pub fn build(
        n_channels: usize, mu: Vec<f32>, cdf: Vec<f32>, a_offset: Vec<i32>, m_lookup: Vec<i32>,
        coefficients: Vec<f32>,
    ) -> Self {
        assert!(n_channels == 1 || n_channels == 3);
        assert_eq!(mu.len().pow(2), cdf.len());
        assert_eq!(mu.len().pow(2), a_offset.len());
        assert_eq!(mu.len().pow(2), m_lookup.len());
        let m_max = *m_lookup.iter().max().unwrap() as usize;
        let a0 = (a_offset.iter().zip(m_lookup.iter()))
            .inspect(|(&offset, &length)| {
                let end = offset as usize + length as usize * n_channels;
                assert!(end <= coefficients.len());
            })
            .map(|(&offset, &length)| {
                if length > 0 {
                    coefficients[offset as usize]
                } else {
                    0.0
                }
            })
            .collect::<Vec<_>>();
        let recip = (0..m_max).map(|i| (i as f32).recip()).collect::<Vec<_>>();
        FourierTable {
            m_max,
            n_channels,
            mu,
            cdf,
            a0,
            a_offset,
            m_lookup,
            a: coefficients,
            recip,
        }
    }

    /// Returns the first index to the knots and weights to interpolate data points using
    /// Catmull-Rom formula.
    ///
    /// The first index may be -1; the last index (first index + 3) may be out of bounds
    /// (equal to `self.mu.len()`).
    pub fn get_weights_and_offset(&self, cos_theta: f32) -> Option<(isize, [f32; 4])> {
        math::spline::catmull_rom_weights(&self.mu, cos_theta)
    }

    pub fn get_ak(&self, offset_i: usize, offset_o: usize) -> (&[f32], usize) {
        let index = offset_o * self.mu.len() + offset_i;
        let m = self.m_lookup[index] as usize;
        let start = self.a_offset[index] as usize;
        (&self.a[start..start + m * self.n_channels], m)
    }

    pub fn from_file(path: &str) -> Result<FourierTable, std::io::Error> {
        let mut file = std::fs::File::open(path)?;
        let mut header_buffer = [0u8; 64];
        file.read_exact(&mut header_buffer)?;
        let header = read_header(&header_buffer)?;
        let n_mu = header.n_mu as usize;
        let n_coeffs = header.n_coeffs as usize;

        let read_n_f32s = |n: usize| {
            let mut whole_buffer = vec![0u8; 4 * n];
            (&file).read_exact(&mut whole_buffer).unwrap();
            (0..n)
                .map(|i| {
                    let slice = &whole_buffer[4 * i..4 * i + 4];
                    let bytes = [slice[0], slice[1], slice[2], slice[3]];
                    f32::from_ne_bytes(bytes)
                })
                .collect::<Vec<_>>()
        };
        let read_n_i32s = |n: usize| {
            let mut whole_buffer = vec![0u8; 4 * n];
            (&file).read_exact(&mut whole_buffer).unwrap();
            (0..n)
                .map(|i| {
                    let slice = &whole_buffer[4 * i..4 * i + 4];
                    let bytes = [slice[0], slice[1], slice[2], slice[3]];
                    i32::from_ne_bytes(bytes)
                })
                .collect::<Vec<_>>()
        };
        let mu = read_n_f32s(n_mu);
        for i in 0..mu.len() - 1 {
            assert!(mu[i] <= mu[i + 1]);
        }
        let cdf = read_n_f32s(n_mu * n_mu);
        let offset_and_length = read_n_i32s(n_mu * n_mu * 2);
        let a = read_n_f32s(n_coeffs);

        let mut a_offset = Vec::with_capacity(n_mu * n_mu);
        let mut m = Vec::with_capacity(n_mu * n_mu);
        for chunk in offset_and_length.chunks(2) {
            if let &[offset, length] = chunk {
                a_offset.push(offset);
                m.push(length);
            }
        }
        println!("raw data ready");
        let table = FourierTable::build(header.n_channels as usize, mu, cdf, a_offset, m, a);
        Ok(table)
    }
}

pub struct FourierBSDF<'a> {
    pub table: &'a FourierTable,
}

/// Computes the sum = sum_{0..n} a_k * cos(k * phi).
fn fourier_sum(a: &[f32], cos_phi: f32) -> f32 {
    // Initialize cosine iterates. This function uses Chebyshev's method to
    // compute cos((k+1)x) from cos((k-1)x) and cos(kx), in order to reduce f32::cos() invocations.
    // cos((k+1)x) = 2 cos(x) cos(kx) - cos((k-1)x)
    a.iter()
        .scan((cos_phi as f64, 1.0f64), |state, a_k| {
            let (cos_k_phi_prev, cos_k_phi) = *state;
            let cos_k_phi_next = 2.0 * cos_phi as f64 * cos_k_phi - cos_k_phi_prev;
            *state = (cos_k_phi, cos_k_phi_next);
            Some(*a_k as f64 * cos_k_phi)
        })
        .sum::<f64>() as f32
}

/// Samples from a distribution whose pdf matches the following function with given `a_k` coeffs:
///
/// `f(x) = sum{k in 0..m} a_k*cos(k*x)`
///
/// Parameter `recip` is a slice where `recip[i] = 1.0 / i`. 1.0/i values will be useful since the
/// CDF integrated from `f(x)` uses them.
/// Returns f-value at the sampled x-value, the x-value, and the PDF of the sample.
fn sample_fourier(ak: &[f32], recip: &[f32], u: f32) -> (f32, f32, Prob) {
    let flip = u >= 0.5;
    let u = match u >= 0.5 {
        true => 1.0 - 2.0 * (u - 0.5),
        false => u * 2.0,
    };
    let mut left = 0.0;
    let mut right = PI;
    let mut phi = 0.5f64 * PI;
    let sampled_f = loop {
        // Evaluates f(phi) and its integral F(phi).
        let (sin_phi, cos_phi) = phi.sin_cos(); // Init as sin(PI/2), cos(PI/2).

        let (f_integral, f) = (1..ak.len())
            .scan((cos_phi, 1.0, -sin_phi, 0.0), |state, k| {
                let (prev_cos_phi, curr_cos_phi, prev_sin_phi, curr_sin_phi) = *state;
                let next_sin_phi = 2.0 * cos_phi * curr_sin_phi - prev_sin_phi;
                let next_cos_phi = 2.0 * cos_phi * curr_cos_phi - prev_cos_phi;
                *state = (curr_cos_phi, next_cos_phi, curr_sin_phi, next_sin_phi);

                Some((
                    (ak[k] * recip[k]) as f64 * next_sin_phi,
                    ak[k] as f64 * next_cos_phi,
                ))
            })
            .fold(
                (ak[0] as f64 * phi, ak[0] as f64),
                |(intf_sum, f_sum), (intf_term, f_term)| (intf_sum + intf_term, f_sum + f_term),
            );
        // Solves the equation F(phi) - u * F(pi) = 0. In this case, F(phi) = ak[0] since
        // sin(k * pi) = 0 for all k, and ak[0] is free of the sine term.
        let f_integral = f_integral - (u * ak[0]) as f64 * PI;
        // Updates bisection bounds using updated phi.
        if f_integral > 0.0 {
            right = phi;
        } else {
            left = phi;
        }

        if f_integral.abs() < 1e-6 || right - left < 1e-6 {
            break f;
        }
        phi -= f_integral / f;
        if !phi.inside_open((left, right)) {
            phi = 0.5 * (left + right);
        }
    };
    if flip {
        phi = 2.0 * PI - phi;
    }
    let pdf = (sampled_f * FRAC_1_PI * 0.5) as f32 / ak[0];
    (sampled_f as f32, phi as f32, Prob::Density(pdf))
}

impl<'a> BxDF for FourierBSDF<'a> {
    fn eval(&self, wo: Omega, wi: Omega) -> Color {
        // Finds the zenith angle cosines and azimuth difference angle.
        let mu_i = -wi.cos_theta();
        let mu_o = wo.cos_theta();
        let cos_phi = wo.cos_dphi(-wi).clamp(-1.0, 1.0);
        assert!(
            cos_phi.inside((-1.0, 1.0)),
            "cos_phi = {}, wo = {}, wi = {}",
            cos_phi,
            wo.0,
            wi.0
        );

        let wt_offset_i = self.table.get_weights_and_offset(mu_i);
        let wt_offset_o = self.table.get_weights_and_offset(mu_o);

        if wt_offset_i.is_none() || wt_offset_o.is_none() {
            return Color::black();
        }
        // Determines offsets and weights for mu_i and mu_o.
        let (offset_i, weights_i) = wt_offset_i.unwrap();
        let (offset_o, weights_o) = wt_offset_o.unwrap();
        assert!(offset_i == -1 || (0..self.table.mu.len()).contains(&(offset_i as usize)));
        assert!(offset_o == -1 || (0..self.table.mu.len()).contains(&(offset_o as usize)));
        assert!(weights_i.iter().sum::<f32>().dist_to(1.0) < 1e-3);
        assert!(weights_o.iter().sum::<f32>().dist_to(1.0) < 1e-3);

        // Allocates storage to accumulate a_k coefficients.
        let mut a_k = vec![0.0; self.table.m_max * self.table.n_channels];

        // Accumulate weighted sums of nearby a_k coefficients.
        let mut m_max = 0;
        for (b, a) in (0..4).cartesian_product(0..4) {
            let weight = weights_i[a] * weights_o[b];
            if weight != 0.0 {
                let offset_i = (offset_i + a as isize) as usize;
                let offset_o = (offset_o + b as isize) as usize;
                let (ap, m) = self.table.get_ak(offset_i, offset_o);
                m_max = m_max.max(m);
                for c in 0..self.table.n_channels {
                    for k in 0..m {
                        a_k[c * self.table.m_max + k] += weight * ap[c * m + k];
                    }
                }
            }
        }

        // Evaluates Fourier expansion for angle phi.
        let (y_slice, rb_slice) = a_k.split_at(self.table.m_max);
        let y = fourier_sum(&y_slice[0..m_max], cos_phi).max(0.0);
        let scale = 1.0f32.try_divide(mu_i.abs()).unwrap_or(0.0);
        if self.table.n_channels == 1 {
            Color::gray(y * scale)
        } else {
            let (r_slice, b_slice) = rb_slice.split_at(self.table.m_max);
            let r = fourier_sum(&r_slice[0..m_max], cos_phi);
            let b = fourier_sum(&b_slice[0..m_max], cos_phi);
            let g = 1.39829 * y - 0.100913 * b - 0.297375 * r;
            (Color::new(r, g, b) * scale).clamp()
        }
    }

    fn sample(&self, wo: Omega, rnd2: (f32, f32)) -> (Color, Omega, math::prob::Prob) {
        // Samples zenith angle component.
        let (u, v) = rnd2;
        let mu_o = wo.cos_theta();
        let (_f_mu, mu_i, pr) = math::spline::sample_catmull_rom_2d(
            &self.table.mu,
            &self.table.mu,
            &self.table.a0,
            &self.table.cdf,
            mu_o,
            v,
        )
        .unwrap();
        let pdf_mu = pr.density();

        // Computes Fourier coefficients a_k for (mu_i, mu_o) pair.
        let wt_offset_i = self.table.get_weights_and_offset(mu_i);
        let wt_offset_o = self.table.get_weights_and_offset(mu_o);
        if wt_offset_i.is_none() || wt_offset_o.is_none() {
            return (Color::black(), Omega::normal(), Prob::Density(0.0));
        }
        // Determines offsets and weights for mu_i and mu_o.
        let (offset_i, weights_i) = wt_offset_i.unwrap();
        let (offset_o, weights_o) = wt_offset_o.unwrap();
        // Allocates storage to accumulate a_k coefficients.
        let mut a_k = vec![0.0; self.table.m_max * self.table.n_channels];

        // Accumulate weighted sums of nearby a_k coefficients.
        let mut m_max = 0;
        for (o, i) in (0..4).cartesian_product(0..4) {
            let weight = weights_i[i] * weights_o[o];
            if weight != 0.0 {
                let offset_i = (offset_i + i as isize) as usize;
                let offset_o = (offset_o + o as isize) as usize;
                let (ap, m) = self.table.get_ak(offset_i, offset_o);
                // assert_gt!(m, 0);
                m_max = m_max.max(m);
                for (c, k) in (0..self.table.n_channels).cartesian_product(0..m) {
                    a_k[c * self.table.m_max + k] += weight * ap[c * m + k];
                }
            }
        }

        // Importance samples the luminance Fourier expansion.
        let (y, phi, pdf_phi) = match m_max {
            0 => (0.0, u * 2.0 * PI as f32, Prob::Density(FRAC_1_PI as f32)),
            _ => sample_fourier(&a_k[..m_max], &self.table.recip, u),
        };
        let pdf = (pdf_phi.density() * pdf_mu).max(0.0);

        // Computes the scattered direction for Fourier BSDF.
        let sin2_theta_i = (1.0 - mu_i * mu_i).max(0.0);
        let norm = f32::sqrt(sin2_theta_i / wo.sin2_theta()).fallback_if(f32::is_infinite, 0.0);
        let (sin_phi, cos_phi) = phi.sin_cos();
        let wi = -Omega::normalize(
            norm * (cos_phi * wo.x() - sin_phi * wo.y()),
            norm * (sin_phi * wo.x() + cos_phi * wo.y()),
            mu_i,
        );

        // Evaluates remaining fourier exapnsions for angle phi.
        let scale = 1.0f32.try_divide(mu_i.abs()).unwrap_or(0.0);
        if mu_i * mu_o > 0.0
        /* && mode == TransportMode::Radiance */
        {
            todo!()
        }

        let refl = if self.table.n_channels == 1 {
            Color::gray(y * scale)
        } else {
            let (r_start, g_start) = (self.table.m_max, self.table.m_max * 2);
            let r = fourier_sum(&a_k[r_start..r_start + m_max], cos_phi);
            let b = fourier_sum(&a_k[g_start..g_start + m_max], cos_phi);
            let g = 1.39829 * y - 0.100913 * b - 0.297375 * r;
            Color::new(r * scale, g * scale, b * scale)
        };
        (refl, wi, Prob::Density(pdf))
    }

    fn prob(&self, wo: Omega, wi: Omega) -> math::prob::Prob {
        let mu_i = (-wi).cos_theta();
        let mu_o = wo.cos_theta();
        let cos_phi = wo.cos_dphi(-wi);

        let wt_offset_i = self.table.get_weights_and_offset(mu_i);
        let wt_offset_o = self.table.get_weights_and_offset(mu_o);
        if wt_offset_i.is_none() || wt_offset_o.is_none() {
            return Prob::Density(0.0);
        }
        // Determines offsets and weights for mu_i and mu_o.
        let (offset_i, weights_i) = wt_offset_i.unwrap();
        let (offset_o, weights_o) = wt_offset_o.unwrap();

        let mut ak = vec![0.0; self.table.m_max];
        let mut order_max = 0;
        for (i, o) in (0..4).cartesian_product(0..4) {
            let weight = weights_i[i] * weights_o[o];
            if weight == 0.0 {
                continue;
            }
            let offset_i = (offset_i + i as isize) as usize;
            let offset_o = (offset_o + o as isize) as usize;
            let (coeffs, order) = self.table.get_ak(offset_i, offset_o);
            order_max = order_max.max(order);
            (0..order).for_each(|k| ak[k] += coeffs[k] * weight);
        }

        let rho = (0..4)
            .map(|o| {
                if weights_o[o] == 0.0 {
                    0.0
                } else {
                    let index =
                        (offset_o as usize + o) * self.table.mu.len() + self.table.mu.len() - 1;
                    weights_o[o] * self.table.cdf[index] * 2.0 * PI as f32
                }
            })
            .sum::<f32>();
        let y = fourier_sum(&ak[0..order_max], cos_phi).max(0.0);

        Prob::Density(y.try_divide(rho).unwrap_or(0.0))
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    use crate::bxdf::*;
    use math::{float::Float, hcm::vec3};
    use rand::Rng;
    #[test]
    fn fourier_sum_test() {
        let a = rand::random::<[f32; 15]>();
        for _ in 0..680 {
            let cos_phi = (rand::random::<f32>() * 2.0 - 1.0).clamp(-1.0, 1.0);
            let phi = cos_phi.acos();
            assert!(phi.is_finite());
            let expected = (0..a.len())
                .map(|k| a[k] * (phi * k as f32).cos())
                .sum::<f32>();
            let actual = fourier_sum(&a, cos_phi);

            assert!(actual.dist_to(expected) < 1e-5, "{}, {}", actual, expected);
        }
    }

    #[test]
    fn read_header_test() {
        let buffer = [
            0x53, 0x43, 0x41, 0x54, 0x46, 0x55, 0x4e, 0x01, // identifier and version
            0x01, 0x00, 0x00, 0x00, // flags
            0x54, 0x03, 0x00, 0x00, // n_mu
            0xd6, 0xb4, 0x73, 0x01, // n_coeffs
            0x3f, 0x06, 0x00, 0x00, // m_max
            0x03, 0x00, 0x00, 0x00, // num_channels
            0x01, 0x00, 0x00, 0x00, // n_bases
            0x00, 0x00, 0x00, 0x00, // metadata bytes
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // num parameters and parameters
            0x00, 0x00, 0x80, 0x3f, // eta
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // alpha * 2
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // unused
        ];

        let header = read_header(&buffer).unwrap();
        let n_mu = header.n_mu;
        assert_eq!(n_mu, 0x0354);
    }

    #[test]
    fn read_fourier_bsdf_test() {
        let path = "../assets/paint.bsdf";
        let table = FourierTable::from_file(path).unwrap();
        println!("mu = {:?}", table.mu);
        // println!("a_offset = {:?}", table.a_offset);
        // println!("m_lookup = {:?}", table.m_lookup);
    }

    #[test]
    fn fourier_bsdf_use_test() {
        let path = "../assets/paint.bsdf";
        let f_mtl = FourierTable::from_file(path).unwrap();

        let fourier_bsdf = FourierBSDF { table: &f_mtl };

        let (wos, (_dtheta, _dphi)) = Omega::tesselate_hemi(30);
        let mut pr_sum = 0.0;
        let mut rho = Color::black();
        for wo in wos {
            let (refl, wi, _pr) = fourier_bsdf.sample(wo, (0.5, 0.5));
            assert!(!refl.has_nan());
            assert!(!wi.0.has_nan());
            assert!(_pr.is_density());
            pr_sum += _pr.density();
            rho += refl * wi.cos_theta().abs() * _pr.density().weak_recip();
        }
        assert!(!rho.has_nan());
        println!("{}", pr_sum / ((30 * 30 * 4) as f32) * PI as f32);

        let wo = Omega::normal();
        let us = math::float::linspace((0.0, 1.0), 10).0;

        rho = Color::black();
        for (&u, &v) in us.iter().cartesian_product(us.iter()) {
            let (refl, wi, pr) = fourier_bsdf.sample(wo, (u, v));
            rho += refl * wi.cos_theta().abs() * pr.density().weak_recip();
        }
        rho = rho.scale_down_by(us.len().pow(2) as u32);
        println!("MC integrate for normal reflectance = {}", rho);

        // Samples over the hemisphere, used as wo, and MC-integrate the Li using random samples.
        let us = math::float::linspace((0.0, 1.0), 10).0;
        let mut rng = rand::thread_rng();
        for (&u, &v) in us.iter().cartesian_product(us.iter()) {
            let wo = cos_sample_hemisphere((u, v));
            let refl = (0..100)
                .map(|_| {
                    let wi_uv = rng.gen::<(f32, f32)>();
                    let (f, wi, pr) = fourier_bsdf.sample(wo, wi_uv);

                    f * wi.cos_theta().abs() * pr.density().weak_recip()
                })
                .sum::<Color>()
                .scale_down_by(100);
            assert!(!refl.has_nan());
        }
    }

    #[test]
    fn fourier_bsdf_simple_test() {
        let path = "../assets/paint.bsdf";
        let f_mtl = FourierTable::from_file(path).unwrap();

        let fourier_bsdf = FourierBSDF { table: &f_mtl };

        let wo = Omega(vec3(-0.255834639, -0.200433612, 0.945713997));
        let wi = Omega(vec3(0.194454342, -0.194454342, 0.961444259));
        let f = fourier_bsdf.eval(wo, wi);
        let pdf = fourier_bsdf.prob(wo, wi);

        let actual_f = vec3(f.r, f.g, f.b);
        let expected_f = vec3(0.1474448, 0.1474451, 0.1474448);

        assert!((actual_f - expected_f).norm() < f32::EPSILON);
        assert!(pdf.density().dist_to(0.3609094) < f32::EPSILON);

        let (f, wi, pr) = fourier_bsdf.sample(wo, (0.38, 0.78));
        let actual_f = vec3(f.r, f.g, f.b);
        let expected_f = vec3(0.110911831, 0.110912055, 0.110911831);
        assert!((actual_f - expected_f).norm() < f32::EPSILON);
        assert!(pr.density().dist_to(0.14557238) < f32::EPSILON);
        let expected_wi = vec3(-0.851179481, 0.0985863954, 0.51553309);
        assert!((expected_wi - wi.0).norm() < f32::EPSILON * 3.0);
    }
}
