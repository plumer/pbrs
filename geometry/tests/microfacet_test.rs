use rand::Rng;
use std::f32::consts::PI;

use geometry::{
    bxdf::{self, BxDF, Omega},
    microfacet as mf,
};
use math::float::{linspace, Float, Inside};
use radiometry::color::Color;

/// Tests if the differential area distribution function `d()` integrates to 1 over the hemisphere.
#[test]
fn diff_area_validate() {
    let beck_mf = mf::MicrofacetDistrib::beckmann(0.2, 0.2);
    let trowbridge_mf = mf::MicrofacetDistrib::trowbridge_reitz(0.2, 0.2);
    for mf in [beck_mf, trowbridge_mf].iter() {
        let projected_area = integrate_differental_area(*mf);
        println!("Differential projected area = {}", projected_area);
        assert!((projected_area - 1.0).abs() < 1e-3);
        let w = Omega::new(0.48, 0.64, 0.6);
        let masked_area = integrate_masking(*mf, w);
        println!("Differential masked area = {}", masked_area);
        assert!((masked_area - w.cos_theta()).abs() < 1e-3);
    }
}

#[test]
fn pdf_integral_validate() {
    let (alphas, _) = linspace((0.1, 0.9), 8);

    for alpha in alphas.iter().copied() {
        println!("Testing alpha = {}", alpha);

        let beck_mf = mf::MicrofacetDistrib::beckmann(alpha, alpha);
        let trowbridge_mf = mf::MicrofacetDistrib::trowbridge_reitz(alpha, alpha);
        let pdf_integral: f32 = integrate_wo_wh(&beck_mf);
        assert!(
            pdf_integral.dist_to(1.0) < 2e-3,
            "actually {}",
            pdf_integral
        );
        let pdf_integral: f32 = integrate_wo_wh(&trowbridge_mf);
        assert!(
            pdf_integral.dist_to(1.0) < 2e-3,
            "actually {}",
            pdf_integral
        );
    }
}

#[test]
fn play_integrate_wo_wh() {
    let mf = mf::MicrofacetDistrib::beckmann(0.3, 0.3);
    for count in [30, 50, 70, 80].iter() {
        let (whs, (d_theta, d_phi)) = Omega::tesselate_hemi(*count);
        let marginal_p_o = whs
            .iter()
            .map(|wh| mf.pdf(Omega::normal(), *wh) * wh.sin_theta() * d_theta * d_phi)
            .sum::<f32>();
        println!(
            "count = {}, marginal pdf integral = {}",
            count, marginal_p_o
        );
    }
}

/// Integrates the PDF of the given microfact distribution over wo and wh hemispheres.
fn integrate_wo_wh(mf: &mf::MicrofacetDistrib) -> f32 {
    let (whs, (d_theta, d_phi)) = Omega::tesselate_hemi(70);
    let (wos, _) = Omega::tesselate_hemi(3);
    let full_pdf_integral = wos
        .iter()
        .map(|wo| {
            let marginal_p_o = whs
                .iter()
                .map(|wh| mf.pdf(*wo, *wh) * wh.sin_theta() * d_theta * d_phi)
                .sum::<f32>();
            marginal_p_o
        })
        .sum::<f32>();
    full_pdf_integral / wos.len() as f32
}

#[allow(dead_code)]
fn mc_integrate_wo_wh(mf: &mf::MicrofacetDistrib) -> f32 {
    let mut rng = rand::thread_rng();
    let num_samples = 20000;
    let volume = PI * 2.0;
    (0..num_samples)
        .map(|_| {
            // let uv = rng.gen::<(f32, f32)>();
            // let wo = bxdf::cos_sample_hemisphere(uv);
            let wo = Omega::normal();
            let uv = rng.gen::<(f32, f32)>();
            let wh = bxdf::cos_sample_hemisphere(uv);
            mf.pdf(wo, wh)
        })
        .sum::<f32>()
        * volume
        / num_samples as f32
}

/// Integrates the differential area distribution function - `d()` - over the hemisphere.
fn integrate_differental_area(mf_distrib: mf::MicrofacetDistrib) -> f32 {
    const N: i32 = 100;
    let mut integral_area = 0.0;
    let (thetas, d_theta) = linspace((0.0, std::f32::consts::FRAC_PI_2), N);
    let (phis, d_phi) = linspace((0.0, std::f32::consts::PI * 2.0), N * 4);
    for theta in thetas.into_iter() {
        for phi in phis.iter().copied() {
            let (sin_theta, cos_theta) = theta.sin_cos();
            let wh = math::hcm::spherical_direction(sin_theta, cos_theta, math::new_rad(phi));
            let diff_area = mf_distrib.d(Omega(wh));
            assert!(!diff_area.is_infinite());
            integral_area += diff_area * cos_theta * (sin_theta * d_theta * d_phi);
        }
    }
    integral_area
}

fn integrate_masking(mf_distrib: mf::MicrofacetDistrib, w: Omega) -> f32 {
    const N: i32 = 100;
    let mut integral_masked_area = 0.0;
    let (thetas, d_theta) = linspace((0.0, std::f32::consts::FRAC_PI_2), N);
    let (phis, d_phi) = linspace((0.0, std::f32::consts::PI * 2.0), N * 4);
    for theta in thetas.into_iter() {
        for phi in phis.iter().copied() {
            let (sin_theta, cos_theta) = theta.sin_cos();
            let wh = math::hcm::spherical_direction(sin_theta, cos_theta, math::new_rad(phi));
            let diff_area = mf_distrib.d(Omega(wh));
            let masked_area = diff_area * mf_distrib.g1(w) * w.dot(Omega(wh)).max(0.0);
            integral_masked_area += masked_area * (sin_theta * d_theta * d_phi);
        }
    }
    integral_masked_area
}

#[test]
fn observe_normals() {
    let mf = mf::MicrofacetDistrib::beckmann(0.1, 0.1);
    use rand::Rng;
    let mut rng = rand::thread_rng();

    let wo = Omega::normalize(0.8, 0.6, 0.1);
    let mut phis = vec![];
    let mut thetas = vec![];
    let mut whs = vec![];
    for _ in 0..100 {
        let u = rng.gen::<f32>();
        let v = rng.gen::<f32>();
        let wh = mf.sample_wh(wo, (u, v));
        let phi = f32::atan2(wh.y(), wh.x());
        let cos_theta = wh.cos_theta();
        phis.push(phi);
        thetas.push(cos_theta.acos());
        whs.push(wh);
    }

    for wh in whs.iter() {
        let forward = wh.dot(wo).signum() * 0.25 + 0.5;
        println!("{}, {}, {}, {}", wh.x(), wh.y(), wh.z(), forward);
    }
}

#[test]
fn beckmann_rho() {
    use rand::Rng;
    let mut rng = rand::thread_rng();
    for alpha in [0.05f32, 0.1, 0.3, 0.6].iter().copied() {
        let (norm_mean, norm_stdev) = {
            let mf = mf::MicrofacetDistrib::beckmann(alpha, alpha);
            let mf_refl = bxdf::MicrofacetReflection::new(Color::white(), mf, bxdf::Fresnel::Nop);

            let wo = Omega::normalize(0.2, 0.6, 0.5);
            let mut norms = vec![];
            for _ in 0..500 {
                let (color, wi, pr) = mf_refl.sample(wo, rng.gen::<(f32, f32)>());
                let color = color * wi.cos_theta().abs();
                let cv = math::hcm::Vec3::new(color.r, color.g, color.b);
                if let Some(norm) = cv.norm().try_divide(pr.density()) {
                    norms.push(norm);
                }
            }

            let norm_mean = norms.iter().sum::<f32>() / norms.len() as f32;
            let norm_variance =
                norms.iter().map(|n| (n - norm_mean).powi(2)).sum::<f32>() / norms.len() as f32;
            (norm_mean, norm_variance)
        };

        assert!((3.0f32.sqrt()).inside((norm_mean - norm_stdev, norm_mean + norm_stdev)));
        assert!((norm_stdev / alpha).inside((0.0, 2.0)), "{} / {}", norm_stdev, alpha);
    }
}
