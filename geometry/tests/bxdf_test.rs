use std::f32::consts::FRAC_1_PI;

use geometry::bxdf::{self, BxDF, MicrofacetReflection, Omega};
use geometry::microfacet::MicrofacetDistrib;
use math::float::{linspace, Float};
use math::hcm::Vec3;
use math::prob::Prob;
use radiometry::color::Color;

fn f32_close(a: f32, b: f32) -> bool {
    b / a > 0.999 && b / a < 1.001
}

#[test]
#[rustfmt::skip]
fn local_trigonometry_test() {
    let w = Omega::new(0.64, 0.48, 0.6);
    assert_eq!(w.cos_theta(), 0.6);
    assert_eq!(w.cos2_theta(), 0.36);
    assert_eq!(w.sin2_theta(), 0.64);
    assert_eq!(w.sin_theta(), 0.8);
    assert!(f32_close(w.cos_phi(), 0.8));
    assert!(f32_close(w.sin_phi(), 0.6));
    assert!(f32_close(w.cos2_phi(), 0.64), "actual value = {}", w.cos2_phi());
    assert!(f32_close(w.sin2_phi(), 0.36), "actual value = {}", w.sin2_phi());
}

#[test]
fn fresnel_test() {
    let glass = bxdf::Fresnel::dielectric(1.0, 2.0);
    let invert_glass = bxdf::Fresnel::dielectric(2.0, 1.0);

    let cos_thetas = [0.3f32, 0.9];
    let expected_forward_values = [0.26872247f32, 0.112083375];
    let expected_inverted_values = [1.0f32, 0.1645631];
    for i in 0..cos_thetas.len() {
        let cos_theta_i = cos_thetas[i];
        let expected_forward_value = expected_forward_values[i];
        let expected_inverted_value = expected_inverted_values[i];

        let actual_forward_value = glass.refl_coeff(cos_theta_i);
        let actual_inverted_value = invert_glass.refl_coeff(cos_theta_i);
        assert_eq!(actual_forward_value, expected_forward_value);
        assert_eq!(actual_inverted_value, expected_inverted_value);

        assert_eq!(actual_forward_value, invert_glass.refl_coeff(-cos_theta_i));
        assert_eq!(actual_inverted_value, glass.refl_coeff(-cos_theta_i));
    }
}

#[test]
fn specular_refl_test() {
    let glass = bxdf::Specular::dielectric(Color::white(), 1.0, 2.0);

    let (bsdf_value, wi, pdf) = glass.sample(Omega::new(0.8, 0.0, 0.6), (0.0, 0.0));
    assert_eq!(wi.x(), -0.8);
    assert_eq!(wi.y(), -0.0);
    assert_eq!(wi.z(), 0.6);
    assert!(matches!(pdf, Prob::Mass(_)));
    println!("bsdf value = {}", bsdf_value);
}

#[test]
fn diffuse_refl_test() {
    let albedo = Color::new(1.0, 2.0, 5.0);
    let matte = bxdf::DiffuseReflect::lambertian(albedo);
    let oren_nayar = bxdf::DiffuseReflect::oren_nayar(albedo, math::new_deg(0.0));
    test_one_diffuse_brdf(&matte, albedo);
    test_one_diffuse_brdf(&oren_nayar, albedo);
}

// TODO: make this test pass #[test]
#[allow(dead_code)]
fn microfacet_refl_test() {
    let albedo = Color::new(0.9, 0.8, 0.6);
    let (alphas, _) = linspace((0.1, 0.9), 8);
    for alpha in alphas.into_iter() {
        let mf_refl = bxdf::MicrofacetReflection::new(
            albedo,
            MicrofacetDistrib::beckmann(alpha, alpha),
            bxdf::Fresnel::dielectric(1.0, 1.2),
        );
        let pdf_hemisphere_integral = riemann_integral_pdf_2d(&mf_refl);
        if (pdf_hemisphere_integral - 1.0).abs() > 1e-2 {
            eprintln!(
                "alpha = {}, Hemisphere pdf doesn't integrate to 1.0 ({} instead)",
                alpha, pdf_hemisphere_integral
            );
        }
        if false {
            let mc_rho = montecarlo_integrate_rho(&mf_refl);
            assert!(
                color_is_close(albedo, mc_rho),
                "Monte-carlo integrated rho doesn't euqal to albedo: {} vs {}",
                albedo,
                mc_rho
            );
        }
    }
}

// Utility functions.
// ----------------------------------------------------------------------------

fn color_is_close(c0: Color, c1: Color) -> bool {
    let v0 = Vec3::new(c0.r, c0.g, c0.b);
    let v1 = Vec3::new(c1.r, c1.g, c1.b);
    if v0.is_zero() || v1.is_zero() {
        (v0 - v1).norm_squared() < 1e-6
    } else {
        let longer_length = v0.norm_squared().max(v1.norm_squared());
        (v0 - v1).norm_squared() / longer_length < 1e-3
    }
}

fn test_one_diffuse_brdf(brdf: &bxdf::DiffuseReflect, albedo: Color) {
    let pdf_hemisphere_integral = riemann_integral_pdf(brdf);
    assert!(
        (pdf_hemisphere_integral - 1.0).abs() < 1e-3,
        "Hemisphere pdf doesn't integrate to 1.0 ({} instead)",
        pdf_hemisphere_integral
    );

    let pdf_hemisphere_integral = riemann_integral_pdf_2d(brdf);
    assert!(
        (pdf_hemisphere_integral - 1.0).abs() < 4e-3,
        "2D Hemisphere pdf doesn't integrate to 1.0 ({} instead)",
        pdf_hemisphere_integral
    );

    let mc_rho = montecarlo_integrate_rho(brdf);
    assert!(
        color_is_close(albedo, mc_rho),
        "Monte-carlo integrated rho doesn't euqal to albedo: {} vs {}",
        albedo,
        mc_rho
    );
}

fn riemann_integral_pdf<BSDF: BxDF>(bsdf: &BSDF) -> f32 {
    riemann_integral_hemi_pdf_i(bsdf, Omega::normalize(0.48, 0.64, 0.6), 25)
}

fn riemann_integral_hemi_pdf_i<BSDF: BxDF>(bsdf: &BSDF, wo: Omega, count: i32) -> f32 {
    let (wis, (d_theta, d_phi)) = Omega::tesselate_hemi(count);
    let pdf_integral = wis
        .into_iter()
        .map(|wi| {
            let pr = bsdf.prob(wo, wi);
            assert!(pr.is_density(), "bxdf returns a mass value");
            pr.density() * wi.sin_theta() * d_theta * d_phi
        })
        .sum();

    pdf_integral
}

fn riemann_integral_pdf_2d<BSDF: BxDF>(bsdf: &BSDF) -> f32 {
    let mut pdf_integral = 0.0;
    const N: i32 = 20;
    let (thetas, d_theta) = linspace((0.0, std::f32::consts::FRAC_PI_2), N);
    let (phis, d_phi) = linspace((0.0, std::f32::consts::PI * 2.0), N * 4);

    for theta in thetas.into_iter() {
        for phi in phis.iter().copied() {
            let (sin_theta, cos_theta) = theta.sin_cos();
            let wo = math::hcm::spherical_direction(sin_theta, cos_theta, math::new_rad(phi));
            let marginal_pdf_o = riemann_integral_hemi_pdf_i(bsdf, Omega(wo), N);

            pdf_integral += marginal_pdf_o * sin_theta * d_theta * d_phi;
        }
    }

    // For a perfect lambertian reflection, integrate(p(wo, wi), d(wi)) = 1, and
    // integrate(integrate(p(wo, wi), d(wi)), d(wo)) = 2pi. So for any other things, "average" of
    // integrate(p(wo, wi), d(wi)) for all wo's should be 2pi as well.

    pdf_integral * FRAC_1_PI * 0.5
}

fn montecarlo_integrate_rho<BSDF: BxDF>(bsdf: &BSDF) -> Color {
    use rand::Rng;
    let mut rng = rand::thread_rng();
    const TRIALS: i32 = 800;
    (1.0 / TRIALS as f32)
        * (0..TRIALS)
            .map(|_| {
                let u = rng.gen::<f32>();
                let v = rng.gen::<f32>();
                let wo = Omega::normalize(0.2, -0.1, 0.9);
                let (bsdf_value, wi, pr) = bsdf.sample(wo, (u, v));
                assert!(!wi.0.has_nan());
                if let Prob::Density(pdf) = pr {
                    bsdf_value * wi.cos_theta().abs() * pdf.weak_recip()
                } else {
                    panic!()
                }
            })
            .fold(Color::black(), |c0, c1| c0 + c1)
}

#[test]
fn play_with_mf_brdf() {
    let albedo = Color::new(3.0, 3.4, 2.9);
    let mf = MicrofacetDistrib::beckmann(0.2, 0.3);
    let brdf = MicrofacetReflection::new(albedo, mf, bxdf::Fresnel::Nop);

    let wo = Omega::normalize(0.6, 0.8, 0.3);
    assert!((wo.0.norm_squared() - 1.0).abs() < 1e-3);
    use rand::Rng;
    let mut rng = rand::thread_rng();
    for _ in 0..10 {
        let u = rng.gen::<f32>();
        let v = rng.gen::<f32>();
        let wh_from_mf = mf.sample_wh(wo, (u, v));
        let (fval, wi_from_brdf, _pdf) = brdf.sample(wo, (u, v));

        if fval.is_black() {
            continue;
        }
        if let Some(wh_from_bisector) = Omega::bisector(wo, wi_from_brdf) {
            let dist_squared = (wh_from_bisector.0 - wh_from_mf.0).norm_squared();
            assert!(dist_squared < 1e-3,
            "dist_squared = {}, wh from mf_distrib: {:.4}; wh from bisector: {:.4}, wo = {}, wi = {}",
            dist_squared,
            wh_from_mf.0,
            wh_from_bisector.0, wo.0, wi_from_brdf.0
        );
        }
    }
}
