use geometry::bxdf::{self, BxDF, MicrofacetReflection, Omega, Prob};
use geometry::microfacet::MicrofacetDistrib;
use math::float::linspace;
use math::hcm::Vec3;
use radiometry::color::Color;

fn f32_close(a: f32, b: f32) -> bool {
    b / a > 0.999 && b / a < 1.001
}

#[test]
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
    assert!(matches!(pdf, bxdf::Prob::Mass(_)));
    println!("bsdf value = {}", bsdf_value);
}

#[test]
fn diffuse_refl_test() {
    let albedo = Color::new(1.0, 2.0, 5.0);
    let matte = bxdf::DiffuseReflect::lambertian(albedo);
    let oren_nayar = bxdf::DiffuseReflect::oren_nayar(albedo, math::hcm::Degree(0.0));
    test_one_diffuse_brdf(&matte, albedo);
    test_one_diffuse_brdf(&oren_nayar, albedo);

    // let mf_refl = bxdf::MicrofacetReflection::new(
    //     albedo,
    //     MicrofacetDistrib::beckmann(0.2, 0.2),
    //     bxdf::Fresnel::dielectric(1.0, 1.2),
    // );
    // test_one_diffuse_brdf(&mf_refl, albedo);
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

    let mc_rho = montecarlo_integrate_rho(brdf);
    assert!(
        color_is_close(albedo, mc_rho),
        "Monte-carlo integrated rho doesn't euqal to albedo: {} vs {}",
        albedo,
        mc_rho
    );
}

fn riemann_integral_pdf(bsdf: &bxdf::DiffuseReflect) -> f32 {
    let mut pdf_integral = 0.0;
    const N: i32 = 100;
    let (thetas, d_theta) = linspace((0.0, std::f32::consts::FRAC_PI_2), N);
    let (phis, d_phi) = linspace((0.0, std::f32::consts::PI * 2.0), N * 4);
    for theta in thetas.into_iter() {
        for phi in phis.iter().copied() {
            let (sin_theta, cos_theta) = theta.sin_cos();
            let wi = math::hcm::spherical_direction(sin_theta, cos_theta, math::hcm::Radian(phi));
            let pr = bsdf.prob(Omega::new(0.48, 0.64, 0.6), Omega(wi));

            if let Prob::Density(pdf) = pr {
                pdf_integral += pdf * sin_theta * d_theta * d_phi;
            } else {
                panic!("bxdf returns a prob-mass value");
            }
        }
    }
    pdf_integral
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
                if let bxdf::Prob::Density(pdf) = pr {
                    // println!("pdf = {}", pdf);
                    if pdf == 0.0 {
                        Color::black()
                    } else {
                        bsdf_value * wi.cos_theta().abs() / pdf
                    }
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
