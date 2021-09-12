use geometry::bxdf::{self, BxDF, Fresnel};
use math::hcm::Vec3;
use radiometry::color::Color;

fn f32_close(a: f32, b: f32) -> bool {
    b / a > 0.999 && b / a < 1.001
}

#[test]
fn local_trigonometry_test() {
    let local_w = math::hcm::Vec3::new(0.64, 0.48, 0.6);
    assert_eq!(bxdf::local::cos_theta(local_w), 0.6);
    assert_eq!(bxdf::local::cos2_theta(local_w), 0.36);
    assert_eq!(bxdf::local::sin2_theta(local_w), 0.64);
    assert_eq!(bxdf::local::sin_theta(local_w), 0.8);
    assert!(f32_close(bxdf::local::cos_phi(local_w), 0.8));
    assert!(f32_close(bxdf::local::sin_phi(local_w), 0.6));
}

#[test]
fn fresnel_test() {
    let glass = bxdf::FresnelDielectric::new(1.0, 2.0);
    let invert_glass = bxdf::FresnelDielectric::new(2.0, 1.0);

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
    let glass =
        bxdf::SpecularReflection::new(Color::white(), bxdf::FresnelDielectric::new(1.0, 2.0));
    let bsdf_value = glass.eval(Vec3::new(0.5, 0.5, 0.5), Vec3::new(0.2, -0.2, 0.7));
    assert!(bsdf_value.is_black());

    let (wi_local, pdf, bsdf_value) = glass.sample(Vec3::new(0.8, 0.0, 0.6), (0.0, 0.0));
    assert_eq!(wi_local.x, -0.8);
    assert_eq!(wi_local.y, -0.0);
    assert_eq!(wi_local.z, 0.6);
    assert!(matches!(pdf, bxdf::HemiPdf::Delta(_)));
    println!("bsdf value = {}", bsdf_value);
}

fn linspace(interval: (f32, f32), count: i32) -> (Vec<f32>, f32) {
    let (a, b) = interval;
    (
        (0..count)
            .into_iter()
            .map(|i| (i as f32 + 0.5) / count as f32 * (b - a) + a)
            .collect::<Vec<_>>(),
        (b - a) / count as f32,
    )
}

#[test]
fn pdf_test() {
    let albedo = Color::new(0.4, 0.5, 0.7);
    let matte = bxdf::LambertianReflection::new(albedo);
    assert!(f32_close(riemann_integral_pdf(&matte), 1.0));
    let stochastic_rho = montecarlo_integrate_rho(&matte);
    assert!(
        color_is_close(albedo, stochastic_rho),
        "Lambertian: albedo = {}, stochastic rho = {}",
        albedo,
        stochastic_rho
    );

    let oren_nayar = bxdf::OrenNayar::new(albedo, math::hcm::Degree(0.0));
    let stochastic_rho = montecarlo_integrate_rho(&oren_nayar);
    println!(
        "Oren-Nayar: albedo = {}, stochastic rho = {}",
        albedo, stochastic_rho
    );
}

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

fn riemann_integral_pdf<BSDF: BxDF>(bsdf: &BSDF) -> f32 {
    let mut pdf_integral = 0.0;
    const N: i32 = 100;
    let (thetas, d_theta) = linspace((0.0, std::f32::consts::FRAC_PI_2), N);
    let (phis, d_phi) = linspace((0.0, std::f32::consts::PI * 2.0), N * 4);
    for theta in thetas.into_iter() {
        for phi in phis.iter().copied() {
            let (sin_theta, cos_theta) = theta.sin_cos();
            let wi = math::hcm::spherical_direction(sin_theta, cos_theta, math::hcm::Radian(phi));
            let pdf = bsdf.pdf(Vec3::new(0.48, 0.64, 0.6), wi);

            pdf_integral += pdf * sin_theta * d_theta * d_phi;
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
                let (wi, pdf, bsdf_value) = bsdf.sample(Vec3::new(0.4, 0.5, 0.7), (u, v));
                if let bxdf::HemiPdf::Regular(pdf) = pdf {
                    bsdf_value * bxdf::local::cos_theta(wi).abs() / pdf
                } else {
                    panic!()
                }
            })
            .fold(Color::black(), |c0, c1| c0 + c1)
}
