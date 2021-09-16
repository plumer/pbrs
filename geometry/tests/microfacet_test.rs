use geometry::{bxdf::Omega, microfacet as mf};
use math::{float::linspace};

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

/// Integrates the differential area distribution function - `d()` - over the hemisphere.
fn integrate_differental_area(mf_distrib: mf::MicrofacetDistrib) -> f32 {
    const N: i32 = 100;
    let mut integral_area = 0.0;
    let (thetas, d_theta) = linspace((0.0, std::f32::consts::FRAC_PI_2), N);
    let (phis, d_phi) = linspace((0.0, std::f32::consts::PI * 2.0), N * 4);
    for theta in thetas.into_iter() {
        for phi in phis.iter().copied() {
            let (sin_theta, cos_theta) = theta.sin_cos();
            let wh = math::hcm::spherical_direction(sin_theta, cos_theta, math::hcm::Radian(phi));
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
            let wh = math::hcm::spherical_direction(sin_theta, cos_theta, math::hcm::Radian(phi));
            let diff_area = mf_distrib.d(Omega(wh));
            let masked_area = diff_area * mf_distrib.g1(w) * w.dot(Omega(wh)).max(0.0);
            integral_masked_area += masked_area * (sin_theta * d_theta * d_phi);
        }
    }
    integral_masked_area
}

#[test]
fn observe_normals() {
    let mf = mf::MicrofacetDistrib::beckmann(0.9, 0.9);
    use rand::Rng;
    let mut rng = rand::thread_rng();

    let mut phis = vec![];
    let mut thetas = vec![];
    let mut whs = vec![];
    for _ in 0..100 {
        let u = rng.gen::<f32>();
        let v = rng.gen::<f32>();
        let wh = mf.sample_wh(Omega::normalize(0.8, 0.6, 0.1), (u, v));
        let phi = f32::atan2(wh.y(), wh.x());
        let cos_theta = wh.cos_theta();
        phis.push(phi);
        thetas.push(cos_theta.acos());
        whs.push(wh);
    }

    println!("x = {:?};", whs.iter().map(|w| w.x()).collect::<Vec<_>>());
    println!("y = {:?};", whs.iter().map(|w| w.y()).collect::<Vec<_>>());
    println!("z = {:?};", whs.iter().map(|w| w.z()).collect::<Vec<_>>());
}
