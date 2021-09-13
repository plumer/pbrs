use geometry::microfacet as mf;
use math::float::linspace;

#[test]
fn diff_area_validate() {
    let beck_mf = mf::MicrofacetDistrib::beckmann(0.2, 0.2);
    let projected_area = integrate_differental_area(beck_mf);
    println!("Differential projected area = {}", projected_area);
    assert!((projected_area - 1.0).abs() < 1e-3);
    
    let trowbridge_mf = mf::MicrofacetDistrib::trowbridge_reitz(0.2, 0.2);
    let projected_area = integrate_differental_area(trowbridge_mf);
    println!("Differential projected area = {}", projected_area);
    assert!((projected_area - 1.0).abs() < 1e-3);
}

fn integrate_differental_area(mf_distrib: mf::MicrofacetDistrib) -> f32 {
    const N: i32 = 100;
    let mut integral_area = 0.0;
    let (thetas, d_theta) = linspace((0.0, std::f32::consts::FRAC_PI_2), N);
    let (phis, d_phi) = linspace((0.0, std::f32::consts::PI * 2.0), N * 4);
    for theta in thetas.into_iter() {
        for phi in phis.iter().copied() {
            let (sin_theta, cos_theta) = theta.sin_cos();
            let wh = math::hcm::spherical_direction(sin_theta, cos_theta, math::hcm::Radian(phi));
            let diff_area = mf_distrib.d(wh);
            assert!(!diff_area.is_infinite());
            integral_area += diff_area * cos_theta * (sin_theta * d_theta * d_phi);
        }
    }
    integral_area
}
