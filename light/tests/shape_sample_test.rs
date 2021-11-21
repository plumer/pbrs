use std::f32::consts::PI;

use light::ShapeSample;
use math::float::Float;
use math::hcm::{Point3, Vec3};
use shape::{Interaction, Sphere};

#[test]
fn test_sphere_sample_pdf_integrate() {
    let s = Sphere::from_raw((5.0, 6.0, 12.0), 2.0);
    let p = Interaction::rayless(Point3::new(0.0, 0.0, 0.0), (0.0, 0.0), Vec3::zbase());

    let pdf_integral = integrate_sphere_pdf(&s, &p);
    assert!(
        pdf_integral.dist_to(1.0) < 1e-2,
        "Int(pdf) = {}",
        pdf_integral
    );
}

/// Monte-Carlo integrates `pdf_at()` of a sphere. Should evaluate to 1.0.
fn integrate_sphere_pdf(sphere: &Sphere, target: &Interaction) -> f32 {
    let (uvec, _spacing) = math::float::linspace((0.0, 1.0), 20);
    let (vvec, _spacing) = math::float::linspace((0.0, 1.0), 20);

    let mut pdf_integral = 0.0;
    let cone_solid_angle = {
        let sin2_t = sphere.radius().powi(2) / target.pos.squared_distance_to(sphere.center());
        let cos_t = (1.0 - sin2_t).max(0.0).sqrt();
        2.0 * PI * (1.0 - cos_t)
    };
    for u in uvec.iter().cloned() {
        for v in vvec.iter().cloned() {
            let point_on_sphere = sphere.sample_towards(target, (u, v));
            let sampled_radius = point_on_sphere.pos.distance_to(sphere.center());
            assert!(sampled_radius.dist_to(sphere.radius()) < 1e-5);
            let wi = point_on_sphere.pos - target.pos;
            let pdf = sphere.pdf_at(&target, wi).unwrap();
            pdf_integral += pdf * cone_solid_angle;
        }
    }
    pdf_integral / (uvec.len() * vvec.len()) as f32
}
