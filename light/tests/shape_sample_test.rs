use std::f32::consts::PI;

use geometry::Interaction;
use light::ShapeSample;
use math::float::Float;
use math::hcm::{point3, vec3, Point3, Vec3};
use shape::Sphere;

#[test]
fn test_sphere_sample_pdf_integrate() {
    let s = Sphere::from_raw((5.0, 6.0, 12.0), 2.0);
    let p = Interaction::rayless(Point3::new(0.0, 0.0, 0.0), (0.0, 0.0), Vec3::Z);

    let pdf_integral = integrate_sphere_pdf(&s, &p);
    assert!(
        pdf_integral.dist_to(1.0) < 1e-2,
        "Int(pdf) = {}",
        pdf_integral
    );
}

#[test]
fn observe_sphere_sample_towards() {
    use shape::Shape;
    let s = Sphere::new(Point3::ORIGIN, 1.5);
    let target = Interaction::rayless(point3(0.0, 3.0, 0.0), (0.0, 0.0), vec3(0.6, -0.8, 0.0));
    if true {
        let s = Sphere::new(point3(9.44999981, 20.0, -8.0), 0.2);
        let target = Interaction::new(
            point3(-18.3287563, 19.3762169, 0.0),
            56.0,
            (0.0417810902, 0.968810856),
            -math::hcm::Vec3::Z,
            vec3(0.327299207, -0.203146726, -1.0),
        )
        .with_dpdu(vec3(0.0, -1.0, 0.0));

        let rnd2 = (0.9898101, 0.724872649);
        s.sample_towards(&target, rnd2);
    }
    let (uvec, _) = math::float::linspace((0.0f32, 1.0), 10);
    let vvec = uvec.clone();
    for u in uvec.iter() {
        for v in vvec.iter() {
            let point_on_sphere = s.sample_towards(&target, (*u, *v));

            let radial = point_on_sphere.pos - s.center();
            assert!(radial.norm_squared().dist_to(s.radius().powi(2)) < 1e-3);
            assert!(
                point_on_sphere.normal.cross(radial).norm_squared() < 1e-3,
                "radial = {}, normal = {}",
                radial,
                point_on_sphere.normal
            );

            let r = target.spawn_ray(point_on_sphere.pos - target.pos);
            let hit = s.intersect(&r).unwrap();
            assert!(
                hit.pos.squared_distance_to(point_on_sphere.pos) < 1e-1,
                "actual {} vs expected {}",
                hit.pos,
                point_on_sphere.pos
            );
        }
    }
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
