use geometry::ray;
use geometry::ray::Ray;
use geometry::Interaction;
use math::assert_le;
use math::float::Float;
use math::hcm::{self, point3, vec3, Point3, Vec3};
use shape::Shape;

#[test]
fn quad_frame_test() {
    let quad_xy = shape::ParallelQuad::new_xy((-1.0, 1.0), (-1.0, 1.0), 0.0);
    let ray = ray::Ray::new(Point3::new(0.5, 0.5, -1.0), Vec3::new(-0.2, -0.2, 1.0));
    let isect = quad_xy.intersect(&ray).unwrap();
    assert!(isect.has_valid_frame());
}

#[test]
fn custom_frame_test() {
    let normal = hcm::vec3(-0.3, 0.5, 1.0).hat();
    let (dpdu, dpdv) = hcm::make_coord_system(normal);
    assert!(
        normal.dot(dpdu).abs() < 1e-4,
        "normal and tangent not perp: {} dot {} = {}",
        normal,
        dpdu,
        normal.dot(dpdu)
    );
    assert!(
        normal.dot(dpdv).abs() < 1e-4,
        "normal and bitangent not perp: {} dot {} = {}",
        normal,
        dpdv,
        normal.dot(dpdv)
    );

    let frame = hcm::Mat3::from_cols(dpdu, dpdv, normal);
    assert!((frame * frame.transpose() - hcm::Mat3::IDENTITY).frobenius_norm_squared() < 1e-6);

    let isect =
        Interaction::rayless(hcm::Point3::new(3.0, 2.5, 2.0), (0.2, 0.8), normal).with_dpdu(dpdu);

    let actual_tangent = isect.tangent();
    let actual_normal = isect.normal;

    assert!(
        (actual_tangent - dpdu).norm_squared() < 1e-6,
        "tangent mismatch, {} (actual) vs. {} (expected)",
        actual_tangent,
        dpdu
    );
    assert!((actual_normal - normal).norm_squared() < 1e-6);
}

#[test]
fn sphere_test() {
    let sphere = shape::Sphere::new(point3(3.0, 4.0, 5.0), 1.6);
    let dir_0 = vec3(1.5, 2.0, 2.5);
    for s in [0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0] {
        let r = Ray::new(point3(0.1, 0.2, 0.1), dir_0 * s).with_extent(1.0 / s);
        assert!(sphere.intersect(&r).is_none(), "ray = {:.5}", r);
        assert!(!sphere.occludes(&r));
    }

    let dir_1 = vec3(3.0, 4.0, 5.0);
    for s in [0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0] {
        let r = Ray::new(point3(0.1, 0.2, 0.1), dir_1 * s).with_extent(1.0 / s);
        assert!(sphere.intersect(&r).is_some(), "ray = {:.5}", r);
        assert!(sphere.occludes(&r));
        let isect_pos = sphere.intersect(&r).unwrap().pos;
        let dist2 = isect_pos.squared_distance_to(sphere.center());
        let radius2 = sphere.radius().powi(2);
        assert_le!(dist2.dist_to(radius2), 1e-4);
    }

    let dir_2 = vec3(4.8, 6.4, 8.0);
    for s in [0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0] {
        let r = Ray::new(point3(0.1, 0.2, 0.1), dir_2 * s).with_extent(1.0 / s);
        assert!(sphere.intersect(&r).is_some(), "ray = {:.5}", r);
        assert!(sphere.occludes(&r));
        let isect_pos = sphere.intersect(&r).unwrap().pos;
        let dist2 = isect_pos.squared_distance_to(sphere.center());
        let radius2 = sphere.radius().powi(2);
        assert_le!(dist2.dist_to(radius2), 1e-4);
    }
}
