use geometry::ray;
use math::hcm::{self, Point3, Vec3};
use shape::Interaction;
use shape::Shape;

#[test]
fn quad_frame_test() {
    let quad_xy = shape::QuadXY::from_raw((-1.0, 1.0), (-1.0, 1.0), 0.0);
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
        normal, dpdv, normal.dot(dpdv)
    );

    let frame = hcm::Mat3::from_vectors(dpdu, dpdv, normal);
    assert!((frame * frame.transpose() - hcm::Mat3::identity()).frobenius_norm_squared() < 1e-6);

    let isect = Interaction::rayless(
        hcm::Point3::new(3.0, 2.5, 2.0),
        (0.2, 0.8),
        normal,
    )
    .with_dpdu(dpdu);

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
