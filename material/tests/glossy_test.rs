use geometry::bxdf::{self, BxDF};
use geometry::Interaction;
use material::Material;
use math::hcm::{Point3, Vec3};
use radiometry::color::Color;

#[test]
fn glossy_test() {
    let glossy = material::Glossy::new(Color::new(0.5, 0.6, 0.8), 0.02);
    let mut bxdfs = glossy.bxdfs_at(&Interaction::rayless(Point3::ORIGIN, (0.0, 0.0), Vec3::Z));
    assert_eq!(bxdfs.len(), 1);
    let bxdf = bxdfs.pop().unwrap();

    let wo = bxdf::Omega::normalize(0.0, 0.0, 0.8);

    let (uvec, _du) = math::float::linspace((0.0, 1.0), 20);
    let (vvec, _dv) = math::float::linspace((0.0, 1.0), 20);

    for u in uvec.iter().copied() {
        for v in vvec.iter().copied() {
            let (_color, wi, pr) = bxdf.sample(wo, (u, v));
            println!("{},{},{}, {}", wi.x(), wi.y(), wi.z(), pr.density());
        }
    }
}
