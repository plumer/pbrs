use math::hcm::{Point3, Vec3, Mat3, Mat4};
use math::Angle;
use std::ops::Mul;
use std::convert::TryFrom;
use crate::ray::Ray;
use crate::bvh::BBox;
use crate::interaction::Interaction;

#[derive(Debug, Clone, Copy)]
pub struct RigidBodyTransform {
    rotation: Mat3,
    translation: Vec3,
}

#[derive(Debug, Clone, Copy)]
pub struct AffineTransform {
    forward: Mat4,
    inverse: Mat4,
}

pub trait Transform<T> {
    fn apply(&self, x: T) -> T;
}

#[allow(dead_code)]
impl RigidBodyTransform {
    fn build(rotation: Mat3, translation: Vec3) -> Self {
        Self {
            rotation,
            translation,
        }
    }
    pub fn identity() -> Self {
        Self::build(Mat3::IDENTITY, Vec3::ZERO)
    }

    pub fn translater(t: Vec3) -> Self {
        Self::build(Mat3::IDENTITY, t)
    }

    // rotation: [1, 0] -> [cosT, sinT], [0, 1] -> [-sinT, cosT]

    pub fn rotater_x(angle: Angle) -> Self {
        Self::build(Mat3::rotater_x(angle), Vec3::ZERO)
    }

    pub fn rotater_y(angle: Angle) -> Self {
        Self::build(Mat3::rotater_y(angle), Vec3::ZERO)
    }

    pub fn rotater_z(angle: Angle) -> Self {
        Self::build(Mat3::rotater_z(angle), Vec3::ZERO)
    }

    pub fn rotater(axis: Vec3, angle: Angle) -> Self {
        Self::build(Mat3::rotater(axis, angle), Vec3::ZERO)
    }

    pub fn inverse(&self) -> Self {
        Self::build(
            self.rotation.transpose(),
            -(self.rotation.transpose() * self.translation),
        )
    }

    /// Applies Translate(t) onto the transform, and returns Translate(t) * self.
    pub fn translate(self, t: Vec3) -> Self {
        Self::build(self.rotation, self.translation + t)
    }

    /// Applies rotation onto the transform, and returns Rotate(angle) * self.
    pub fn rotate_x(self, angle: Angle) -> Self {
        let rot = Mat3::rotater_x(angle);
        Self::build(rot * self.rotation, rot * self.translation)
    }
    pub fn rotate_y(self, angle: Angle) -> Self {
        let rot = Mat3::rotater_y(angle);
        Self::build(rot * self.rotation, rot * self.translation)
    }
    pub fn rotate_z(self, angle: Angle) -> Self {
        let rot = Mat3::rotater_z(angle);
        Self::build(rot * self.rotation, rot * self.translation)
    }
}

impl Mul for RigidBodyTransform {
    type Output = RigidBodyTransform;
    fn mul(self, rhs: Self) -> Self::Output {
        Self::build(
            self.rotation * rhs.rotation,
            self.rotation * rhs.translation + self.translation,
        )
    }
}

impl std::fmt::Display for RigidBodyTransform {
    #[rustfmt::skip]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let rot = &self.rotation;
        write!(
            f,
            "\n|{:5.2} {:5.2} {:5.2} {:5.2}\
             \n|{:5.2} {:5.2} {:5.2} {:5.2}\
             \n|{:5.2} {:5.2} {:5.2} {:5.2}\n",
            rot.cols[0].x, rot.cols[1].x, rot.cols[2].x, self.translation.x,
            rot.cols[0].y, rot.cols[1].y, rot.cols[2].y, self.translation.y,
            rot.cols[0].z, rot.cols[1].z, rot.cols[2].z, self.translation.z
        )
    }
}

impl std::fmt::Display for AffineTransform {
    #[rustfmt::skip]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let m = &self.forward;
        write!(
            f,
            "\n|{:5.2} {:5.2} {:5.2} {:5.2}|\
             \n|{:5.2} {:5.2} {:5.2} {:5.2}|\
             \n|{:5.2} {:5.2} {:5.2} {:5.2}|\
             \n|{:5.2} {:5.2} {:5.2} {:5.2}|\n",
            m.cols[0][0], m.cols[1][0], m.cols[2][0], m.cols[3][0],
            m.cols[0][1], m.cols[1][1], m.cols[2][1], m.cols[3][1],
            m.cols[0][2], m.cols[1][2], m.cols[2][2], m.cols[3][2],
            m.cols[0][3], m.cols[1][3], m.cols[2][3], m.cols[3][3]
        )
    }
}

#[allow(dead_code)]
impl AffineTransform {
    pub fn identity() -> Self {
        Self {
            forward: Mat4::IDENTITY,
            inverse: Mat4::IDENTITY,
        }
    }
    pub fn translater(t: Vec3) -> Self {
        Self {
            forward: Mat4::translater(t),
            inverse: Mat4::translater(-t),
        }
    }
    pub fn rotater(axis: Vec3, angle: Angle) -> Self {
        let forward = Mat4::rotater(axis, angle);
        Self {
            forward: Mat4::rotater(axis, angle),
            inverse: forward.transpose(),
        }
    }
    pub fn inverse(&self) -> Self {
        Self {
            forward: self.inverse,
            inverse: self.forward,
        }
    }
    pub fn scaler(scale: Vec3) -> Self {
        let Vec3 { x, y, z } = scale;
        let scale_inv = Vec3::new(1.0 / x, 1.0 / y, 1.0 / z);
        Self {
            forward: Mat4::nonuniform_scale(scale),
            inverse: Mat4::nonuniform_scale(scale_inv),
        }
    }

    /// Applies Translate(t) onto the transform, and returns Translate(t) * self.
    pub fn translate(self, t: Vec3) -> Self {
        Self::translater(t) * self
    }

    /// Applies rotation onto the transform, and returns Rotate(angle) * self.
    pub fn rotate_x(self, angle: Angle) -> Self {
        Self::rotater(Vec3::X, angle) * self
    }
    pub fn rotate_y(self, angle: Angle) -> Self {
        Self::rotater(Vec3::Y, angle) * self
    }
    pub fn rotate_z(self, angle: Angle) -> Self {
        Self::rotater(Vec3::Z, angle) * self
    }
}

impl Mul for AffineTransform {
    type Output = AffineTransform;
    fn mul(self, rhs: Self) -> Self::Output {
        // self * rhs -> self.forward * rhs.forward, rhs.inverse * self.inverse.
        Self {
            forward: self.forward * rhs.forward,
            inverse: rhs.inverse * self.inverse,
        }
    }
}

// use RigidBodyTransform as InstanceTransform;
pub use AffineTransform as InstanceTransform;
pub fn identity() -> InstanceTransform {
    InstanceTransform::identity()
}


// Implements all kinds of transforms that `RigidbodyTransform` can do.
// Transforms on:
// - Vec3
// - Point3
// - Ray
// - BBox
// -------------------------------------------------------------------------------------------------

impl Transform<Vec3> for RigidBodyTransform {
    fn apply(&self, x: Vec3) -> Vec3 {
        self.rotation * x
    }
}
impl Transform<Point3> for RigidBodyTransform {
    fn apply(&self, p: Point3) -> Point3 {
        let res = self.rotation * Vec3::from(p) + self.translation;
        Point3::from(res)
    }
}
impl Transform<Ray> for RigidBodyTransform {
    fn apply(&self, r: Ray) -> Ray {
        Ray::new(self.apply(r.origin), self.apply(r.dir)).with_extent(r.t_max)
    }
}
impl Transform<BBox> for RigidBodyTransform {
    fn apply(&self, b: BBox) -> BBox {
        let bases = self.rotation.cols;
        let mut res_box = BBox::empty();
        let diag = b.diag();
        for i in 0..8 {
            let mut corner = self.apply(b.min());
            if i & 1 != 0 {
                corner = corner + diag[0] * bases[0];
            }
            if i & 2 != 0 {
                corner = corner + diag[1] * bases[1];
            }
            if i & 4 != 0 {
                corner = corner + diag[2] * bases[2];
            }
            res_box = res_box.union(corner);
        }

        res_box
    }
}
impl Transform<Interaction> for RigidBodyTransform {
    fn apply(&self, i: Interaction) -> Interaction {
        // Note that the transform is rigid-body, so transforming the normal is straightforward.
        let new_pos = self.apply(i.pos);
        let new_normal = self.apply(i.normal);
        let new_wo = self.apply(i.wo);
        Interaction::new(new_pos, i.ray_t, i.uv, new_normal, new_wo)
    }
}

// Implements all kinds of transforms that `UniveralTransform` can do.
// Transforms on:
// - Vec3
// - Point3
// - Ray
// - BBox
// -------------------------------------------------------------------------------------------------

impl Transform<Vec3> for AffineTransform {
    fn apply(&self, x: Vec3) -> Vec3 {
        let x4 = x.as_vec4();
        (self.forward * x4).into()
    }
}
impl Transform<Point3> for AffineTransform {
    fn apply(&self, p: Point3) -> Point3 {
        let v4 = p.as_vec4();
        let v4 = self.forward * v4;
        assert_eq!(v4[3], 1.0, "v4 = {v4}, forward = {self}",);
        // Point3::new(v4[0], v4[1], v4[2])
        Point3::try_from(v4).unwrap()
    }
}
impl Transform<Ray> for AffineTransform {
    fn apply(&self, r: Ray) -> Ray {
        Ray::new(self.apply(r.origin), self.apply(r.dir)).with_extent(r.t_max)
    }
}
impl Transform<BBox> for AffineTransform {
    fn apply(&self, b: BBox) -> BBox {
        let bases = self.forward.orientation().cols;
        let mut res_box = BBox::empty();
        let diag = b.diag();
        for i in 0..8 {
            let mut corner = self.apply(b.min());
            if i & 1 != 0 {
                corner = corner + diag[0] * bases[0];
            }
            if i & 2 != 0 {
                corner = corner + diag[1] * bases[1];
            }
            if i & 4 != 0 {
                corner = corner + diag[2] * bases[2];
            }
            res_box = res_box.union(corner);
        }

        res_box
    }
}
impl Transform<Interaction> for AffineTransform {
    fn apply(&self, i: Interaction) -> Interaction {
        assert!(!i.pos.has_nan());
        let new_pos = self.apply(i.pos);
        let new_wo = self.apply(i.wo);
        let new_normal = self.inverse.transpose() * i.normal;
        let res = Interaction::new(new_pos, i.ray_t, i.uv, new_normal, new_wo)
            .with_dpdu(self.apply(i.tangent()));
        assert!(res.has_valid_frame());
        res
    }
}

#[cfg(test)]
mod test {
    use math::assert_le;

    #[test]
    pub fn test_inverse() {
        type Trans = super::RigidBodyTransform;
        let trans = Trans::rotater(super::Vec3::new(0.6, 0.8, 0.0), math::new_rad(0.3));
        let trans = trans * Trans::translater(super::Vec3::new(0.3, 0.4, 0.6));

        let inv = trans.inverse();

        let expected_identity = inv * trans;

        assert_le!(
            (expected_identity.rotation - super::Mat3::IDENTITY).frobenius_norm_squared(),
            f32::EPSILON
        );
        assert_le!(expected_identity.translation.norm_squared(), f32::EPSILON);
    }

    #[test]
    pub fn test_bbox_transform() {
        use super::*;
        type Trans = RigidBodyTransform;
        let trans = Trans::rotater(Vec3::new(0.6, 0.8, 0.0), math::new_rad(0.3));
        let trans = trans * Trans::translater(Vec3::new(7.0, 8.0, -13.0));

        let bbox = BBox::new(Point3::new(-0.3, 0.4, 0.8), Point3::new(3.4, 2.3, 4.4));

        let t_bbox = trans.apply(bbox);

        for corner in bbox.all_corners().iter() {
            assert!(t_bbox.contains(trans.apply(*corner)));
        }
    }
}
