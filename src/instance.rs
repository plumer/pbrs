use std::{ops::Mul, sync::Arc};

use crate::ray::Ray;
use crate::shape::Shape;
use crate::{bvh::BBox, material::Material};
use crate::{
    hcm::{Mat3, Point3, Radian, Vec3},
    shape::Interaction,
};

#[derive(Debug, Clone, Copy)]
pub struct RigidBodyTransform {
    rotation: Mat3,
    translation: Vec3,
}

pub fn identity() -> RigidBodyTransform {
    RigidBodyTransform::identity()
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
        Self::build(Mat3::identity(), Vec3::zero())
    }

    pub fn translater(t: Vec3) -> Self {
        Self::build(Mat3::identity(), t)
    }

    // rotation: [1, 0] -> [cosT, sinT], [0, 1] -> [-sinT, cosT]

    pub fn rotater_x(angle: Radian) -> Self {
        Self::build(Mat3::rotater_x(angle), Vec3::zero())
    }

    pub fn rotater_y(angle: Radian) -> Self {
        Self::build(Mat3::rotater_y(angle), Vec3::zero())
    }

    pub fn rotater_z(angle: Radian) -> Self {
        Self::build(Mat3::rotater_z(angle), Vec3::zero())
    }

    pub fn rotater(axis: Vec3, angle: Radian) -> Self {
        Self::build(Mat3::rotater(axis, angle), Vec3::zero())
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
    pub fn rotate_x(self, angle: Radian) -> Self {
        let rot = Mat3::rotater_x(angle);
        Self::build(rot * self.rotation, rot * self.translation)
    }
    pub fn rotate_y(self, angle: Radian) -> Self {
        let rot = Mat3::rotater_y(angle);
        Self::build(rot * self.rotation, rot * self.translation)
    }
    pub fn rotate_z(self, angle: Radian) -> Self {
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

#[derive(Clone)]
pub struct Instance {
    pub shape: Arc<dyn Shape>,
    pub mtl: Arc<dyn Material>,
    pub transform: RigidBodyTransform,
}

impl std::fmt::Debug for Instance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Instance[shape bbox = {}]", self.shape.bbox())
    }
}

#[allow(dead_code)]
impl Instance {
    pub fn new(shape: Arc<dyn Shape>, mtl: Arc<dyn Material>) -> Self {
        Instance {
            shape,
            mtl,
            transform: RigidBodyTransform::identity(),
        }
    }
    pub fn from_raw<S: 'static, M: 'static>(shape: S, mtl: M) -> Self
    where
        S: Shape,
        M: Material,
    {
        Instance::new(Arc::new(shape), Arc::new(mtl))
    }
    pub fn with_transform(self, transform: RigidBodyTransform) -> Self {
        Instance {
            shape: self.shape,
            mtl: self.mtl,
            transform,
        }
    }
    pub fn bbox(&self) -> BBox {
        self.transform.apply(self.shape.bbox())
    }
    pub fn intersect(&self, ray: &Ray) -> Option<(Interaction, &Arc<dyn Material>)> {
        let inv_ray = self.transform.inverse().apply(*ray);
        match self.shape.intersect(&inv_ray) {
            None => None,
            Some(hit) => Some((self.transform.apply(hit), &self.mtl)),
        }
    }
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
        Ray::new(self.apply(r.origin), self.apply(r.dir))
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
        Interaction::new(self.apply(i.pos), i.ray_t, i.uv, self.apply(i.normal))
    }
}

#[cfg(test)]
mod test {
    use crate::assert_le;

    #[test]
    pub fn test_inverse() {
        type Trans = super::RigidBodyTransform;
        let trans = Trans::rotater(super::Vec3::new(0.6, 0.8, 0.0), super::Radian(0.3));
        let trans = trans * Trans::translater(super::Vec3::new(0.3, 0.4, 0.6));

        let inv = trans.inverse();

        let expected_identity = inv * trans;

        assert_le!(
            (expected_identity.rotation - super::Mat3::identity()).frobenius_norm_squared(),
            f32::EPSILON
        );
        assert_le!(expected_identity.translation.norm_squared(), f32::EPSILON);
    }

    #[test]
    pub fn test_bbox_transform() {
        use super::*;
        type Trans = RigidBodyTransform;
        let trans = Trans::rotater(Vec3::new(0.6, 0.8, 0.0), Radian(0.3));
        let trans = trans * Trans::translater(Vec3::new(7.0, 8.0, -13.0));

        let bbox = BBox::new(Point3::new(-0.3, 0.4, 0.8), Point3::new(3.4, 2.3, 4.4));

        let t_bbox = trans.apply(bbox);

        for corner in bbox.all_corners().iter() {
            assert!(t_bbox.contains(trans.apply(*corner)));
        }
    }
}
