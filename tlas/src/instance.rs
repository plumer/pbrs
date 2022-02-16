use std::sync::Arc;

use geometry::bvh::BBox;
use geometry::ray::Ray;
use geometry::Interaction;
use geometry::{transform::Transform, InstanceTransform};
use material::Material;

use shape::Shape;

#[derive(Clone)]
pub struct Instance {
    pub shape: Arc<dyn Shape>,
    pub mtl: Arc<dyn Material>,
    pub transform: InstanceTransform,
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
            transform: InstanceTransform::identity(),
        }
    }
    pub fn from_raw<S: 'static, M: 'static>(shape: S, mtl: M) -> Self
    where
        S: Shape,
        M: Material,
    {
        Instance::new(Arc::new(shape), Arc::new(mtl))
    }
    pub fn with_transform(self, transform: InstanceTransform) -> Self {
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
        assert!(inv_ray.dir.norm_squared() > 1e-3, "ray = {:?}", ray);
        let hit = self.shape.intersect(&inv_ray)?;
        assert!(
            !hit.pos.has_nan(),
            "shape {} intersect ray {ray} has nan",
            self.shape.summary(),
        );
        assert!(
            hit.has_valid_frame(),
            "hit doesn't have valid frame. shape = {}, tangent = {}, ray = {:.3}",
            self.shape.summary(),
            hit.tangent(),
            ray,
        );
        Some((self.transform.apply(hit), &self.mtl))
    }
    pub fn occludes(&self, ray: &Ray) -> bool {
        let inv_ray = self.transform.inverse().apply(*ray);
        assert!(inv_ray.dir.norm_squared() > 1e-6, "ray = {:?}", ray);
        self.shape.occludes(&inv_ray)
    }
}
