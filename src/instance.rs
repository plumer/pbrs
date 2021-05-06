use std::sync::Arc;

use crate::{shape::Interaction};
use crate::material::Material;
use crate::shape::Shape;
use crate::ray::Ray;

#[derive(Clone)]
pub struct Instance {
    pub shape: Arc<dyn Shape>,
    pub mtl: Arc<dyn Material>,
}

impl std::fmt::Debug for Instance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Instance").finish()
    }
}

impl Instance {
    pub fn new(shape: Arc<dyn Shape>, mtl: Arc<dyn Material>) -> Self {
        Instance { shape, mtl }
    }
    pub fn intersect(&self, ray : &Ray) -> Option<(Interaction, &Arc<dyn Material>)> {
        match self.shape.intersect(ray) {
            None => None,
            Some(hit) => Some((hit, &self.mtl))
        }
    }
}
