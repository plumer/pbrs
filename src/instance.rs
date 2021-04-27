use crate::shape::Shape;
use crate::material::Material;


#[allow(dead_code)]
pub struct Instance<'a> {
    pub shape: &'a dyn Shape,
    pub mtl: &'a dyn Material,
}

impl<'a> Instance<'a> {
    pub fn new(shape: &'a dyn Shape, mtl: &'a dyn Material) -> Self {
        Instance{shape, mtl}
    }
}