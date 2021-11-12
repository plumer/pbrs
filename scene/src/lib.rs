pub mod loader;
pub mod plyloader;
pub mod preset;

use geometry::camera::Camera;
use light::{DeltaLight, DiffuseAreaLight};

#[allow(dead_code)]
pub struct Scene {
    // texture_descriptors: Vec<Box<dyn Texture>>,
    // named_textures: HashMap<String, usize>,

    // meshes: Vec<TriangleMeshRaw>,
    // named_meshes: HashMap<String, usize>,

    // materials: Vec<Box<dyn Material>>,
    // named_materials: HashMap<String, usize>,
    pub tlas: tlas::bvh::BvhNode,
    pub delta_lights: Vec<DeltaLight>,
    pub area_lights: Vec<DiffuseAreaLight>,
    pub env_light: Option<light::EnvLight>,
    pub camera: Camera,
}

impl Scene {
    pub fn new(tlas: tlas::bvh::BvhNode, camera: Camera) -> Self {
        Self {
            tlas,
            env_light: None,
            delta_lights: vec![],
            area_lights: vec![],
            camera,
        }
    }
    pub fn with_env_light(self, env_light: light::EnvLight) -> Self {
        Self {
            env_light: Some(env_light),
            ..self
        }
    }
    pub fn with_lights(
        self, delta_lights: Vec<DeltaLight>, area_lights: Vec<DiffuseAreaLight>,
    ) -> Self {
        Self {
            delta_lights,
            area_lights,
            ..self
        }
    }
}
