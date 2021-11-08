pub mod ast;
pub mod lexer;
pub mod parser;
pub mod plyloader;
pub mod token;
pub mod loader;
pub mod preset;

use light::{DeltaLight, DiffuseAreaLight};
use geometry::camera::Camera;

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
    pub fn new(tlas: tlas::bvh::BvhNode, camera: Camera, env_light: light::EnvLight) -> Self {
        Self {
            tlas,
            env_light: Some(env_light),
            delta_lights: vec![],
            area_lights: vec![],
            camera,
        }
    }
    pub fn new_no_envlight(tlas: tlas::bvh::BvhNode, camera: Camera) -> Self {
        Self {
            tlas,
            env_light: None,
            delta_lights: vec![],
            area_lights: vec![],
            camera,
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