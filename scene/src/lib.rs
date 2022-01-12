pub mod loader;
pub mod plyloader;
pub mod preset;

use std::f32::consts::{PI, FRAC_1_PI};

use geometry::{camera::Camera, ray::Ray};
use light::{DeltaLight, DiffuseAreaLight};
use radiometry::color::Color;
use texture::Texture;

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
    env_light: Option<light::EnvLight>,
    env_map: Option<texture::Image>,
    pub camera: Camera,
}

impl Scene {
    pub fn new(tlas: tlas::bvh::BvhNode, camera: Camera) -> Self {
        Self {
            tlas,
            env_light: None,
            env_map: None,
            delta_lights: vec![],
            area_lights: vec![],
            camera,
        }
    }
    pub fn with_env_light(self, env_light: light::EnvLight) -> Self {
        assert!(self.env_map.is_none());
        Self {
            env_light: Some(env_light),
            ..self
        }
    }
    pub fn with_env_map(self, env_map: texture::Image) -> Self {
        assert!(self.env_light.is_none());
        Self {
            env_map: Some(env_map),
            ..self
        }
    }
    
    pub fn has_env_light(&self) -> bool {
        self.env_light.is_some() || self.env_map.is_some()
    }
    
    /// Evaluates environment light on the given ray, or black if no environment lights are set.
    pub fn eval_env_light(&self, ray: Ray) -> Color {
        if let Some(env_light) = &self.env_light {
            env_light(ray)
        } else if let Some(env_map) = &self.env_map {
            let phi = ray.dir.z.atan2(ray.dir.x);
            let u = (phi * FRAC_1_PI * 0.5 + 1.0).fract();
            let cos_theta = ray.dir.y / ray.dir.norm();
            let v = cos_theta.acos() / PI;
            env_map.value((u, v), math::hcm::Point3::origin())
        } else {
            Color::black()
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
