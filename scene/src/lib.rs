pub mod loader;
pub mod plyloader;
pub mod preset;

use std::f32::consts::{FRAC_1_PI, PI};

use geometry::{camera::Camera, ray::Ray};
use light::{DeltaLight, DiffuseAreaLight};
use radiometry::color::Color;
use texture::Texture;

pub enum EnvLight {
    Fn(light::EnvLight),
    Image(texture::Image, Color),
    Constant(Color),
}

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
    env_light: EnvLight,
    pub camera: Camera,
}

impl Scene {
    pub fn new(tlas: tlas::bvh::BvhNode, camera: Camera) -> Self {
        Self {
            tlas,
            delta_lights: vec![],
            area_lights: vec![],
            env_light: EnvLight::Constant(Color::black()),
            camera,
        }
    }

    pub fn from_loader(loader: loader::SceneLoader) -> Self {
        let camera = loader.camera.expect("camera not built in the loader");
        let boxed_instances = loader
            .instances
            .into_iter()
            .map(|i| Box::new(i))
            .collect::<Vec<_>>();
        let tlas = *tlas::build_bvh(boxed_instances);
        let mut delta_lights = loader.delta_lights;
        for distant_light in delta_lights.iter_mut() {
            if let light::DeltaLight::Distant { world_radius, .. } = distant_light {
                *world_radius = tlas.bbox().diag().norm() * 0.5;
            }
        }
        let mut new_scene = Self::new(tlas, camera).with_lights(delta_lights, loader.area_lights);
        new_scene.env_light = loader.env_light;
        new_scene
    }

    pub fn with_fn_env_light(self, fn_env_light: light::EnvLight) -> Self {
        if matches!(self.env_light, EnvLight::Image(..)) {
            log::warn!("Discarding existing image map environment light");
        }
        Self {
            env_light: EnvLight::Fn(fn_env_light),
            ..self
        }
    }

    pub fn with_const_env_light(self, color: Color) -> Self {
        if matches!(self.env_light, EnvLight::Image(..) | EnvLight::Fn(_)) {
            eprintln!("Discarding the existing environment light");
        }

        Self {
            env_light: EnvLight::Constant(color),
            ..self
        }
    }

    pub fn with_env_map(self, env_map: texture::Image, scale_factor: Color) -> Self {
        if matches!(self.env_light, EnvLight::Fn(_)) {
            log::warn!("Discarding existing environment light function");
        }
        Self {
            env_light: EnvLight::Image(env_map, scale_factor),
            ..self
        }
    }

    pub fn has_env_light(&self) -> bool {
        match self.env_light {
            EnvLight::Fn(_) => true,
            EnvLight::Image(..) => true,
            EnvLight::Constant(c) => !c.is_black(),
        }
    }

    /// Evaluates environment light on the given ray, or black if no environment lights are set.
    pub fn eval_env_light(&self, ray: Ray) -> Color {
        match &self.env_light {
            EnvLight::Fn(f) => f(ray),
            EnvLight::Image(env_map, scale_factor) => {
                let phi = ray.dir.z.atan2(ray.dir.x);
                let u = (phi * FRAC_1_PI * 0.5 + 1.0).fract();
                let cos_theta = ray.dir.y / ray.dir.norm();
                let v = cos_theta.acos() / PI;
                env_map.value((u, v), math::hcm::Point3::ORIGIN) * *scale_factor
            }
            EnvLight::Constant(c) => *c,
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
