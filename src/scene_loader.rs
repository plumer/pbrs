use core::panic;
use log::{error, info, warn};
use std::collections::HashMap;
use std::sync::Arc;

use math::hcm;
use geometry::camera::Camera;

use radiometry::color::Color;
use crate::instance::AffineTransform;
use crate::light;
use crate::light::{Light, ShapeSample};
use crate::material::{self as mtl, Material};
use scene::{
    ast::{self, ArgValue, ParameterSet},
    lexer, parser, plyloader, token,
};
use shape::{self, IsolatedTriangle, Shape, TriangleMeshRaw};
use crate::texture::{self as tex, Texture};

#[allow(dead_code)]
pub struct Scene {
    texture_descriptors: Vec<Box<dyn Texture>>,
    named_textures: HashMap<String, usize>,

    meshes: Vec<TriangleMeshRaw>,
    named_meshes: HashMap<String, usize>,

    materials: Vec<Box<dyn Material>>,
    named_materials: HashMap<String, usize>,
    
    pub tlas: crate::tlas::BvhNode,
    pub lights: Vec<Box<dyn Light>>,
}

pub struct SceneLoader {
    root_dir: std::path::PathBuf,
    // ctm_stack: Vec<hcm::Mat4>,
    ctm_stack: Vec<crate::instance::InstanceTransform>,
    current_mtl: Option<Arc<dyn Material>>,
    current_arealight_luminance: Option<Color>,
    reverse_orientation_stack: Vec<bool>,

    named_textures: HashMap<String, Arc<dyn Texture>>,
    named_materials: HashMap<String, Arc<dyn Material>>,
    pub instances: Vec<crate::instance::Instance>,
    pub camera: Option<Camera>,
    pub lights: Vec<Box<dyn Light>>,
}

#[allow(dead_code)]
pub fn build_scene(path: &str) -> SceneLoader {
    let lexer = lexer::Lexer::from_file(path).unwrap();
    let mut tokens = lexer.read_tokens();
    let has_errors = tokens.iter().any(|t| *t == token::Token::Error);
    info!("Has errors = {}", has_errors);
    if has_errors {
        println!("{:?}", tokens);
    }
    tokens.push(token::Token::End);

    let mut parser = parser::Parser::new(tokens.into_iter());
    let ast = parser.parse_scene();

    let root_dir = std::path::PathBuf::from(std::path::Path::new(path).parent().unwrap());
    let mut loader = SceneLoader::new(root_dir);
    loader.traverse_tree(ast);
    return loader;
}

impl SceneLoader {
    fn new(root_dir: std::path::PathBuf) -> Self {
        SceneLoader {
            root_dir,
            ctm_stack: vec![AffineTransform::identity()],
            // ctm_stack: vec![hcm::Mat4::identity()],
            current_mtl: None,
            current_arealight_luminance: None,
            reverse_orientation_stack: vec![false],
            named_textures: HashMap::new(),
            named_materials: HashMap::new(),
            instances: vec![],
            camera: None,
            lights: vec![],
        }
    }

    fn build_camera(scene_options: Vec<ast::SceneWideOption>) -> Option<Camera> {
        let mut fov = None;
        let mut w = None;
        let mut h = None;
        let mut pose = None;
        for _scene_option in scene_options.into_iter() {
            match _scene_option {
                ast::SceneWideOption::Camera(camera_impl, mut args) => {
                    if camera_impl != "perspective" {
                        error!("Non perspective camera {} unsupported", camera_impl);
                    }
                    fov = match args.extract("float fov") {
                        None => Some(hcm::Degree(60.0)),
                        Some(ArgValue::Number(deg)) => Some(hcm::Degree(deg)),
                        Some(wtf) => panic!("complicated fov degree: {:?}", wtf),
                    };
                }
                ast::SceneWideOption::Film(_film_impl, args) => {
                    w = args.lookup_f32("integer xresolution");
                    h = args.lookup_f32("integer yresolution");
                }
                ast::SceneWideOption::Transform(t) => {
                    use ast::Transform;
                    pose = match t {
                        Transform::LookAt(from, target, up) => Some((from, target, up)),
                        wtf => panic!(
                            "unsupported transform in scene-wide options (lookat only): {:?}",
                            wtf
                        ),
                    };
                }
                _ => error!("unhandled scene-wide option {:?}", _scene_option),
            }
        }
        let mut camera: Camera;
        match (fov, w, h) {
            (Some(degree), Some(width), Some(height)) => {
                camera = Camera::new((width as u32, height as u32), degree.to_radian());
            }
            _ => return None,
        }
        match pose {
            Some((eye, target, up)) => camera.look_at(eye, target, up),
            _ => (),
        }
        Some(camera)
    }

    fn traverse_tree(&mut self, ast: ast::Scene) {
        self.camera = Self::build_camera(ast.options);
        let items: Vec<ast::WorldItem> = ast.items;
        for world_item in items.into_iter() {
            self.traverse_world_item(world_item);
        }
        for instance in self.instances.iter() {
            info!(
                "transform = {}, shape summary = {}, bbox = {}, mtl type = {}",
                instance.transform,
                instance.shape.summary(),
                instance.bbox(),
                instance.mtl.summary()
            );
        }
    }

    fn traverse_world_item(&mut self, item: ast::WorldItem) {
        use ast::WorldItem;
        match item {
            WorldItem::Transform(t) => {
                // Parses the transform from parameters and applies that onto the current transform.
                *self.ctm_stack.last_mut().unwrap() =
                    *self.ctm_stack.last().unwrap() * Self::parse_transform(t);
            }
            WorldItem::Shape(shape_impl, mut args) => {
                args.extract_string("alpha");

                if self.current_mtl.is_some() && self.current_arealight_luminance.is_some() {
                    warn!("Both material and arealight are set, using only the light");
                    let shapes = self.parse_samplable_shape(&shape_impl, args);
                    shapes.into_iter().for_each(|shape| {
                        let diffuse_light = light::DiffuseAreaLight::new(
                            self.current_arealight_luminance.unwrap(),
                            shape,
                        );
                        self.lights.push(Box::new(diffuse_light));
                    });
                } else if let Some(mtl) = &self.current_mtl {
                    let shape = self.parse_shape(&shape_impl, args);
                    let inst = crate::instance::Instance::new(shape, mtl.clone())
                        .with_transform(self.ctm_stack.last().unwrap().clone());
                    self.instances.push(inst);
                } else if let Some(luminance) = self.current_arealight_luminance {
                    let shapes = self.parse_samplable_shape(&shape_impl, args);
                    shapes.into_iter().for_each(|shape| {
                        let diffuse_light = light::DiffuseAreaLight::new(luminance.clone(), shape);
                        self.lights.push(Box::new(diffuse_light));
                    });
                } else {
                    error!("Neither arealight luminance or material are set");
                }
            }
            WorldItem::Material(mtl_impl, parameters) => {
                let mtl = self.parse_material(mtl_impl, parameters);
                self.current_mtl = Some(mtl);
            }
            WorldItem::AttributeBlock(items) => {
                let last_transform = self.ctm_stack.last().unwrap().clone();
                self.ctm_stack.push(last_transform);
                let last_ro = self.reverse_orientation_stack.last().unwrap().clone();
                self.reverse_orientation_stack.push(last_ro);
                self.current_mtl = None;
                self.current_arealight_luminance = None;

                for child_item in items {
                    self.traverse_world_item(child_item);
                }
                self.ctm_stack.pop();
                self.reverse_orientation_stack.pop();
            }
            WorldItem::TransformBlock(items) => {
                let last_transform = self.ctm_stack.last().unwrap().clone();
                self.ctm_stack.push(last_transform);
                for child_item in items {
                    self.traverse_world_item(child_item);
                }
                self.ctm_stack.pop();
            }
            WorldItem::ObjectBlock(name, items) => {
                self.create_object(name, items);
            }
            WorldItem::MakeMaterial(name, mut args) => {
                let mtl_impl = args
                    .extract("string type")
                    .expect("no material type specified");
                if let ArgValue::String(mtl_impl) = mtl_impl {
                    let mtl = self.parse_material(mtl_impl, args);
                    self.named_materials.insert(name, mtl);
                } else {
                    panic!("ill-formed string type: {:?}", mtl_impl);
                }
            }
            WorldItem::Texture(tex_impl, tex_type, name, args) => {
                if tex_type == "color" {
                    let tex = self.parse_color_texture(tex_impl, args);
                    self.named_textures.insert(name, tex);
                }
            }
            WorldItem::MaterialInstance(name) => {
                self.current_mtl = self.named_materials.get(&name).cloned();
            }
            WorldItem::Light(light_impl, args) => {
                let light = Self::parse_light(light_impl, args);
                self.lights.push(light);
            }
            WorldItem::AreaLight(light_impl, mut args) => {
                if light_impl == "diffuse" {
                    let luminance = match args.extract_substr("L") {
                        None => unimplemented!("default illuminance for diffuse light"),
                        Some((key, ArgValue::Numbers(num))) => {
                            let spectrum_type = key.split(' ').next().unwrap();
                            Self::parse_constant_color(spectrum_type, num)
                        }
                        Some((_, wtf)) => unimplemented!("complicated luminance: {:?}", wtf),
                    };
                    self.current_arealight_luminance = Some(luminance);
                    info!("current arealight set to {}", luminance);
                } else {
                    error!("unhandled area light: {}", light_impl);
                }
            }
            _ => {
                error!("unhandled world item: {}", item);
            }
        }
    }

    fn parse_shape(&self, r#impl: &String, mut parameters: ast::ParameterSet) -> Arc<dyn Shape> {
        let implementation = r#impl.trim_matches('\"');
        match implementation {
            "sphere" => {
                let radius = parameters.lookup_f32("float radius").unwrap_or(1.0);
                Arc::new(shape::Sphere::new(hcm::Point3::origin(), radius))
            }
            "plymesh" => {
                let ply_file_name = parameters
                    .lookup_string("string filename")
                    .expect("no ply file specified");
                let mut ply_file_path = self.root_dir.clone();
                ply_file_path.push(ply_file_name);

                let _alpha_texture = parameters.lookup_string("alpha");
                let mesh = plyloader::load_ply(ply_file_path.to_str().unwrap());
                let tri_bvh = shape::TriangleMesh::build_from_raw(&mesh);
                info!(
                    "Triangle mesh with {} vertices and {} indices, bvh shape: {}",
                    mesh.vertices.len(),
                    mesh.indices.len(),
                    tri_bvh.bvh_shape_summary()
                );
                Arc::new(tri_bvh)
            }
            "trianglemesh" | "loopsubdiv" => {
                if implementation == "loopsubdiv" {
                    warn!("Unsupported subdiv, using regular triangle mesh");
                }
                let points_raw = match parameters.extract("point P").expect("missing points") {
                    ArgValue::Numbers(nums) => nums,
                    wtf => panic!("incorrect format for points: {:?}", wtf),
                };
                let points: Vec<_> = points_raw
                    .chunks_exact(3)
                    .map(|xyz| hcm::Point3::new(xyz[0], xyz[1], xyz[2]))
                    .collect();

                let uv_raw = match parameters.extract("float uv") {
                    None => vec![0.0; points.len() * 2],
                    Some(ArgValue::Numbers(nums)) => nums,
                    Some(wtf) => panic!("incorrect format for uv coords: {:?}", wtf),
                };
                let uvs: Vec<_> = uv_raw.chunks_exact(2).map(|uv| (uv[0], uv[1])).collect();

                let indices_raw = match parameters.extract("integer indices") {
                    None => panic!("missing indices"),
                    Some(ArgValue::Numbers(nums)) => nums,
                    Some(arg) => panic!("incorrect format for indices: {:?}", arg),
                };
                let indices = indices_raw
                    .chunks_exact(3)
                    .map(|ijk| (ijk[0] as usize, ijk[1] as usize, ijk[2] as usize))
                    .collect::<Vec<_>>();

                let normal_raw = match parameters.extract_substr("normal") {
                    None => vec![0.0; points_raw.len()],
                    Some((_, ArgValue::Numbers(nums))) => nums,
                    Some((_, wtf)) => panic!("incorrect format for normals: {:?}", wtf),
                };
                let normals = normal_raw
                    .chunks_exact(3)
                    .map(|xyz| hcm::Vec3::new(xyz[0], xyz[1], xyz[2]))
                    .collect::<Vec<_>>();

                let tri_bvh = shape::TriangleMesh::from_soa(points, normals, uvs, indices);
                info!("Triangle bvh summary: {}", tri_bvh.bvh_shape_summary());
                Arc::new(tri_bvh)
            }
            _ => unimplemented!("shape of {}", implementation),
        }
    }

    fn parse_samplable_shape(
        &self,
        shape_impl: &String,
        args: ast::ParameterSet,
    ) -> Vec<Box<dyn ShapeSample>> {
        if shape_impl == "sphere" {
            let radius = args.lookup_f32("float radius").unwrap_or(1.0);
            vec![Box::new(shape::Sphere::new(hcm::Point3::origin(), radius))]
        } else if shape_impl == "plymesh" {
            let ply_file_name = args
                .lookup_string("string filename")
                .expect("no ply file specified");
            let mut ply_file_path = self.root_dir.clone();
            ply_file_path.push(ply_file_name);

            let mesh = plyloader::load_ply(ply_file_path.to_str().unwrap());
            let mut triangles: Vec<Box<dyn ShapeSample>> = Vec::new();
            mesh.indices.chunks_exact(3).for_each(|ijk| {
                if let [i, j, k] = ijk {
                    let t = IsolatedTriangle::new(
                        mesh.vertices[*i as usize].pos,
                        mesh.vertices[*j as usize].pos,
                        mesh.vertices[*k as usize].pos,
                    );
                    triangles.push(Box::new(t));
                } else {
                    panic!("indices should be multiple of 3")
                }
            });
            triangles
        } else {
            unimplemented!("samplable shape: {}", shape_impl)
        }
    }

    fn parse_light(light_impl: String, _args: ast::ParameterSet) -> Box<dyn Light> {
        unimplemented!("light of {}", light_impl)
    }

    fn parse_material(
        &mut self,
        mtl_impl: String,
        mut parameters: ast::ParameterSet,
    ) -> Arc<dyn Material> {
        if mtl_impl == "glass" {
            let kr = match parameters.extract_substr("Kr") {
                None => Color::white(),
                Some((key, ArgValue::Numbers(num))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    Self::parse_constant_color(spectrum_type, num)
                }
                Some((_, ArgValue::Number(g))) => Color::gray(g),
                Some(_) => unimplemented!("textured reflectivity in glass"),
            };
            let kt = match parameters.extract_substr("Kt") {
                None => Color::white(),
                Some((key, ArgValue::Numbers(num))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    Self::parse_constant_color(spectrum_type, num)
                }
                Some((_, ArgValue::Number(g))) => Color::gray(g),
                Some(_) => unimplemented!("textured transmissivity in glass"),
            };
            let eta = match parameters.extract_substr("eta") {
                None => 1.5,
                Some((_, ArgValue::Number(e))) => e,
                Some((_, wtf)) => unimplemented!("complicated eta: {:?}", wtf),
            };
            let mtl = mtl::Dielectric::new(eta).with_colors(kr, kt);
            Arc::new(mtl)
        } else if mtl_impl == "mirror" {
            let reflectivity = match parameters.extract_substr("Kr") {
                None => Color::gray(0.9),
                Some((_, ArgValue::Number(g))) => Color::gray(g),
                Some((key, ArgValue::Numbers(nums))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    Self::parse_constant_color(spectrum_type, nums)
                }
                Some((_, ArgValue::String(_))) => {
                    unimplemented!("unsupported: textured reflectivity for mirror material")
                }
            };
            Arc::new(mtl::Mirror::new(reflectivity))
        } else if mtl_impl == "matte" {
            let kd_tex = match parameters.extract_substr("Kd") {
                None => Arc::new(tex::Solid::new(Color::gray(0.5))),
                Some((key, value)) => self.solid_or_image_tex(key, value),
            };
            let _sigma = match parameters.extract("sigma") {
                None => 0.0,
                Some(ArgValue::Number(num)) => num,
                Some(_) => unimplemented!("non-solid texture Kd unsupported for matte"),
            };
            // TODO: add Oren-Nayar model to Lambertian material.
            Arc::new(mtl::Lambertian::textured(kd_tex))
        } else if mtl_impl == "metal" {
            let roughness = match parameters.extract_substr("roughness") {
                None => 0.01,
                Some((_, ArgValue::Number(num))) => num,
                Some((_, wtf)) => panic!("roughness value isn't a number: {:?}", wtf),
            };
            let _remap_roughness = match parameters.extract("remaproughness") {
                None => true,
                Some(ArgValue::String(bool_str)) => bool_str.parse::<bool>().unwrap(),
                Some(wtf) => panic!("remaproughness value isn't bool: {:?}", wtf),
            };
            let _eta = 0.1;
            let _k = 0.2;
            Arc::new(mtl::Metal::new(Color::white(), roughness))
        } else if mtl_impl == "plastic" {
            let kd = match parameters.extract_substr("Kd") {
                None => Color::gray(0.25),
                Some((key, ArgValue::Numbers(nums))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    Self::parse_constant_color(spectrum_type, nums)
                }
                Some((_, wtf)) => unimplemented!("plastic, kd = {:?}", wtf),
            };
            let ks = match parameters.extract_substr("Ks") {
                None => Color::gray(0.25),
                Some((key, ArgValue::Numbers(nums))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    Self::parse_constant_color(spectrum_type, nums)
                }
                Some((_, wtf)) => unimplemented!("plastic, ks = {:?}", wtf),
            };
            let roughness = match parameters.extract_substr("roughness") {
                None => 0.1,
                Some((_, ArgValue::Number(r))) => r,
                Some((_, wtf)) => unimplemented!("plastic, kd = {:?}", wtf),
            };
            let remap_roughness = match parameters.extract_substr("remaproughness") {
                None => true,
                Some((_, ArgValue::String(bool_str))) => {
                    bool_str.parse::<bool>().expect("invalid boolean string")
                }
                Some((_, wtf)) => panic!("invalid remapreoughness: {:?}", wtf),
            };
            let mtl = mtl::Plastic {
                diffuse: kd,
                specular: ks,
                roughness,
                remap_roughness,
            };
            Arc::new(mtl)
        } else if mtl_impl == "uber" {
            let kd_tex = match parameters.extract_substr("Kd") {
                None => Arc::new(tex::Solid::new(Color::gray(0.25))),
                Some((key, value)) => self.solid_or_image_tex(key, value),
            };
            let ks_tex = match parameters.extract_substr("Ks") {
                None => Arc::new(tex::Solid::new(Color::gray(0.25))),
                Some((key, value)) => self.solid_or_image_tex(key, value),
            };
            let kr_tex = match parameters.extract_substr("Kr") {
                None => None,
                Some((key, value)) => Some(self.solid_or_image_tex(key, value)),
            };
            let kt_tex = match parameters.extract_substr("Kt") {
                None => None,
                Some((key, value)) => Some(self.solid_or_image_tex(key, value)),
            };
            let u_roughness = match parameters.extract_substr("uroughness") {
                None => 0.0,
                Some((_, ArgValue::Number(ur))) => ur,
                Some((_, wtf)) => panic!("invalid uroughness: {:?}", wtf),
            };
            let v_roughness = match parameters.extract_substr("vroughness") {
                None => 0.0,
                Some((_, ArgValue::Number(vr))) => vr,
                Some((_, wtf)) => panic!("invalid vroughness: {:?}", wtf),
            };
            let roughness = match parameters.extract_substr("roughness") {
                None => 0.0,
                Some((_, ArgValue::Number(roughness))) => roughness,
                Some((_, wtf)) => panic!("invalid roughness: {:?}", wtf),
            };
            let eta = match parameters.extract_substr("eta") {
                None => 1.5,
                Some((_, ArgValue::Number(eta))) => eta,
                Some((_, wtf)) => panic!("invalid eta: {:?}", wtf),
            };
            let opacity = match parameters.extract_substr("eta") {
                None => 1.0,
                Some((_, ArgValue::Number(o))) => o,
                Some((_, wtf)) => panic!("invalid opacity: {:?}", wtf),
            };
            let remap_roughness = match parameters.extract_substr("remaproughness") {
                None => true,
                Some((_, ArgValue::String(bool_str))) => {
                    bool_str.parse::<bool>().expect("invalid boolean string")
                }
                Some((_, wtf)) => panic!("invalid remapreoughness: {:?}", wtf),
            };
            let rough = if u_roughness == v_roughness {
                mtl::Roughness::Iso(roughness)
            } else {
                mtl::Roughness::UV((u_roughness, v_roughness))
            };
            Arc::new(mtl::Uber {
                kd: kd_tex,
                ks: ks_tex,
                kr: kr_tex,
                kt: kt_tex,
                rough,
                eta,
                opacity,
                remap_roughness,
            })
        } else if mtl_impl == "substrate" {
            let kd = match parameters.extract_substr("Kd") {
                None => Arc::new(tex::Solid::new(Color::gray(0.5))),
                Some((key, value)) => self.solid_or_image_tex(key, value),
            };
            let ks = match parameters.extract_substr("Ks") {
                None => Arc::new(tex::Solid::new(Color::gray(0.5))),
                Some((key, value)) => self.solid_or_image_tex(key, value),
            };
            let v_roughness = match parameters.extract_substr("vroughness") {
                None => 0.1,
                Some((_, ArgValue::Number(vr))) => vr,
                Some((_, wtf)) => panic!("invalid vroughness: {:?}", wtf),
            };
            let u_roughness = match parameters.extract_substr("uroughness") {
                None => 0.1,
                Some((_, ArgValue::Number(ur))) => ur,
                Some((_, wtf)) => panic!("invalid roughness: {:?}", wtf),
            };
            let remap_roughness = match parameters.extract_substr("remaproughness") {
                None => true,
                Some((_, ArgValue::String(bool_str))) => {
                    bool_str.parse::<bool>().expect("invalid boolean string")
                }
                Some((_, wtf)) => panic!("invalid remapreoughness: {:?}", wtf),
            };
            let rough = if v_roughness == u_roughness {
                mtl::Roughness::Iso(v_roughness)
            } else {
                mtl::Roughness::UV((u_roughness, v_roughness))
            };
            Arc::new(mtl::Substrate {
                kd,
                ks,
                rough,
                remap_roughness,
            })
        } else if mtl_impl == "fourier" {
            error!("unimplemented fourier");
            Arc::new(mtl::Lambertian::solid(Color::white()))
        } else {
            panic!("not recognized material: {}", mtl_impl)
        }
    }

    fn parse_color_texture(&self, tex_impl: String, mut args: ParameterSet) -> Arc<dyn Texture> {
        if tex_impl == "imagemap" {
            let file_name = match args.extract("string filename") {
                None => panic!("missing file name for image map texture"),
                Some(ArgValue::String(f)) => f,
                Some(_) => panic!("invalid filename for image map"),
            };
            let mut file_path = self.root_dir.clone();
            file_path.push(file_name);
            let file_path = file_path.to_str().unwrap();

            let i = tex::Image::from_file(file_path).expect("can't load image");
            Arc::new(i)
        } else {
            unimplemented!("tex impl = {}", tex_impl)
        }
    }

    /// Consumes the parameter value and returns an `Arc<dyn Texture>`. The texture can be either:
    ///
    /// - A solid-color texture, where the parameter should be a 3-number array of RGB values,
    /// - or an image texture which has been recorded by the SceneLoader before.
    fn solid_or_image_tex(&mut self, key: String, arg_value: ast::ArgValue) -> Arc<dyn Texture> {
        match arg_value {
            ast::ArgValue::Numbers(nums) => {
                let color_type = key.split(' ').next().unwrap();
                let color = Self::parse_constant_color(color_type, nums);
                let solid_tex = tex::Solid::new(color);
                Arc::new(solid_tex)
            }
            ast::ArgValue::Number(x) => {
                let color = Color::gray(x);
                Arc::new(tex::Solid::new(color))
            }
            ast::ArgValue::String(tex_name) => self.named_textures.get(&tex_name).unwrap().clone(),
        }
    }

    /// Converts the parameter value into a `Color` using the specified `spectrum_type`.
    /// The spectrum type can be any of "rgb", "xyz", "blackbody" or "spectrum".
    ///
    /// Panics if the spectrum type isn't any of the supported types.
    fn parse_constant_color(spectrum_type: &str, nums: Vec<f32>) -> Color {
        match spectrum_type {
            "rgb" | "color" => Color::new(nums[0], nums[1], nums[2]),
            "xyz" => Color::from_xyz(nums[0], nums[1], nums[2]),
            "spectrum" => unimplemented!("spectrum color"),
            "blackbody" => radiometry::spectrum::temperature_to_color(nums[0]) * nums[1],
            _ => panic!("unrecognized spectrum type \'{}\'", spectrum_type),
        }
    }

    fn create_object(&mut self, _name: String, items: Vec<ast::WorldItem>) {
        let mut object_loader = SceneLoader::new(self.root_dir.clone());

        object_loader
            .ctm_stack
            .push(self.ctm_stack.last().unwrap().clone());
        object_loader
            .reverse_orientation_stack
            .push(self.reverse_orientation_stack.last().unwrap().clone());

        for item in items {
            object_loader.traverse_world_item(item);
        }
        unimplemented!()
    }

    // Static functions
    // ---------------------------------------------------------------------------------------------
    #[allow(dead_code)]
    fn parse_transform(t: ast::Transform) -> AffineTransform {
        use ast::Transform;
        match t {
            Transform::Identity => AffineTransform::identity(),
            Transform::Translate(v) => AffineTransform::translater(v),
            Transform::Scale(s) => AffineTransform::scaler(s),
            Transform::Rotate(axis, angle) => AffineTransform::rotater(axis, angle.to_radian()),
            Transform::LookAt(_, _, _) => panic!("unsupported lookat in modeling step"),
        }
    }
    #[allow(dead_code)]
    fn parse_rbtransform(t: ast::Transform) -> crate::instance::RigidBodyTransform {
        use crate::instance::RigidBodyTransform as RBTrans;
        use ast::Transform;
        match t {
            Transform::Identity => RBTrans::identity(),
            Transform::Translate(v) => RBTrans::translater(v),
            Transform::Scale(s) => {
                error!("scaling of {} unsupported", s);
                RBTrans::identity()
            }
            Transform::Rotate(axis, angle) => RBTrans::rotater(axis, angle.to_radian()),
            Transform::LookAt(..) => panic!("unsupported lookat in modeling step"),
        }
    }
}
