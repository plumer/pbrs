pub mod ast;
pub mod lexer;
pub mod parser;
pub mod plyloader;
pub mod token;

use std::collections::HashMap;
use std::sync::Arc;

pub use plyloader::load_ply;

use crate::hcm;
use crate::image::Color;
use crate::material as mtl;
use crate::material::Material;
use crate::shape;
use crate::shape::Shape;
use crate::shape::TriangleMeshRaw;
use crate::texture as tex;
use crate::texture::Texture;

use self::ast::{ArgValue, ParameterSet};

struct Scene {
    texture_descriptors: Vec<Box<dyn Texture>>,
    named_textures: HashMap<String, usize>,

    meshes: Vec<TriangleMeshRaw>,
    named_meshes: HashMap<String, usize>,

    materials: Vec<Box<dyn Material>>,
    named_materials: HashMap<String, usize>,
}

struct SceneLoader {
    root_dir: std::path::PathBuf,
    ctm_stack: Vec<hcm::Mat4>,
    current_mtl: Option<Arc<dyn Material>>,
    reverse_orientation_stack: Vec<bool>,

    named_textures: HashMap<String, Arc<dyn Texture>>,
    named_materials: HashMap<String, Arc<dyn Material>>,
    pub instances: Vec<crate::instance::Instance>,
}

pub fn build_scene(path: &str) {
    let lexer = lexer::Lexer::from_file(path).unwrap();
    let mut tokens = lexer.read_tokens();
    // println!("{:?}", tokens);
    let has_errors = tokens.iter().any(|t| *t == token::Token::Error);
    println!("has errors = {}", has_errors);
    tokens.push(token::Token::End);

    let mut parser = parser::Parser::new(tokens.into_iter());
    let ast = parser.parse_scene();

    let root_dir = std::path::PathBuf::from(std::path::Path::new(path).parent().unwrap());
    let mut loader = SceneLoader::new(root_dir);
    loader.traverse_tree(ast);
}

impl SceneLoader {
    fn new(root_dir: std::path::PathBuf) -> Self {
        SceneLoader {
            root_dir,
            ctm_stack: vec![hcm::Mat4::identity()],
            current_mtl: None,
            reverse_orientation_stack: vec![false],
            named_textures: HashMap::new(),
            named_materials: HashMap::new(),
            instances: vec![],
        }
    }

    fn traverse_tree(&mut self, ast: ast::Scene) {
        for _scene_option in ast.options.iter() {}
        let items: Vec<ast::WorldItem> = ast.items;
        for world_item in items.into_iter() {
            self.traverse_world_item(world_item);
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
            WorldItem::Shape(implementation, mut parameters) => {
                parameters.extract_string("alpha");
                let shape = self.parse_shape(&implementation, parameters);

                if let Some(mtl) = &self.current_mtl {
                    let inst = crate::instance::Instance::new(shape, mtl.clone());
                    self.instances.push(inst);
                } else {
                    eprintln!("material not set");
                }
            }
            WorldItem::Material(r#impl, parameters) => {
                let mtl = self.parse_material(r#impl, parameters);
                self.current_mtl = Some(mtl);
            }
            WorldItem::AttributeBlock(items) => {
                let last_transform = self.ctm_stack.last().unwrap().clone();
                self.ctm_stack.push(last_transform);
                let last_ro = self.reverse_orientation_stack.last().unwrap().clone();
                self.reverse_orientation_stack.push(last_ro);

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
            _ => {
                eprintln!("unhandled world item: {}", item);
            },
        }
    }

    fn parse_shape(&self, r#impl: &String, parameters: ast::ParameterSet) -> Arc<dyn Shape> {
        let implementation = r#impl.trim_matches('\"');
        match implementation {
            "sphere" => {
                let radius = parameters.lookup_f32("radius").unwrap_or(1.0);
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
                Arc::new(tri_bvh)
            }
            _ => unimplemented!("implementation = {}", implementation),
        }
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
                    self.parse_constant_color(spectrum_type, num)
                }
                Some((_, ArgValue::Number(g))) => Color::gray(g),
                Some(_) => unimplemented!("textured reflectivity in glass"),
            };
            let kt = match parameters.extract_substr("Kt") {
                None => Color::white(),
                Some((key, ArgValue::Numbers(num))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    self.parse_constant_color(spectrum_type, num)
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
                    self.parse_constant_color(spectrum_type, nums)
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
                    self.parse_constant_color(spectrum_type, nums)
                }
                Some((_, wtf)) => unimplemented!("plastic, kd = {:?}", wtf),
            };
            let ks = match parameters.extract_substr("Ks") {
                None => Color::gray(0.25),
                Some((key, ArgValue::Numbers(nums))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    self.parse_constant_color(spectrum_type, nums)
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
            let mtl = mtl::Plastic{diffuse: kd, specular: ks, roughness, remap_roughness};
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
            eprintln!("unimplemented fourier");
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
                let color = self.parse_constant_color(color_type, nums);
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
    fn parse_constant_color(&self, spectrum_type: &str, nums: Vec<f32>) -> Color {
        match spectrum_type {
            "rgb" | "color" => Color::new(nums[0], nums[1], nums[2]),
            "xyz" => unimplemented!("xyz color"),
            "spectrum" => unimplemented!("spectrum color"),
            "blackbody" => unimplemented!("blackbody"),
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
    fn parse_transform(t: ast::Transform) -> hcm::Mat4 {
        use ast::Transform;
        match t {
            Transform::Identity => hcm::Mat4::identity(),
            Transform::Translate(v) => hcm::Mat4::translater(v),
            Transform::Scale(s) => hcm::Mat4::nonuniform_scale(s),
            Transform::Rotate(axis, angle) => hcm::Mat4::rotater(axis, angle.to_radian()),
            Transform::LookAt(_, _, _) => panic!("unsupported lookat in modeling step"),
        }
    }
}
