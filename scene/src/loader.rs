use core::panic;

use log::{error, info, warn};
use std::fs::File;
use std::sync::Arc;
use std::{collections::HashMap, io::Write};

use crate::plyloader;
use geometry::camera::Camera;
use geometry::AffineTransform;
use light::{self, DeltaLight, DiffuseAreaLight, SamplableShape};
use material::{self as mtl, Material};
use math::hcm;
use radiometry::color::Color;
use scene_parser::{
    ast::{self, ArgValue, ParameterSet},
    lexer, parser, token,
};
use shape::{self, IsolatedTriangle, Shape};
use texture::{self as tex, Texture};

pub struct SceneLoader {
    root_dir: std::path::PathBuf,
    // ctm_stack: Vec<hcm::Mat4>,
    ctm_stack: Vec<geometry::InstanceTransform>,
    current_mtl: Option<Arc<dyn Material>>,
    current_arealight_luminance: Option<Color>,
    reverse_orientation_stack: Vec<bool>,

    named_textures: HashMap<String, Arc<dyn Texture>>,
    named_materials: HashMap<String, Arc<dyn Material>>,
    pub instances: Vec<tlas::instance::Instance>,
    pub camera: Option<Camera>,
    pub filter: math::filter::Filter,
    pub delta_lights: Vec<DeltaLight>,
    pub area_lights: Vec<DiffuseAreaLight>,
    pub env_light: crate::EnvLight,
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
            filter: math::filter::Filter::Box {
                radius: math::filter::UV { u: 0.5, v: 0.5 },
            },
            delta_lights: vec![],
            area_lights: vec![],
            env_light: crate::EnvLight::Constant(Color::black()),
        }
    }

    fn resolve_relative_path(&self, relative_path: &str) -> String {
        let mut absolute_path_buf = self.root_dir.clone();
        absolute_path_buf.push(relative_path);
        absolute_path_buf.to_str().unwrap().to_string()
    }

    /// Use relevant scene-wide options to build a camera and return it.
    /// Used scene-wide options will be consumed and others are left untouched in `scene_options`.
    /// Returns `None` if any of `fov`, `xresolution` or `yresolution` is missing in the options.
    fn build_camera(scene_options: &mut Vec<ast::SceneWideOption>) -> Option<Camera> {
        let mut fov = None;
        let mut w = None;
        let mut h = None;
        let mut pose = None;
        // Collects unused scene-wide options.
        let mut remaining_options = vec![];
        for scene_option in scene_options.drain(..) {
            match scene_option {
                ast::SceneWideOption::Camera(camera_impl, mut args) => {
                    if camera_impl != "perspective" {
                        error!("Non perspective camera {} unsupported", camera_impl);
                    }
                    fov = match args.extract("float fov") {
                        None => Some(math::new_deg(60.0)),
                        Some(ArgValue::Number(deg)) => Some(math::new_deg(deg)),
                        Some(wtf) => panic!("complicated fov degree: {:?}", wtf),
                    };
                }
                ast::SceneWideOption::Film(_film_impl, args) => {
                    w = args.lookup_f32("integer xresolution");
                    h = args.lookup_f32("integer yresolution");
                }
                ast::SceneWideOption::Transform(t) => {
                    use ast::Transform;
                    match t {
                        Transform::LookAt(from, target, up) => pose = Some((from, target, up)),
                        _ => remaining_options.push(ast::SceneWideOption::Transform(t)),
                    };
                }
                _ => remaining_options.push(scene_option),
            }
        }
        // Constructs the camera if all of fov, width, height are present.
        let ((angle, width), height) = fov.zip(w).zip(h)?;
        let mut camera = Camera::new((width as u32, height as u32), angle);

        // Performs look-at transform is a pose is parsed from parameters.
        if let Some((eye, target, up)) = pose {
            camera.look_at(eye, target, up);
        }
        // Unused scene-wide options are returned back to the fn argument.
        scene_options.extend(remaining_options);
        Some(camera)
    }

    fn traverse_tree(&mut self, mut ast: ast::Scene) {
        self.camera = Self::build_camera(&mut ast.options);
        let mut world_transform = AffineTransform::identity();
        for scene_option in ast.options.into_iter() {
            match scene_option {
                ast::SceneWideOption::Transform(t) => {
                    match t {
                        ast::Transform::LookAt(..) => log::error!("LookAt for non-camera"),
                        _ => world_transform = world_transform * Self::parse_transform(t),
                    };
                }
                ast::SceneWideOption::Filter(filter_impl, args) => {
                    self.filter = Self::parse_filter(filter_impl, args);
                }
                _ => error!("unhandled scene-wide option {:?}", scene_option),
            }
        }

        let items: Vec<ast::WorldItem> = ast.items;
        for world_item in items.into_iter() {
            self.traverse_world_item(world_item);
        }
        for instance in self.instances.iter_mut() {
            instance.transform = world_transform * instance.transform;
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

                if let Some(luminance) = self.current_arealight_luminance {
                    // Creates an area light and an instance for each shape.
                    let (samplable_shapes, shapes) = self.parse_samplable_shape(&shape_impl, args);
                    let ctm = self.ctm_stack.last().unwrap().clone();
                    self.area_lights
                        .extend(samplable_shapes.into_iter().map(|shape| {
                            light::DiffuseAreaLight::new(luminance, shape.transformed_by(ctm))
                        }));
                    // The material for the object instance is set to DiffuseLight of the same
                    // luminance.
                    if let Some(mtl) = &self.current_mtl {
                        let mtl_desc = mtl.summary();
                        if !mtl_desc.contains("DiffuseLight") {
                            warn!("Using diffuse light instead of current material {mtl_desc}");
                        }
                    }
                    let mtl = Arc::new(mtl::DiffuseLight::new(luminance));
                    self.instances.extend(shapes.into_iter().map(|shape| {
                        tlas::instance::Instance::new(shape, mtl.clone()).with_transform(ctm)
                    }));
                } else if let Some(mtl) = &self.current_mtl {
                    let shape = self.parse_shape(&shape_impl, args);
                    let inst = tlas::instance::Instance::new(shape, mtl.clone())
                        .with_transform(self.ctm_stack.last().unwrap().clone());
                    self.instances.push(inst);
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
                // TODO: set current mtl and arealight to None at the exit of an attribute block.
                // Study more PBRT input files and make the decision.
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
                if tex_type == "color" || tex_type == "spectrum" {
                    let tex = self.parse_color_texture(tex_impl, args);
                    self.named_textures.insert(name, tex);
                } else {
                    log::error!("texture of type {tex_type}, args = {:?}", args);
                }
            }
            WorldItem::MaterialInstance(name) => {
                self.current_mtl = self.named_materials.get(&name).cloned();
            }
            WorldItem::Light(light_impl, mut args) => {
                if light_impl == "infinite" {
                    println!("args = {:?}", args);
                    let multiplier = args.extract_substr("L").map(|(key, args)| {
                        if let ArgValue::Numbers(nums) = args {
                            let spectrum_type = key.split(' ').next().unwrap();
                            Self::parse_constant_color(spectrum_type, nums)
                        } else {
                            unimplemented!("Unrecognized luminance in infinite light: {:?}", args);
                        }
                    });
                    let map_image = args.extract_string("string mapname").map(|map_name| {
                        let map_path = self.resolve_relative_path(&map_name);
                        tex::Image::from_file(&map_path).expect("can't read image")
                    });
                    if let Some(image) = map_image {
                        self.env_light =
                            crate::EnvLight::Image(image, multiplier.unwrap_or(Color::ONE));
                    } else if let Some(color) = multiplier {
                        self.env_light = crate::EnvLight::Constant(color);
                    } else {
                        panic!("can't process the infinite light");
                    }
                } else {
                    let delta_light = Self::parse_light(light_impl, args);
                    self.delta_lights.push(delta_light);
                }
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
                Arc::new(shape::Sphere::new(hcm::Point3::ORIGIN, radius))
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
                    "Triangle mesh with {} vertices and {} triangles, bvh shape: {}",
                    mesh.vertices.len(),
                    mesh.index_triples.len(),
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

                let uv_raw =
                    match (parameters.extract("float uv")).or(parameters.extract("float st")) {
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
                let index_triples = indices_raw
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
                if implementation == "loopsubdiv" {
                    let subdivided = shape::loop_subdivide(&points, &index_triples);
                    info!("using loop subdiv");
                    let mut output_file = std::fs::File::create("a.obj").unwrap();
                    output_file
                        .write(subdivided.serialize_as_obj().as_bytes())
                        .unwrap();
                    Arc::new(subdivided)
                } else {
                    let tri_bvh =
                        shape::TriangleMesh::from_soa(points, normals, uvs, index_triples);
                    info!("Triangle bvh summary: {}", tri_bvh.bvh_shape_summary());
                    Arc::new(tri_bvh)
                }
            }
            _ => unimplemented!("shape of {}", implementation),
        }
    }

    /// Creates a list of samplable shapes from the arguments.
    ///
    /// TODO: Currently, trait upcasting coersion is not stable, making it prohibitively difficult
    /// to convert `dyn ShapeSample` to `dyn Sample`. Once this feature is stable, consider
    /// deploying it to reduce memory footprint. https://github.com/rust-lang/rust/issues/65991
    fn parse_samplable_shape(
        &self, shape_impl: &String, args: ast::ParameterSet,
    ) -> (Vec<SamplableShape>, Vec<Arc<dyn Shape>>) {
        if shape_impl == "sphere" {
            let radius = args.lookup_f32("float radius").unwrap_or(1.0);
            let sphere = shape::Sphere::new(hcm::Point3::ORIGIN, radius);
            (vec![SamplableShape::Sphere(sphere)], vec![Arc::new(sphere)])
        } else if shape_impl == "plymesh" {
            let ply_file_name = args
                .lookup_string("string filename")
                .expect("no ply file specified");
            let mut ply_file_path = self.root_dir.clone();
            ply_file_path.push(ply_file_name);

            let mesh = plyloader::load_ply(ply_file_path.to_str().unwrap());
            let mut triangles = Vec::new();
            for ijk in mesh.index_triples.iter() {
                let (i, j, k) = *ijk;
                let t = IsolatedTriangle::new(
                    mesh.vertices[i].pos,
                    mesh.vertices[j].pos,
                    mesh.vertices[k].pos,
                );
                triangles.push(t);
            }
            (
                triangles
                    .iter()
                    .map(|t| SamplableShape::Triangle(*t))
                    .collect(),
                triangles
                    .into_iter()
                    .map(|t| Arc::new(t) as Arc<dyn Shape>)
                    .collect(),
            )
        } else {
            unimplemented!("samplable shape: {}", shape_impl)
        }
    }

    fn parse_light(light_impl: String, mut args: ast::ParameterSet) -> DeltaLight {
        if light_impl == "distant" {
            let from = match args.extract_substr("from") {
                None => hcm::Point3::ORIGIN,
                Some((_key, ArgValue::Numbers(num))) => hcm::Point3::new(num[0], num[1], num[2]),
                _ => panic!("Can't parse 3d point/vector"),
            };
            let to = match args.extract_substr("to") {
                None => hcm::Point3::new(0.0, 0.0, 1.0),
                Some((_key, ArgValue::Numbers(num))) => hcm::Point3::new(num[0], num[1], num[2]),
                _ => panic!("Can't parse 3d point/vector for parameter 'to'"),
            };
            let emit_radiance = match args.extract_substr("L") {
                None => Color::white(),
                Some((key, ArgValue::Numbers(nums))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    Self::parse_constant_color(spectrum_type, nums)
                }
                Some((_, ArgValue::Number(g))) => Color::gray(g),
                Some(_) => panic!("Can't parse radiance"),
            };
            light::DeltaLight::distant(f32::INFINITY, to - from, emit_radiance)
        } else if light_impl == "point" {
            let position = match args.extract_substr("from") {
                None => hcm::Point3::ORIGIN,
                Some((_key, ArgValue::Numbers(num))) => hcm::Point3::new(num[0], num[1], num[2]),
                _ => panic!("Can't parse 3d point/vector"),
            };
            let intensity = match args.extract_substr("L") {
                None => Color::white(),
                Some((key, ArgValue::Numbers(nums))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    Self::parse_constant_color(spectrum_type, nums)
                }
                Some((_, ArgValue::Number(g))) => Color::gray(g),
                Some(_) => panic!("Can't parse intensity"),
            };
            light::DeltaLight::point(position, intensity)
        } else if light_impl == "projection" {
            unimplemented!("light of {}", light_impl)
        } else if light_impl == "spot" {
            unimplemented!("light of {}", light_impl)
        } else {
            unimplemented!("not delta light: {}", light_impl)
        }
    }

    fn parse_material(
        &mut self, mtl_impl: String, mut parameters: ast::ParameterSet,
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
            let eta = match parameters.extract_substr("eta") {
                None => crate::preset::copper_fresnel().0,
                Some((key, ArgValue::Numbers(nums))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    Self::parse_constant_color(spectrum_type, nums)
                }
                Some((_, ArgValue::String(spd_file_path))) => {
                    color_from_spd_file(&self.resolve_relative_path(&spd_file_path))
                }
                Some((_, wtf)) => unimplemented!("metal, eta = {:?}", wtf),
            };
            let k = match parameters.extract_substr("k") {
                None => crate::preset::copper_fresnel().0,
                Some((key, ArgValue::Numbers(nums))) => {
                    let spectrum_type = key.split(' ').next().unwrap();
                    Self::parse_constant_color(spectrum_type, nums)
                }
                Some((_, ArgValue::String(spd_file_path))) => {
                    color_from_spd_file(&self.resolve_relative_path(&spd_file_path))
                }
                Some((_, wtf)) => unimplemented!("metal, k = {:?}", wtf),
            };
            Arc::new(mtl::Metal::from_ior(eta, k, roughness))
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
                u_roughness
            } else {
                error!("UV roughness not supported, using u only: {u_roughness}, {v_roughness}");
                // mtl::Roughness::UV((u_roughness, v_roughness))
                u_roughness
            };
            Arc::new(mtl::Substrate::new(kd, ks, rough, remap_roughness))
        } else if mtl_impl == "fourier" {
            let bsdf_file = parameters
                .lookup_string("string bsdffile")
                .expect("string bsdffile");
            let bsdf_path = self.resolve_relative_path(&bsdf_file);
            Arc::new(mtl::Fourier::from_file(&bsdf_path))
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
    fn parse_transform(t: ast::Transform) -> AffineTransform {
        use ast::Transform;
        match t {
            Transform::Identity => AffineTransform::identity(),
            Transform::Translate(v) => AffineTransform::translater(v),
            Transform::Scale(s) => AffineTransform::scaler(s),
            Transform::Rotate(axis, angle) => {
                // The rotate-around-arbitrary-axis transformation implemented in pbrt-v3 is in fact
                // its inverse (the bases of the rotation matrix are incorrectly stored in row-major
                // order), so to replicate this behavior, the inverse rotation (negated angle) is
                // used instead.
                AffineTransform::rotater(axis, -angle)
            }
            Transform::LookAt(_, _, _) => panic!("unsupported lookat in modeling step"),
            Transform::CoordSys(name) => unimplemented!("coordsys({name})"),
        }
    }
    #[allow(dead_code)]
    fn parse_rbtransform(t: ast::Transform) -> geometry::RigidBodyTransform {
        use ast::Transform;
        use geometry::RigidBodyTransform as RBTrans;
        match t {
            Transform::Identity => RBTrans::identity(),
            Transform::Translate(v) => RBTrans::translater(v),
            Transform::Scale(s) => {
                error!("scaling of {} unsupported", s);
                RBTrans::identity()
            }
            // See parse_transform() for explanation of the negated angle.
            Transform::Rotate(axis, angle) => RBTrans::rotater(axis, -angle),
            Transform::LookAt(..) => panic!("unsupported lookat in modeling step"),
            Transform::CoordSys(name) => unimplemented!("coordsys({name})"),
        }
    }

    #[allow(non_snake_case)]
    fn parse_filter(filter_impl: String, mut args: ParameterSet) -> math::filter::Filter {
        use math::filter::{Filter, UV};
        let default_radius = match filter_impl.as_str() {
            "box" => 0.5,
            "sinc" => 4.0,
            _ => 2.0,
        };
        let xwidth = args.extract_f32("xwidth").unwrap_or(default_radius);
        let ywidth = args.extract_f32("ywidth").unwrap_or(default_radius);
        let radius = UV {
            u: xwidth,
            v: ywidth,
        };

        let filter = if filter_impl == "box" {
            Filter::Box { radius }
        } else if filter_impl == "gaussian" {
            let alpha = args.extract_f32("alpha").unwrap_or(2.0);
            Filter::Gaussian { radius, alpha }
        } else if filter_impl == "mitchell" {
            let B = args.extract_f32("B").unwrap_or(1.0 / 3.0);
            let C = args.extract_f32("C").unwrap_or(1.0 / 3.0);
            Filter::MitchellNetravali { radius, B, C }
        } else if filter_impl == "sinc" {
            let tau = args.extract_f32("tau").unwrap_or(3.0);
            Filter::LanczosSinc { radius, tau }
        } else {
            panic!("Unhandled filter impl: {}", filter_impl)
        };
        if !args.0.is_empty() {
            log::error!("Parameters left for parsing filter: {:?}", args);
        }
        filter
    }
}

pub fn color_from_spd_file(path: &str) -> Color {
    use std::io::BufRead;
    let spd_file = File::open(path).unwrap();
    let mut lambdas_and_values = Vec::new();
    for line in std::io::BufReader::new(spd_file).lines() {
        if let Ok(content) = line {
            if content.trim().starts_with('#') {
                continue;
            }
            let numbers = content
                .split(' ')
                .map(|s| s.parse::<f32>().unwrap())
                .collect::<Vec<_>>();
            assert!(numbers.len() >= 2);
            if numbers.len() > 2 {
                eprintln!("more than 2 numbers in the line: {:?}", numbers);
            }
            lambdas_and_values.push((numbers[0], numbers[1]));
        }
    }
    radiometry::spectrum::sampled_spectrum_to_color(&mut lambdas_and_values)
}
