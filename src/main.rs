mod bsdf;
mod cli_options;
mod directlighting;
mod instance;
mod light;
mod material;
mod scene_loader;
mod texture;
mod tlas;

use io::Write;
use log::*;
use std::f32::consts::PI;
use std::fs::File;
use std::io::{self, BufWriter};
use std::sync::Arc;
use std::time::Instant;

use crate::directlighting::direct_lighting_integrator;
use crate::scene_loader::Scene;
use geometry::{bvh, camera, ray};
use instance::Instance;
use light::EnvLight;
use material as mtl;
use math::hcm::{Point3, Vec3};
use math::{assert_le, float};
use radiometry::color::Color;
use shape::{self, QuadXZ, Sphere};
use texture as tex;
use tlas::BvhNode;

use glog::Flags;
use rayon::prelude::*;

use crate::{camera::Camera, texture::Texture};

const WIDTH: u32 = 1200;
const HEIGHT: u32 = 800;
const MSAA: usize = 2; // 275R: 12x12 = 63.6s, 24x24=252.2s
const SAMPLES_PER_PIXEL: usize = MSAA * MSAA;

fn vec3_to_color(v: Vec3) -> Color {
    Color::new(v.x, v.y, v.z)
}

fn rand_f32() -> f32 {
    rand::random::<f32>()
}

pub fn write_image(file_name: &str, data: &[u8], (width, height): (u32, u32)) {
    assert_eq!(data.len(), (width * height * 3) as usize);

    let file = File::create(file_name).unwrap();
    let ref mut w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width, height);
    encoder.set_color(png::ColorType::RGB);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();

    writer.write_image_data(&data).unwrap();
}

#[allow(unreachable_code)]
fn main() {
    glog::new()
        .init(Flags {
            logtostderr: true,
            colorlogtostderr: true,
            ..Default::default()
        })
        .unwrap();

    // Parses options from the command line arguments: input file or hard-coded scene name, choice
    // of integrator, etc.
    let options = cli_options::parse_args(std::env::args().collect::<Vec<_>>());
    if let Err(message) = &options {
        error!("Can't parse command-line options: {}", message);
    }
    let options = options.unwrap();

    let scene = if let Some(pbrs_file_path) = options.pbrt_file.as_ref() {
        load_pbrt_scene(pbrs_file_path)
    } else {
        match options.scene_name.as_ref().map(|n| n.as_str()) {
            Some("125_spheres") => scene_125_spheres(),
            Some("two_perlin_spheres") => scene_two_perlin_spheres(),
            Some("earth") => scene_earth(),
            Some("quad_light") => scene_quad_light(),
            Some("quad") => scene_quad(),
            Some("cornell_box") => scene_cornell_box(),
            Some("plates") => scene_plates(),
            Some("everything") => scene_everything(),
            None | Some(_) => {
                error!("No scene file or name specified. Abort.");
                eprintln!(
                    "Available scenes: 125_spheres | two_perlin_spheres | earth | quad_light \
                           | quad | cornell_box | everything"
                );
                std::process::exit(1);
            }
        }
    };

    let integrator = match options.integrator {
        cli_options::Integrator::Direct => direct_lighting_integrator,
        cli_options::Integrator::Path => path_integrator,
    };

    info!(
        "building bvh success: {}, height = {}",
        scene.tlas.geometric_sound(),
        scene.tlas.height()
    );

    let (width, height) = scene.camera.resolution();
    // Function lambda to render one row. It's going to be used in the main rendering process.
    let render_one_row = |row| {
        if (row % 10) == 0 {
            print!("{} ", row);
            io::stdout().flush().unwrap();
        }
        let mut colors_for_row = vec![];
        for col in 0..width {
            let mut color_sum = Color::black();

            for i in 0..MSAA * MSAA {
                let jitter = (
                    ((i / MSAA) as f32 + rand::random::<f32>()) / MSAA as f32,
                    ((i % MSAA) as f32 + rand::random::<f32>()) / MSAA as f32,
                );
                // let jitter = (rand::random::<f32>(), rand::random::<f32>());
                let ray = scene.camera.shoot_ray(row, col, jitter).unwrap();
                // color_sum = color_sum + dummy_integrator(&bvh, ray, 1, env_light);
                color_sum = color_sum + integrator(&scene, ray, 5);
            }

            let color = color_sum / (SAMPLES_PER_PIXEL as f32);
            colors_for_row.push(color);
        }
        colors_for_row
    };

    // Initiates rendering. Based on command line options, the process may or may not use
    // multi-threading.
    let start_render = Instant::now();

    let image_map: Vec<_> = if options.use_multi_thread {
        (0..height)
            .into_par_iter()
            .map(render_one_row)
            .flatten()
            .collect()
    } else {
        (0..height)
            .into_iter()
            .map(render_one_row)
            .flatten()
            .collect()
    };

    // Collects image data as a huge array of `Color`.
    let image_data: Vec<_> = image_map
        .iter()
        .map(|color| color.gamma_encode().to_u8().to_vec())
        .flatten()
        .collect();

    let whole_render_time = Instant::now().duration_since(start_render);
    println!("whole render time = {:?}", whole_render_time);

    // Builds the file name using scene name and SPP, and writes the resulting image to a file.
    let scene_name = match (options.scene_name, options.pbrt_file) {
        (Some(name), None) => name,
        (None, Some(path)) => std::path::Path::new(&path)
            .file_stem()
            .map(|s| s.to_owned().into_string())
            .unwrap()
            .unwrap(),
        _ => "output".to_owned(),
    };

    let output_file_name = format!(
        "{}-{}-{}spp.png",
        scene_name,
        options.integrator.to_str(),
        MSAA.pow(2)
    );
    println!("Image written to {}", output_file_name);
    write_image(&output_file_name, &image_data, scene.camera.resolution());
}

// Functions that computes the radiance along a ray. One for computing the radiance correctly and
// another one for debugging purposes.
// ------------------------------------------------------------------------------------------------

fn path_integrator(scene: &Scene, mut ray: ray::Ray, depth: i32) -> Color {
    if depth <= 0 {
        return Color::black();
    }

    let hit_info = scene.tlas.intersect(&mut ray);

    match hit_info {
        None => scene.env_light.map(|l| l(ray)).unwrap_or(Color::black()),
        Some((hit, mtl)) => {
            if !mtl.emission().is_black() {
                // Stops recursive path tracing if the material is emissive, and
                // returns the emitted radiance.
                mtl.emission()
            } else {
                let (scattered_ray, attenuation) = mtl.scatter(-ray.dir, &hit);
                let in_radiance = path_integrator(scene, scattered_ray, depth - 1);
                attenuation * in_radiance
            }
        }
    }
}

#[allow(dead_code)]
fn debug_pt(scene: &Box<BvhNode>, mut ray: ray::Ray, depth: i32, env_light: EnvLight) -> Color {
    if depth <= 0 {
        return Color::black();
    }

    let hit_info = scene.intersect(&mut ray);

    match hit_info {
        None => env_light(ray),
        Some((hit, mtl)) => {
            assert_le!(hit.pos.distance_to(ray.position_at(hit.ray_t)), 0.001);
            if !mtl.emission().is_black() {
                // Stops recursive path tracing if the material is emissive, and
                // returns the emitted radiance.
                mtl.emission()
            } else {
                let (scattered_ray, attenuation) = mtl.scatter(-ray.dir, &hit);
                println!(
                    "depth {}, hit = {}, scat_ray = {}",
                    depth, hit, scattered_ray
                );
                let incident_radiance = debug_pt(scene, scattered_ray, depth - 1, env_light);
                println!(
                    "attenuation = {:?}, Li = {:?}",
                    attenuation, incident_radiance
                );
                attenuation * incident_radiance
            }
        }
    }
}

#[allow(dead_code)]
fn dummy_integrator(scene: &Box<BvhNode>, mut ray: ray::Ray, _: i32, env_light: EnvLight) -> Color {
    let hit_info = scene.intersect(&mut ray);

    match hit_info {
        None => env_light(ray),
        Some((hit, mtl)) => {
            let (_, albedo) = mtl.scatter(-ray.dir, &hit);
            (albedo + vec3_to_color(hit.normal)) * 0.5
            // albedo
            // (Color::from(hit.normal) + Color::white()) * 0.5
        }
    }
}

// Functions that build the scenes: environment light and collection of objects.
// ------------------------------------------------------------------------------------------------

fn blue_sky(r: ray::Ray) -> Color {
    let bg_color_top = Color::new(0.5, 0.7, 1.0);
    let bg_color_bottom = Color::white();
    let y = (r.dir.hat().y + 1.0) * 0.5;
    bg_color_top * y + bg_color_bottom * (1.0 - y)
}

fn dark_room(r: ray::Ray) -> Color {
    let bg_color_top = Color::gray(0.0);
    let bg_color_bottom = Color::gray(0.0);
    let y = (r.dir.hat().y + 1.0) * 0.5;
    bg_color_top * y + bg_color_bottom * (1.0 - y)
}

#[allow(dead_code)]
#[allow(unused_mut)]
fn scene_125_spheres() -> Scene {
    let mut camera = camera::Camera::new((WIDTH, HEIGHT), math::new_deg(25.0));
    camera.look_at(
        Point3::new(13.0, 2.0, 3.0),
        Point3::new(0.0, 0.0, 0.0),
        Vec3::ybase(),
    );

    let mut spheres = vec![
        Sphere::from_raw((0.0, -1000.0, 1.0), 1000.0),
        Sphere::from_raw((0.0, 1.0, 0.0), 1.0),
        Sphere::from_raw((-4.0, 1.0, 0.0), 1.0),
        Sphere::from_raw((4.0, 1.0, 0.0), 1.0),
    ];

    let mut mtls: Vec<Arc<dyn mtl::Material>> = vec![
        Arc::new(mtl::Lambertian::solid(Color::new(0.5, 0.5, 0.5))),
        Arc::new(mtl::Dielectric::new(1.5)),
        Arc::new(mtl::Lambertian::solid(Color::new(0.4, 0.2, 0.1))),
        Arc::new(mtl::Metal::new(Color::new(0.7, 0.6, 0.5), 0.0)),
    ];

    // for a in -11..11 {
    //     for b in -11..11 {
    //         let choose_mtl = rand_f32();
    //         let center =
    //             Point3::new(a as f32, 0.2, b as f32) + 0.9 * Vec3::new(rand_f32(), 0.0, rand_f32());

    //         if (center - Point3::new(4.0, 0.2, 0.0)).norm() > 0.9 {
    //             spheres.push(Sphere::new(center, 0.2));
    //             let mtl: Arc<dyn mtl::Material> = if choose_mtl < 0.8 {
    //                 let albedo = Color::new(rand_f32(), rand_f32(), rand_f32());
    //                 Arc::new(mtl::Lambertian::solid(albedo))
    //             } else if choose_mtl < 0.95 {
    //                 let albedo = Color::new(rand_f32(), rand_f32(), rand_f32());
    //                 let albedo = (albedo + Color::white()) * 0.5;
    //                 Arc::new(mtl::Metal::new(albedo, rand_f32() * 0.5))
    //             } else {
    //                 Arc::new(mtl::Dielectric::new(1.4))
    //             };
    //             mtls.push(mtl);
    //         }
    //     }
    // }

    let spheres: Vec<_> = spheres.into_iter().map(|s| Arc::new(s)).collect();

    let instances: Vec<Instance> = mtls
        .into_iter()
        .zip(spheres.into_iter())
        .map(|(mtl, sphere)| Instance::new(sphere, mtl))
        .collect();

    let boxed_instances: Vec<Box<Instance>> =
        instances.iter().map(|x| Box::from(x.clone())).collect();
    Scene::new(*tlas::build_bvh(boxed_instances), camera, blue_sky)
}

fn scene_two_perlin_spheres() -> Scene {
    let perlin_tex = tex::Perlin::with_freq(4.0);

    let (mut cmin, mut cmax) = (0.0f32, 0.0f32);
    for _ in 0..1_000_000 {
        let p = Point3::new(rand_f32(), rand_f32(), rand_f32());
        let c = perlin_tex.value((0.0, 0.0), p).r;
        cmin = cmin.min(c);
        cmax = cmax.max(c);
    }
    println!("perlin min, max = {} {}", cmin, cmax);
    // std::process::exit(0);

    let mtl = Arc::new(mtl::Lambertian::textured(Arc::new(perlin_tex)));

    let shapes = vec![
        Arc::new(Sphere::from_raw((0.0, -1000.0, 0.0), 1000.0)),
        Arc::new(Sphere::from_raw((0.0, 2.0, 0.0), 2.0)),
    ];

    let instances = shapes
        .into_iter()
        .map(|sphere| Box::new(Instance::new(sphere, mtl.clone())))
        .collect();
    let mut cam = Camera::new((WIDTH, HEIGHT), math::new_deg(20.0));
    cam.look_at(
        Point3::new(13.0, 2.0, -3.0),
        Point3::origin(),
        Vec3::ybase(),
    );

    Scene::new(*tlas::build_bvh(instances), cam, blue_sky)
}

fn scene_earth() -> Scene {
    let earth_tex = Arc::new(tex::Image::from_file("assets/earthmap.png").unwrap());
    let earth_mtl = Arc::new(material::Lambertian::textured(earth_tex));

    let globe = Arc::new(Sphere::new(Point3::origin(), 2.0));
    let instances = vec![Box::new(Instance::new(globe, earth_mtl))];

    let mut cam = Camera::new((WIDTH, HEIGHT), math::new_deg(20.0));
    cam.look_at(
        Point3::new(13.0, 2.0, -3.0),
        Point3::origin(),
        Vec3::ybase(),
    );

    Scene::new(*tlas::build_bvh(instances), cam, blue_sky)
}

fn scene_quad_light() -> Scene {
    let perlin_tex = Arc::new(tex::Perlin::with_freq(4.0));

    let mtl = Arc::new(mtl::Lambertian::textured(perlin_tex));
    let light_power = Color::gray(4.0);
    let light = Arc::new(mtl::DiffuseLight::new(light_power));

    let shapes = vec![
        Arc::new(Sphere::from_raw((0.0, -1000.0, 0.0), 1000.0)),
        Arc::new(Sphere::from_raw((0.0, 2.0, 0.0), 2.0)),
    ];

    let mut instances: Vec<_> = shapes
        .into_iter()
        .map(|sphere| Box::new(Instance::new(sphere, mtl.clone())))
        .collect();

    let light_quad = shape::QuadXY::from_raw((3.0, 5.0), (1.0, 3.0), 2.1);
    let light_sphere = Sphere::from_raw((0.0, 7.0, 0.0), 2.0);
    instances.extend(vec![
        Box::new(Instance::new(Arc::new(light_quad.clone()), light.clone())),
        Box::new(Instance::new(Arc::new(light_sphere.clone()), light.clone())),
    ]);
    let area_lights = vec![
        light::DiffuseAreaLight::new(light_power, Box::new(light_quad.clone())),
        light::DiffuseAreaLight::new(light_power, Box::new(light_sphere.clone())),
    ];

    let mut cam = Camera::new((WIDTH, HEIGHT), math::new_deg(20.0));
    cam.look_at(
        Point3::new(26.0, 3.0, -6.0),
        Point3::new(0.0, 2.0, 0.0),
        Vec3::ybase(),
    );

    Scene::new(*tlas::build_bvh(instances), cam, dark_room).with_lights(vec![], area_lights)
}

fn scene_quad() -> Scene {
    let xy_quad = shape::QuadXY::from_raw((-0.5, 0.5), (-0.3, 0.6), 2.5);
    let lam = Arc::new(mtl::Lambertian::solid(Color::new(0.2, 0.3, 0.7)));
    let instances = vec![Box::new(Instance::new(Arc::new(xy_quad), lam))];

    let cam = Camera::new((WIDTH, HEIGHT), math::new_deg(45.0));

    Scene::new(*tlas::build_bvh(instances), cam, blue_sky)
}

fn scene_cornell_box() -> Scene {
    let red = mtl::Lambertian::solid(Color::new(0.65, 0.05, 0.05));
    let white = mtl::Lambertian::solid(Color::gray(0.73));
    let green = mtl::Lambertian::solid(Color::new(0.12, 0.45, 0.15));
    let light_color = Color::gray(15.0);
    let light = mtl::DiffuseLight::new(light_color);

    let red = Arc::new(red);
    let white = Arc::new(white);
    let green = Arc::new(green);
    let light = Arc::new(light);

    let area_lights = vec![light::DiffuseAreaLight::new(
        light_color,
        Box::new(shape::QuadXZ::from_raw(
            (213.0, 343.0),
            (227.0, 332.0),
            554.0,
        )),
    )];

    let shapes: Vec<Arc<dyn shape::Shape>> = vec![
        Arc::new(shape::QuadYZ::from_raw((0.0, 555.0), (0.0, 555.0), 555.0)), // green
        Arc::new(shape::QuadYZ::from_raw((0.0, 555.0), (0.0, 555.0), 0.0)),   // red
        Arc::new(shape::QuadXZ::from_raw(
            (213.0, 343.0),
            (227.0, 332.0),
            554.0,
        )), // light
        Arc::new(shape::QuadXZ::from_raw((0.0, 555.0), (0.0, 555.0), 0.0)),   // white floor
        Arc::new(shape::QuadXZ::from_raw((0.0, 555.0), (0.0, 555.0), 555.0)), // white ceiling
        Arc::new(shape::QuadXY::from_raw((0.0, 555.0), (0.0, 555.0), 555.0)), // white back
        Arc::new(shape::Cuboid::from_points(
            Point3::origin(),
            Point3::new(165.0, 165.0, 165.0),
        )),
        Arc::new(shape::Cuboid::from_points(
            Point3::origin(),
            Point3::new(165.0, 330.0, 165.0),
        )),
        // Arc::new(Sphere::from_raw((250.0, 250.0, 250.0), 50.0))
    ];

    let mtl_seq: Vec<Arc<dyn mtl::Material>> =
        // vec![light];
        vec![red, green, light, white.clone(), white.clone(), white.clone(), white.clone(), white.clone()];

    let mut instances: Vec<_> = shapes
        .into_iter()
        .zip(mtl_seq.into_iter())
        .map(|(shape, mtl)| Box::new(Instance::new(shape, mtl)))
        .collect();
    instances[6].transform = instance::identity()
        .rotate_y(math::new_deg(15.0))
        .translate(Vec3::new(265.0, 0.0, 105.0));
    instances[7].transform = instance::identity()
        .rotate_y(math::new_deg(-18.0))
        .translate(Vec3::new(130.0, 0.0, 225.0));

    println!("{:?}, {:?}", instances[6].transform, instances[7].transform);
    // std::process::exit(0);

    let mut cam = Camera::new((600, 600), math::new_deg(40.0));
    cam.look_at(
        Point3::new(278.0, 278.0, -800.0),
        Point3::new(278.0, 278.0, 0.0),
        Vec3::ybase(),
    );

    Scene::new_no_envlight(*tlas::build_bvh(instances), cam).with_lights(vec![], area_lights)
}

fn scene_plates() -> Scene {
    let mut instances = vec![];
    let r = 20.0;
    // Builds the background.
    let wall = shape::QuadXY::from_raw((-r, r), (0.0, r), r);
    let floor = shape::QuadXZ::from_raw((-r, r), (0.0, r), 0.0);
    let matte = mtl::Lambertian::solid(Color::gray(0.4));

    let wall_instance = Instance::from_raw(wall, matte.clone());
    let floor_instance = Instance::from_raw(floor, matte);
    instances.push(wall_instance);
    instances.push(floor_instance);

    // axis: y = 10, z = 0
    let (angles, spacing) = float::linspace((-PI * 0.4, -PI * 0.05), 4);
    let delta_angle = spacing * 0.65;
    let (left, right) = (-r * 0.68, r * 0.68);
    let half_width = delta_angle * r * 0.5;
    for angle in angles.iter() {
        // let glossy = mtl::Glossy::new(Color::gray(0.5), angle * -1.0);
        let glossy = mtl::Lambertian::solid(Color::rgb(80, 180, 50));
        let plate = QuadXZ::from_raw((left, right), (-half_width, half_width), 4.0);
        let trans = instance::identity()
            .translate(Vec3::new(0.0, -r, 0.0))
            .rotate_x(math::new_rad(-angle))
            .translate(Vec3::new(0.0, r * 0.8, 0.0));
        instances.push(Instance::from_raw(plate, glossy).with_transform(trans));
    }
    let instances: Vec<_> = instances.into_iter().map(|i| Box::new(i)).collect();

    let camera = camera::Camera::new((800, 800), math::Angle::pi() * 0.19).looking_at(
        Point3::new(0.0, r * 0.9, -r * 2.0),
        Point3::new(0.0, r * 0.2, r * 2.0),
        Vec3::ybase(),
    );

    Scene::new(*tlas::build_bvh(instances), camera, blue_sky)
}

fn scene_everything() -> Scene {
    let ground = Arc::new(mtl::Lambertian::solid(Color::new(0.48, 0.83, 0.53)));

    const BOXES_PER_SIDE: i32 = 20;

    let mut instances = Vec::<Instance>::new();
    for i in 0..BOXES_PER_SIDE {
        for j in 0..BOXES_PER_SIDE {
            let x0 = -1000.0 + i as f32 * 100.0;
            let z0 = -1000.0 + j as f32 * 100.0;

            let x1 = x0 + 100.0;
            let y1 = rand_f32() * 100.0 + 1.0;
            let z1 = z0 + 100.0;

            instances.push(Instance::new(
                Arc::new(shape::Cuboid::from_points(
                    Point3::new(x0, 0.0, z0),
                    Point3::new(x1, y1, z1),
                )),
                ground.clone(),
            ));
        }
    }

    let light = mtl::DiffuseLight::new(Color::gray(7.0));
    let light_quad = shape::QuadXZ::from_raw((123.0, 423.0), (147.0, 412.0), 554.0);
    let area_lights = vec![light::DiffuseAreaLight::new(
        Color::gray(7.0),
        Box::new(light_quad.clone()),
    )];

    instances.push(Instance::from_raw(light_quad, light));

    let glass_ball = Sphere::from_raw((260.0, 150.0, 45.0), 50.0);
    let glass_mtl = mtl::Dielectric::new(1.5);
    instances.push(Instance::from_raw(glass_ball, glass_mtl));

    let metal_ball = Sphere::from_raw((0.0, 150.0, 145.0), 50.0);
    let metal_mtl = mtl::Metal::new(Color::new(0.8, 0.8, 0.9), 1.0);
    // instances.push(Instance::new(Arc::new(metal_ball), Arc::new(metal_mtl)));
    instances.push(Instance::from_raw(metal_ball, metal_mtl));

    let boundary_ball = Sphere::from_raw((360.0, 150.0, 145.0), 70.0);
    let glass_mtl = mtl::Dielectric::new(1.5);
    instances.push(Instance::from_raw(boundary_ball, glass_mtl));

    let earth_map = tex::Image::from_file("assets/earthmap.png").unwrap();
    let earth_mtl = mtl::Lambertian::textured(Arc::new(earth_map));
    let earth_shape = Sphere::from_raw((400.0, 200.0, 400.0), 100.0);
    instances.push(Instance::from_raw(earth_shape, earth_mtl));

    let perlin_tex = tex::Perlin::with_freq(10.0);
    let matte_perlin = mtl::Lambertian::textured(Arc::new(perlin_tex));
    let noise_ball = Sphere::from_raw((220.0, 280.0, 300.0), 80.0);
    instances.push(Instance::from_raw(noise_ball, matte_perlin));

    let matte_white = mtl::Lambertian::solid(Color::gray(0.73));

    let rand_165 = || rand_f32() * 165.0;
    let ping_pong_balls: Vec<_> = (0..1000)
        .map(|_| Sphere::from_raw((rand_165(), rand_165(), rand_165()), 10.0))
        .collect();
    let ping_pong_balls = shape::IsoBlas::build(ping_pong_balls);
    let pp_trans = instance::identity()
        .rotate_y(math::new_deg(15.0))
        .translate(Vec3::new(-100.0, 270.0, 395.0));
    instances.push(Instance::from_raw(ping_pong_balls, matte_white).with_transform(pp_trans));

    let mut cam = camera::Camera::new((800, 800), math::new_deg(40.0));
    cam.look_at(
        Point3::new(478.0, 278.0, -600.0),
        Point3::new(278.0, 278.0, 0.0),
        Vec3::ybase(),
    );

    let instances: Vec<_> = instances.into_iter().map(|i| Box::new(i)).collect();

    Scene::new(*tlas::build_bvh(instances), cam, dark_room).with_lights(vec![], area_lights)
}

fn load_pbrt_scene(pbrt_file_path: &str) -> Scene {
    let pbrt_scene = scene_loader::build_scene(pbrt_file_path);
    let cam = pbrt_scene.camera.expect("camera not built in the scene");
    let tlas = pbrt_scene
        .instances
        .into_iter()
        .map(|i| Box::new(i))
        .collect::<Vec<_>>();
    Scene::new(*tlas::build_bvh(tlas), cam, blue_sky)
        .with_lights(pbrt_scene.delta_lights, pbrt_scene.area_lights)
}

// Monte-carlo playground
// -------------------------------------------------------------------------------------------------

fn rand_f32_uniform(a: f32, b: f32) -> f32 {
    rand_f32() * (b - a) + a
}
fn rand_f64_uniform(a: f64, b: f64) -> f64 {
    rand::random::<f64>() * (b - a) + a
}

#[allow(dead_code)]
fn estimate_pi() {
    let sqrt_n = 4000i32;
    let mut count = 0;
    let mut stratified_count = 0;
    for trial in 0..sqrt_n.pow(2) {
        let x = rand_f32_uniform(-1.0, 1.0);
        let y = rand_f32_uniform(-1.0, 1.0);
        count += if x * x + y * y < 1.0 { 1 } else { 0 };

        let (i, j) = (trial / sqrt_n, trial % sqrt_n);
        let x = 2.0 * ((i as f32 + rand_f32()) / sqrt_n as f32) - 1.0;
        let y = 2.0 * ((j as f32 + rand_f32()) / sqrt_n as f32) - 1.0;
        // println!("stratified x, y = {}, {}", x, y);
        stratified_count += if x * x + y * y < 1.0 { 1 } else { 0 };
    }
    let tries = sqrt_n.pow(2);
    println!(
        "#{:12}, Estimate of Pi = {:.10}, stratified = {:.10}",
        tries,
        4.0 * count as f32 / tries as f32,
        4.0 * stratified_count as f32 / tries as f32,
    );
}

#[allow(dead_code)]
fn play_integrator() {
    let integrand = |x| x * x;
    let pdf = |x| x * x * 3.0 / 8.0;
    let sample_mapper = |x| x;
    let integral_result = integrate(integrand, pdf, sample_mapper, (0.0, 2.0));
    println!("area(x*x, 0, 2) = {:10}", integral_result);

    let cosine_squared = |v: Vec3| v.z * v.z;
    let uniform_sphere_pdf = |_: Vec3| std::f32::consts::FRAC_1_PI * 0.25;
    // let identity_vec3_mapper = |v: Vec3| v;

    let integral_result = spherical_integrate(cosine_squared, uniform_sphere_pdf, |v| v);
    println!("spherical_integrate(cos^2(theta)) = {:10}", integral_result);
}

fn integrate<F, F1, F2>(integrand: F, pdf: F1, sample_mapper: F2, interval: (f64, f64)) -> f64
where
    F: Fn(f64) -> f64,
    F1: Fn(f64) -> f64,
    F2: Fn(f64) -> f64,
{
    let tries = 5_000;
    let (lb, ub) = interval;
    let sum: f64 = (0..tries)
        .map(|_| {
            let x = rand_f64_uniform(lb, ub);
            let x = sample_mapper(x);
            // integrand(x)
            // let x = sample_mapper(rand_f64());
            integrand(x) / pdf(x)
        })
        .sum();

    sum / tries as f64
}

fn spherical_integrate<F, F1, M>(integrand: F, pdf: F1, sample_mapper: M) -> f32
where
    F: Fn(Vec3) -> f32,
    F1: Fn(Vec3) -> f32,
    M: Fn(Vec3) -> Vec3,
{
    let tries = 252_000;
    let sum: f32 = (0..tries)
        .map(|_| {
            let x = mtl::uniform_sphere();
            let x = sample_mapper(x);
            integrand(x) / pdf(x)
        })
        .sum();

    sum / tries as f32
}
