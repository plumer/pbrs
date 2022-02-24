mod bsdf;
mod cli_options;
mod directlighting;
mod pathintegrator;

use io::Write;
use itertools::Itertools;
use log::*;
use std::fs::File;
use std::io::{self, BufWriter};
use std::time::Instant;

use crate::directlighting::*;
use geometry::ray;
use light::EnvLight;
use material as mtl;
use math::{assert_le, hcm, hcm::Vec3};
use radiometry::color::Color;
use scene::{preset, Scene};
use tlas::bvh::BvhNode;

use rayon::prelude::*;

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

pub fn write_exr(file_name: &str, colors: &[Color], (width, height): (u32, u32)) {
    exr::prelude::write_rgb_file(
        file_name,
        width as usize,
        height as usize, // write an image with 2048x2048 pixels
        |x, y| {
            let c = colors[y * width as usize + x];
            (c.r, c.g, c.b)
        },
    )
    .unwrap();
}

#[allow(unreachable_code)]
fn main() {
    env_logger::init();

    // Parses options from the command line arguments: input file or hard-coded scene name, choice
    // of integrator, etc.
    let options = cli_options::parse_args(std::env::args().collect::<Vec<_>>());
    if let Err(message) = &options {
        error!("Can't parse command-line options: {}", message);
    }
    let options = options.unwrap();
    let msaa = options.msaa;
    let scene = if let Some(pbrs_file_path) = options.pbrt_file.as_ref() {
        Scene::from_loader(scene::loader::build_scene(pbrs_file_path))
    } else {
        match options.scene_name.as_ref().map(|n| n.as_str()) {
            Some("125_spheres") => preset::mixed_spheres(),
            Some("two_perlin_spheres") => preset::two_perlin_spheres(),
            Some("earth") => preset::earth(),
            Some("quad_light") => preset::quad_light(),
            Some("quad") => preset::quad(),
            Some("cornell_box") => preset::cornell_box(),
            Some("plates") => preset::plates(),
            Some("everything") => preset::everything(),
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
    let scene_name = match (options.scene_name, options.pbrt_file) {
        (Some(name), None) => name,
        (None, Some(path)) => std::path::Path::new(&path)
            .file_stem()
            .map(|s| s.to_owned().into_string())
            .unwrap()
            .unwrap(),
        _ => "output".to_owned(),
    };

    info!(
        "building bvh success: {}, height = {}",
        scene.tlas.geometric_sound(),
        scene.tlas.height()
    );
    info!(
        "num of lights: {} delta, {} area, {} env",
        scene.delta_lights.len(),
        scene.area_lights.len(),
        scene.has_env_light()
    );

    if false {
        let (row, col) = (505, 514);
        let mut colors = vec![];
        for i in 0..msaa * msaa {
            let jitter = (
                ((i / msaa) as f32 + rand::random::<f32>()) / msaa as f32,
                ((i % msaa) as f32 + rand::random::<f32>()) / msaa as f32,
            );
            let ray = scene.camera.shoot_ray(row, col, jitter).unwrap();
            let sample_color = directlighting::direct_lighting_debug_integrator(&scene, ray, 2);
            assert!(!sample_color.has_nan());
            colors.push(sample_color);
        }
        let black_count = colors.iter().filter(|c| c.is_black()).count();
        info!("black colors = {}/{}", black_count, msaa * msaa);
        let nonblacks = colors
            .into_iter()
            .filter(|c| !c.is_black())
            .collect::<Vec<_>>();
        info!("remaining color average = {}", Color::average(&nonblacks));
        info!("Pixel debug at {:?}, exiting now", (row, col));
        // std::process::exit(1);
    }

    if false {
        info!("debugging a specific ray");
        let mut ray = ray::Ray::new(
            hcm::point3(400.0, 20.0, 30.0),
            hcm::vec3(-1.025, 0.018, -0.108),
        );
        scene.tlas.intersect(&mut ray);
        std::process::exit(1);
    }

    let integrator = match options.integrator {
        cli_options::Integrator::Direct => direct_lighting_integrator,
        cli_options::Integrator::Path => pathintegrator::path_integrator,
    };
    let (width, height) = scene.camera.resolution();

    // Produces visualizations if specified.
    let visualize = |visualizer: fn(&Scene, ray::Ray, i32) -> Color, what: &str| {
        let visualized_image = ((0..height).cartesian_product(0..width))
            .map(|(row, col)| {
                let ray = scene.camera.shoot_ray(row, col, (0.0, 0.0)).unwrap();
                visualizer(&scene, ray, 0).gamma_encode().to_u8()
            })
            .flatten()
            .collect::<Vec<_>>();
        write_image(
            &format!("{}-{}-vis.png", scene_name, what),
            &visualized_image,
            (width, height),
        );
    };

    if options.visualize_normals {
        visualize(normal_visualizer, "normal");
    }
    if options.visualize_materials {
        visualize(material_visualizer, "mtl");
    }

    // Function lambda to render one row. It's going to be used in the main rendering process.
    let render_one_row = |row| {
        if (row % 10) == 0 {
            print!("{} ", row);
            io::stdout().flush().unwrap();
        }
        let mut colors_for_row = vec![];
        for col in 0..width {
            let mut color_sum = Color::black();

            for i in 0..msaa * msaa {
                let jitter = (
                    ((i / msaa) as f32 + rand::random::<f32>()) / msaa as f32,
                    ((i % msaa) as f32 + rand::random::<f32>()) / msaa as f32,
                );
                // let jitter = (rand::random::<f32>(), rand::random::<f32>());
                let ray = scene.camera.shoot_ray(row, col, jitter).unwrap();
                // color_sum = color_sum + dummy_integrator(&bvh, ray, 1, env_light);
                color_sum = color_sum + integrator(&scene, ray, 5);
            }

            let color = color_sum.scale_down_by(msaa * msaa);
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

    exr::prelude::write_rgb_file(
        "test.exr",
        width as usize,
        height as usize, // write an image with 2048x2048 pixels
        |x, y| {
            let c = image_map[y * width as usize + x];
            (c.r, c.g, c.b)
        },
    )
    .unwrap();
    // Collects image data as a huge array of `Color`.
    let image_data: Vec<_> = image_map
        .iter()
        .map(|color| color.gamma_encode().to_u8().to_vec())
        .flatten()
        .collect();

    let whole_render_time = Instant::now().duration_since(start_render);
    println!("whole render time = {:?}", whole_render_time);

    // Builds the file name using scene name and SPP, and writes the resulting image to a file.
    let output_file_name = format!(
        "{}-{}-{}spp.exr",
        scene_name,
        options.integrator.to_str(),
        msaa.pow(2)
    );
    println!("Image written to {}", output_file_name);
    write_exr(&output_file_name, &image_map, scene.camera.resolution());
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
        None => scene.eval_env_light(ray),
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

#[test]
fn metal_test() {
    let observe_metal = |name: &str, real_spd_path: &str, imag_spd_path: &str| {
        let eta_real = scene::loader::color_from_spd_file(real_spd_path);
        let eta_imag = scene::loader::color_from_spd_file(imag_spd_path);
        println!("{} eta: {:.6} + {:.6}i", name, eta_real, eta_imag);

        let metal_fresnel = geometry::bxdf::Fresnel::conductor(eta_real, eta_imag);

        for cos_theta in [0.9, 0.8, 0.7, 0.6, 0.5].iter() {
            println!(
                "{} at cos_theta = {}, refl = {:.5}",
                name,
                cos_theta,
                metal_fresnel.eval(*cos_theta)
            );
        }
    };
    observe_metal(
        "Silver",
        "assets/metals/Ag.eta.spd",
        "assets/metals/Ag.k.spd",
    );
    observe_metal(
        "Aluminium",
        "assets/metals/Al.eta.spd",
        "assets/metals/Al.k.spd",
    );
    observe_metal("Gold", "assets/metals/Au.eta.spd", "assets/metals/Au.k.spd");
    observe_metal(
        "Chromium",
        "assets/metals/Cr.eta.spd",
        "assets/metals/Cr.k.spd",
    );
    observe_metal(
        "Copper",
        "assets/metals/Cu.eta.spd",
        "assets/metals/Cu.k.spd",
    );
    observe_metal(
        "Mercury",
        "assets/metals/Hg.eta.spd",
        "assets/metals/Hg.k.spd",
    );
}
