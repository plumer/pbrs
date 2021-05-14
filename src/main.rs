mod bvh;
mod camera;
mod float;
mod hcm;
mod image;
mod instance;
mod material;
mod ray;
mod shape;
mod texture;

use rayon::prelude::*;
use std::io::{self, BufWriter};
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;
use std::{fs::File, mem::size_of};

use hcm::{Point3, Vec3};
use image::Color;
use instance::Instance;
use io::Write;
use material as mtl;
use texture as tex;

use crate::{bvh::{build_bvh, BvhNode}, camera::Camera, texture::Texture};

const WIDTH: u32 = 1200;
const HEIGHT: u32 = 800;
const MSAA: usize = 12;
const SAMPLES_PER_PIXEL: usize = MSAA * MSAA;

type EnvLight = fn(ray::Ray) -> Color;
type Scene = (Box<BvhNode>, camera::Camera, EnvLight);

impl From<Vec3> for Color {
    fn from(v: Vec3) -> Self {
        Color::new(v.x, v.y, v.z)
    }
}

fn rand_f32() -> f32 {
    rand::random::<f32>()
}

pub fn write_image(file_name: &str, data: &[u8], (width, height): (u32, u32)) {
    assert_eq!(data.len(), (width * height * 3) as usize);

    let path = Path::new(file_name);
    let file = File::create(path).unwrap();
    let ref mut w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, width, height);
    encoder.set_color(png::ColorType::RGB);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();

    writer.write_image_data(&data).unwrap();
}

fn main() {
    println!(
        "Hello, world! BvhNode size = {}, BBox size = {}",
        size_of::<BvhNode>(),
        size_of::<bvh::BBox>()
    );
    let half_right_angle = hcm::Degree(45.0);
    println!("{} is {} ", half_right_angle, half_right_angle.to_radian());

    // Prepares the scene and environmental lighting.
    let (bvh, camera, env_light) = scene_cornell_box();
    println!(
        "building bvh success: {}, height = {}",
        bvh.geometric_sound(),
        bvh.height()
    );
    let (width, height) = camera.resolution();

    let start_render = Instant::now();

    let image_map: Vec<_> = (0..height)
        .into_par_iter()
        .map(|row| {
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
                    let ray = camera.shoot_ray(row, col, jitter).unwrap();
                    // color_sum = color_sum + dummy_integrator(&bvh, ray, 3, env_light);
                    color_sum = color_sum + path_integrator(&bvh, ray, 50, env_light);
                }

                let color = color_sum / (SAMPLES_PER_PIXEL as f32);
                colors_for_row.push(color);
            }
            colors_for_row
        })
        .flatten()
        .collect();

    let image_data: Vec<_> = image_map
        .iter()
        .map(|color| color.gamma_encode().to_u8().to_vec())
        .flatten()
        .collect();

    let whole_render_time = Instant::now().duration_since(start_render);

    println!("whole render time = {} us", whole_render_time.as_micros());

    write_image(r"output.png", &image_data, camera.resolution());
}

// Functions that computes the radiance along a ray. One for computing the radiance correctly and
// another one for debugging purposes.
// ------------------------------------------------------------------------------------------------

fn path_integrator(scene: &Box<BvhNode>, mut ray: ray::Ray, depth: i32, env_light: EnvLight) -> Color {
    if depth <= 0 {
        return Color::black();
    }

    let hit_info = scene.intersect(&mut ray);

    match hit_info {
        None => env_light(ray),
        Some((hit, mtl)) => {
            if !mtl.emission().is_black() {
                // Stops recursive path tracing if the material is emissive, and
                // returns the emitted radiance.
                mtl.emission()
            } else {
                let (scattered_ray, attenuation) = mtl.scatter(-ray.dir, &hit);
                let in_radiance =  path_integrator(scene, scattered_ray, depth - 1, env_light);
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
                println!("depth {}, hit = {}, scat_ray = {}", depth, hit, scattered_ray);
                let incident_radiance =  debug_pt(scene, scattered_ray, depth - 1, env_light);
                println!("attenuation = {:?}, Li = {:?}", attenuation, incident_radiance);
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
            (albedo + Color::from(hit.normal)) * 0.5
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
fn scene_125_spheres() -> (Box<BvhNode>, camera::Camera, EnvLight) {
    let mut camera = camera::Camera::new((WIDTH, HEIGHT), hcm::Degree(25.0).to_radian());
    camera.look_at(
        Point3::new(13.0, 2.0, 3.0),
        Point3::new(0.0, 0.0, 0.0),
        Vec3::ybase(),
    );

    let mut spheres = vec![
        shape::Sphere::new(Point3::new(0.0, -1000.0, 1.0), 1000.0),
        shape::Sphere::new(Point3::new(0.0, 1.0, 0.0), 1.0),
        shape::Sphere::new(Point3::new(-4.0, 1.0, 0.0), 1.0),
        shape::Sphere::new(Point3::new(4.0, 1.0, 0.0), 1.0),
    ];

    let mut mtls: Vec<Arc<dyn mtl::Material>> = vec![
        Arc::new(mtl::Lambertian::solid(Color::new(0.5, 0.5, 0.5))),
        Arc::new(mtl::Dielectric::new(1.5)),
        Arc::new(mtl::Lambertian::solid(Color::new(0.4, 0.2, 0.1))),
        Arc::new(mtl::Metal::new(Color::new(0.7, 0.6, 0.5), 0.0)),
    ];

    for a in -11..11 {
        for b in -11..11 {
            let choose_mtl = rand_f32();
            let center =
                Point3::new(a as f32, 0.2, b as f32) + 0.9 * Vec3::new(rand_f32(), 0.0, rand_f32());

            if (center - Point3::new(4.0, 0.2, 0.0)).norm() > 0.9 {
                spheres.push(shape::Sphere::new(center, 0.2));
                let mtl: Arc<dyn mtl::Material> = if choose_mtl < 0.8 {
                    let albedo = Color::new(rand_f32(), rand_f32(), rand_f32());
                    Arc::new(mtl::Lambertian::solid(albedo))
                } else if choose_mtl < 0.95 {
                    let albedo = Color::new(rand_f32(), rand_f32(), rand_f32());
                    let albedo = (albedo + Color::white()) * 0.5;
                    Arc::new(mtl::Metal::new(albedo, rand_f32() * 0.5))
                } else {
                    Arc::new(mtl::Dielectric::new(1.4))
                };
                mtls.push(mtl);
            }
        }
    }

    let spheres: Vec<_> = spheres.into_iter().map(|s| Arc::new(s)).collect();

    let instances: Vec<Instance> = mtls
        .into_iter()
        .zip(spheres.into_iter())
        .map(|(mtl, sphere)| Instance::new(sphere, mtl))
        .collect();

    let boxed_instances: Vec<Box<Instance>> =
        instances.iter().map(|x| Box::from(x.clone())).collect();
    (bvh::build_bvh(boxed_instances), camera, blue_sky)
}

#[allow(dead_code)]
fn scene_two_perlin_spheres() -> (Box<BvhNode>, camera::Camera, EnvLight) {
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
        Arc::new(shape::Sphere::new(Point3::new(0.0, -1000.0, 0.0), 1000.0)),
        Arc::new(shape::Sphere::new(Point3::new(0.0, 2.0, 0.0), 2.0)),
    ];

    let instances = shapes
        .into_iter()
        .map(|sphere| Box::new(Instance::new(sphere, mtl.clone())))
        .collect();
    let mut cam = Camera::new((WIDTH, HEIGHT), hcm::Degree(20.0).to_radian());
    cam.look_at(
        Point3::new(13.0, 2.0, -3.0),
        Point3::origin(),
        Vec3::ybase(),
    );

    (build_bvh(instances), cam, blue_sky)
}

#[allow(dead_code)]
fn scene_earth() -> (Box<BvhNode>, camera::Camera, EnvLight) {
    let earth_tex = Arc::new(tex::Image::from_file("assets/earthmap.png").unwrap());
    let earth_mtl = Arc::new(material::Lambertian::textured(earth_tex));

    let globe = Arc::new(shape::Sphere::new(Point3::origin(), 2.0));
    let instances = vec![Box::new(Instance::new(globe, earth_mtl))];

    let mut cam = Camera::new((WIDTH, HEIGHT), hcm::Degree(20.0).to_radian());
    cam.look_at(
        Point3::new(13.0, 2.0, -3.0),
        Point3::origin(),
        Vec3::ybase(),
    );

    (build_bvh(instances), cam, blue_sky)
}

#[allow(dead_code)]
fn scene_quad_light() -> Scene {
    let perlin_tex = Arc::new(tex::Perlin::with_freq(4.0));

    let mtl = Arc::new(mtl::Lambertian::textured(perlin_tex));
    let light = Arc::new(mtl::DiffuseLight::new(Color::gray(4.0)));

    let shapes = vec![
        Arc::new(shape::Sphere::new(Point3::new(0.0, -1000.0, 0.0), 1000.0)),
        Arc::new(shape::Sphere::new(Point3::new(0.0, 2.0, 0.0), 2.0)),
    ];

    let mut instances: Vec<_> = shapes
        .into_iter()
        .map(|sphere| Box::new(Instance::new(sphere, mtl.clone())))
        .collect();

    let xy_quad = shape::QuadXY::from_raw((3.0, 5.0), (1.0, 3.0), 2.1);
    instances.push(Box::new(Instance::new(Arc::new(xy_quad), light.clone())));
    instances.push(Box::new(Instance::new(
        Arc::new(shape::Sphere::new(Point3::new(0.0, 7.0, 0.0), 2.0)),
        light.clone(),
    )));

    let mut cam = Camera::new((WIDTH, HEIGHT), hcm::Degree(20.0).to_radian());
    cam.look_at(
        Point3::new(26.0, 3.0, -6.0),
        Point3::new(0.0, 2.0, 0.0),
        Vec3::ybase(),
    );

    (bvh::build_bvh(instances), cam, dark_room)
}

#[allow(dead_code)]
fn scene_quad() -> Scene {
    let xy_quad = shape::QuadXY::from_raw((-0.5, 0.5), (-0.3, 0.6), 2.5);
    let lam = Arc::new(mtl::Lambertian::solid(Color::new(0.2, 0.3, 0.7)));
    let instances = vec![Box::new(Instance::new(Arc::new(xy_quad), lam))];

    let cam = Camera::new((WIDTH, HEIGHT), hcm::Degree(45.0).to_radian());

    (bvh::build_bvh(instances), cam, blue_sky)
}

#[allow(dead_code)]
fn scene_cornell_box() -> Scene {
    use instance::identity;
    
    let red = mtl::Lambertian::solid(Color::new(0.65, 0.05, 0.05));
    let white = mtl::Lambertian::solid(Color::gray(0.73));
    let green = mtl::Lambertian::solid(Color::new(0.12, 0.45, 0.15));
    let light = mtl::DiffuseLight::new(Color::new(15.0, 15.0, 15.0));

    let red = Arc::new(red);
    let white = Arc::new(white);
    let green = Arc::new(green);
    let light = Arc::new(light);

    let shapes: Vec<Arc<dyn shape::Shape>> = vec![
        Arc::new(shape::QuadYZ::from_raw((0.0, 555.0), (0.0, 555.0), 555.0)), // green
        Arc::new(shape::QuadYZ::from_raw((0.0, 555.0), (0.0, 555.0), 0.0)),   // red
        Arc::new(shape::QuadXZ::from_raw(
            (213.0, 343.0),
            (227.0, 332.0),
            554.0,
        )),  // light
        Arc::new(shape::QuadXZ::from_raw((0.0, 555.0), (0.0, 555.0), 0.0)),   // white floor
        Arc::new(shape::QuadXZ::from_raw((0.0, 555.0), (0.0, 555.0), 555.0)), // white ceiling
        Arc::new(shape::QuadXY::from_raw((0.0, 555.0), (0.0, 555.0), 555.0)), // white back
        
        Arc::new(shape::Cuboid::from_points(Point3::origin(), Point3::new(165.0, 165.0, 165.0))),
        Arc::new(shape::Cuboid::from_points(Point3::origin(), Point3::new(165.0, 330.0, 165.0))),
        // Arc::new(shape::Sphere::new(Point3::new(250.0, 250.0, 250.0), 50.0))
    ];

    let mtl_seq: Vec<Arc<dyn mtl::Material>> =
        // vec![light];
        vec![red, green, light, white.clone(), white.clone(), white.clone(), 
             white.clone(), white.clone()];

    let mut instances: Vec<_> = shapes
        .into_iter()
        .zip(mtl_seq.into_iter())
        .map(|(shape, mtl)| Box::new(Instance::new(shape, mtl)))
        .collect();
    instances[6].transform = identity().rotate_y(hcm::Degree(15.0).to_radian()).translate(Vec3::new(265.0, 0.0, 105.0));
    instances[7].transform = identity().rotate_y(hcm::Degree(-18.0).to_radian()).translate(Vec3::new(130.0, 0.0, 225.0));
    
    println!("{:?}, {:?}", instances[6].transform, instances[7].transform);
    // std::process::exit(0);
        
    let mut cam = Camera::new((600, 600), hcm::Degree(40.0).to_radian());
    cam.look_at(Point3::new(278.0, 278.0, -800.0), Point3::new(278.0, 278.0, 0.0), Vec3::ybase());
    
    (build_bvh(instances), cam, dark_room)
}
