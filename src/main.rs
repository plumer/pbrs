mod bvh;
mod camera;
mod hcm;
mod image;
mod instance;
mod material;
mod ray;
mod shape;

use rayon::prelude::*;
use std::fs::File;
use std::io::{self, BufWriter};
use std::path::Path;
use std::sync::Arc;
use std::time::{Instant};

use hcm::{Point3, Vec3};
use image::Color;
use instance::Instance;
use io::Write;
use material::{Dielectric, Lambertian, Metal};

const WIDTH: u32 = 1200;
const HEIGHT: u32 = 800;
const MSAA: usize = 12;
const SAMPLES_PER_PIXEL: usize = MSAA * MSAA;

fn radiance_in(scene: &[Instance], mut ray: ray::Ray, depth: i32) -> Color {
    let bg_color_top = Color::new(0.5, 0.7, 1.0);
    let bg_color_bottom = Color::white();

    if depth <= 0 {
        return Color::black();
    }

    let mut closest_hit = None;
    let mut mtl = &scene[0].mtl;
    for inst in scene.iter() {
        let isect = inst.shape.intersect(&ray);
        match isect {
            None => (),
            Some(hit) => {
                ray.set_extent(hit.ray_t);
                closest_hit = isect;
                mtl = &inst.mtl;
            }
        }
    }

    match closest_hit {
        None => {
            let y = (ray.dir.hat().y + 1.0) * 0.5;
            bg_color_top * y + bg_color_bottom * (1.0 - y)
        }
        Some(hit) => {
            // let diffuse = hit.normal.dot(-directional_light).max(0.1);
            // sphere_color * diffuse
            // hit.normal + material::uniform_hemisphere();

            let (scattered_ray, attenuation) = mtl.scatter(-ray.dir, &hit);
            let color = radiance_in(scene, scattered_ray, depth - 1);
            color * attenuation
        }
    }
}

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
    println!("Hello, world!");
    let half_right_angle = hcm::Degree(45.0);
    println!("{} is {} ", half_right_angle, half_right_angle.to_radian());

    // Builds the camera.
    let mut camera = camera::Camera::new((WIDTH, HEIGHT), hcm::Degree(25.0).to_radian());
    camera.look_at(
        Point3::new(13.0, 2.0, 3.0),
        Point3::new(0.0, 0.0, 0.0),
        Vec3::ybase(),
    );

    let mut spheres = vec![
        shape::Sphere::new(Point3::new(0.0, -1000.5, 1.0), 1000.0),
        shape::Sphere::new(Point3::new(0.0, 1.0, 0.0), 1.0),
        shape::Sphere::new(Point3::new(-4.0, 1.0, 0.0), 1.0),
        shape::Sphere::new(Point3::new(4.0, 1.0, 0.0), 1.0),
    ];

    let mut mtls: Vec<Arc<dyn material::Material>> = vec![
        Arc::new(Lambertian::new(Color::new(0.5, 0.5, 0.5))),
        Arc::new(Dielectric::new(1.5)),
        Arc::new(Lambertian::new(Color::new(0.4, 0.2, 0.1))),
        Arc::new(Metal::new(Color::new(0.7, 0.6, 0.5), 0.0)),
    ];

    for a in -11..11 {
        for b in -11..11 {
            let choose_mtl = rand_f32();
            let center =
                Point3::new(a as f32, 0.2, b as f32) + 0.9 * Vec3::new(rand_f32(), 0.0, rand_f32());

            if (center - Point3::new(4.0, 0.2, 0.0)).norm() > 0.9 {
                spheres.push(shape::Sphere::new(center, 0.2));
                let mtl: Arc<dyn material::Material> = if choose_mtl < 0.8 {
                    let albedo = Color::new(rand_f32(), rand_f32(), rand_f32());
                    Arc::new(Lambertian::new(albedo))
                } else if choose_mtl < 0.95 {
                    let albedo = Color::new(rand_f32(), rand_f32(), rand_f32());
                    let albedo = (albedo + Color::white()) * 0.5;
                    Arc::new(Metal::new(albedo, rand_f32() * 0.5))
                } else {
                    Arc::new(Dielectric::new(1.4))
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

    let start_render = Instant::now();

    let color_data: Vec<_> = (0..HEIGHT)
        .into_par_iter()
        .map(|row| {
            if (row % 10) == 0 {
                print!("{} ", row);
                io::stdout().flush().unwrap();
            }
            let mut colors_for_row = vec![];
            for col in 0..WIDTH {
                let mut color_sum = Color::black();

                for i in 0..MSAA * MSAA {
                    let jitter = (
                        ((i / MSAA) as f32 + rand::random::<f32>()) / MSAA as f32,
                        ((i % MSAA) as f32 + rand::random::<f32>()) / MSAA as f32,
                    );
                    let jitter = (rand::random::<f32>(), rand::random::<f32>());
                    let ray = camera.shoot_ray(row, col, jitter).unwrap();
                    color_sum = color_sum + radiance_in(&instances, ray, 50);
                }

                let color = color_sum / (SAMPLES_PER_PIXEL as f32);
                colors_for_row.push(color);
            }
            colors_for_row
        })
        .collect();

    // let mut image_map = vec![];
    // for mut row in color_data.into_iter() {
    //     image_map.append(&mut row);
    // }

    let image_map = color_data
        .into_iter()
        .fold(vec![], |mut partial_image, mut row| {
            partial_image.append(&mut row);
            partial_image
        });
    let image_data = image_map
        .iter()
        .fold(Vec::<u8>::new(), |mut partial_data, color| {
            partial_data.append(&mut color.gamma_encode().to_u8().to_vec());
            partial_data
        });

    let whole_render_time = Instant::now().duration_since(start_render);

    println!("whole render time = {} us", whole_render_time.as_micros());

    write_image(r"output.png", &image_data, (WIDTH, HEIGHT));

    // let mut data: Vec<u8> = Vec::new();
    // data.resize(256 * 256 * 4 as usize, 230);
    // for (dx, dy) in jitters.iter().take(10000) {
    //     let ix = (dx * 192.0) as usize + 32;
    //     let iy = (dy * 192.0) as usize + 32;

    //     let base = (iy * 256 + ix) * 4;
    //     data[base + 0] = 30;
    //     data[base + 1] = 30;
    //     data[base + 2] = 30;
    //     data[base + 3] = 255;
    // }

    // write_image(r"samples.png", &data, (256, 256));
}
