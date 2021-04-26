mod camera;
mod hcm;
mod ray;
mod shape;
mod image;
mod material;

use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use hcm::Point3;
use image::Color;
use shape::{Interaction, Shape};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const NUM_CHANNELS: u32 = 4;
const NUM_BYTES: usize = (WIDTH * HEIGHT * NUM_CHANNELS) as usize;

fn intersect(spheres: &[shape::Sphere], mut ray: ray::Ray) -> Option<Interaction> {
    let mut closest_hit = None;
    for s in spheres.iter() {
        let isect = s.intersect(&ray);
        match isect {
            None => (),
            Some(hit) => {
                ray.set_extent(hit.ray_t);
                closest_hit = isect;
            }
        }
    }
    closest_hit
}

// RGB(165, 200, 240)
// RGB(206, 140, 126)
fn main() {
    println!("Hello, world!");
    let half_right_angle = hcm::Degree(45.0);
    println!("{} is {} ", half_right_angle, half_right_angle.to_radian());

    // Builds the camera.
    let camera = camera::Camera::new((WIDTH, HEIGHT), hcm::Degree(90.0).to_radian());
    let sphere = shape::Sphere::new(Point3::new(0.0, 0.0, 2.0), 1.0);
    let sphere_color = Color::rgb(25, 99, 150);
    let bg_color_top = Color::rgb(165, 200, 240);
    let bg_color_bottom = Color::rgb(206, 140, 126);
    let directional_light = hcm::Vec3::new(-0.5, -0.5, 0.5).hat();
    
    let spheres = [sphere];

    let mut data: Vec<u8> = Vec::new();
    data.resize(NUM_BYTES, 0);
    for row in 0..HEIGHT {
        for col in 0..WIDTH {

            if (row, col) == (HEIGHT/2, WIDTH/2) {
                println!("center");
            }            
            let ray = camera.shoot_ray(row, col).unwrap();
            let isect = intersect(&spheres, ray);
            let color = match isect {
                None => {
                    let y = row as f32 / HEIGHT as f32;
                    bg_color_top * (1.0-y) + bg_color_bottom * y
                }
                Some(hit) => {
                    let diffuse = hit.normal.dot(-directional_light).max(0.1);
                    sphere_color * diffuse
                }
            }.to_u8();

            
            let base = (row * WIDTH + col) as usize * 4;
            data[base + 0] = color[0];
            data[base + 1] = color[1];
            data[base + 2] = color[2];
            data[base + 3] = 255;
        }
        // println!("");
    }

    let path = Path::new(r"output.png");
    let file = File::create(path).unwrap();
    let ref mut w = BufWriter::new(file);

    let mut encoder = png::Encoder::new(w, WIDTH, HEIGHT);
    encoder.set_color(png::ColorType::RGBA);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().unwrap();

    writer.write_image_data(&data).unwrap();
}
