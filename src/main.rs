mod camera;
mod hcm;
mod ray;
mod shape;

use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use hcm::Point3;
use shape::Shape;

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const NUM_CHANNELS: u32 = 4;
const NUM_BYTES: usize = (WIDTH * HEIGHT * NUM_CHANNELS) as usize;

fn main() {
    println!("Hello, world!");
    let half_right_angle = hcm::Degree(45.0);
    println!("{} is {} ", half_right_angle, half_right_angle.to_radian());

    // Builds the camera.
    let camera = camera::Camera::new((WIDTH, HEIGHT), hcm::Degree(90.0).to_radian());
    let sphere = shape::Sphere::new(Point3::new(0.0, 0.0, 2.0), 1.0);

    let mut data: Vec<u8> = Vec::new();
    data.resize(NUM_BYTES, 0);
    for row in 0..HEIGHT {
        for col in 0..WIDTH {

            if (row, col) == (HEIGHT/2, WIDTH/2) {
                println!("center");
            }            
            let ray = camera.shoot_ray(row, col).unwrap();
            let isect = sphere.intersect(&ray);
            let color = match isect {
                None => [34u8, 34, 34],
                Some(_) => [25u8, 99, 150],
            };

            
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
