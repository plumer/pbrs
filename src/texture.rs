use crate::{hcm::*, image::Color};
use std::fs::File;

pub trait Texture: Send + Sync {
    fn value(&self, uv: (f32, f32), p: Point3) -> Color;
}

pub struct Solid {
    value: Color,
}

impl Solid {
    pub fn new(value: Color) -> Solid {
        Solid { value }
    }
}

impl Texture for Solid {
    fn value(&self, _: (f32, f32), _: Point3) -> Color {
        self.value
    }
}

pub struct Checker {
    odd: Color,
    even: Color,
}

impl Texture for Checker {
    fn value(&self, _: (f32, f32), p: Point3) -> Color {
        let sines = (10.0 * p.x).sin() * (10.0 * p.y).sin() * (10.0 * p.z).sin();
        if sines < 0.0 {
            self.odd
        } else {
            self.even
        }
    }
}

const PERLIN_NUM_POINTS: usize = 256;
pub struct Perlin {
    // rand_f: [f32; PERLIN_NUM_POINTS],

    // Ken Perlin's very clever trick was to instead put random unit vectors
    // (instead of just `f32`s on the lattice points), and use a dot-product
    // to move the min and max off the lattice.
    rand_vec: [Vec3; PERLIN_NUM_POINTS],
    perm_x: [u32; PERLIN_NUM_POINTS],
    perm_y: [u32; PERLIN_NUM_POINTS],
    perm_z: [u32; PERLIN_NUM_POINTS],
    freq: f32,
}

impl Perlin {
    pub fn new() -> Perlin {
        let mut rand_vec = [Vec3::xbase(); PERLIN_NUM_POINTS];
        for v in rand_vec.iter_mut() {
            *v = crate::material::uniform_sphere();
        }
        Perlin {
            rand_vec,
            perm_x: Self::make_random_permutation(),
            perm_y: Self::make_random_permutation(),
            perm_z: Self::make_random_permutation(),
            freq: 1.0,
        }
    }

    pub fn with_freq(freq: f32) -> Perlin {
        let mut res = Self::new();
        res.freq = freq;
        res
    }

    fn make_random_permutation() -> [u32; PERLIN_NUM_POINTS] {
        let mut perm = [0u32; PERLIN_NUM_POINTS];
        for i in 0..PERLIN_NUM_POINTS {
            perm[i] = i as u32;
        }
        for i in 0..PERLIN_NUM_POINTS {
            let target = rand::random::<usize>() % PERLIN_NUM_POINTS;
            perm.swap(target, i);
        }
        perm
    }

    fn noise(&self, p: Point3) -> f32 {
        let split_f32 = |f: f32| (f.floor() as i32, f - f.floor());
        let (i, u) = split_f32(p.x * self.freq);
        let (j, v) = split_f32(p.y * self.freq);
        let (k, w) = split_f32(p.z * self.freq);

        let u = u * u * (3.0 - 2.0 * u);
        let v = v * v * (3.0 - 2.0 * v);
        let w = w * w * (3.0 - 2.0 * w);

        let mut c = [[[Vec3::zero(); 2]; 2]; 2];
        for di in 0..2 {
            for dj in 0..2 {
                for dk in 0..2 {
                    let i = ((i + di) & 255) as usize;
                    let j = ((j + dj) & 255) as usize;
                    let k = ((k + dk) & 255) as usize;
                    let index = self.perm_x[i] ^ self.perm_y[j] ^ self.perm_z[k];
                    c[di as usize][dj as usize][dk as usize] = self.rand_vec[index as usize];
                }
            }
        }

        let mut accum = 0.0;
        for di in 0..2 {
            for dj in 0..2 {
                for dk in 0..2 {
                    let weight_v = Vec3::new(u - di as f32, v - dj as f32, w - dk as f32);
                    let dot_product = c[di][dj][dk].dot(weight_v);
                    accum += (di as f32 * u + (1 - di) as f32 * (1.0 - u))
                        * (dj as f32 * v + (1 - dj) as f32 * (1.0 - v))
                        * (dk as f32 * w + (1 - dk) as f32 * (1.0 - w))
                        * dot_product;
                }
            }
        }
        assert!(accum >= -1.0);
        assert!(accum <= 1.0);

        accum
    }

    fn turbulance(&self, p: Point3) -> f32 {
        let scale_point =
            |p: Point3, scale: f32| Point3::new(p.x * scale, p.y * scale, p.z * scale);
        (0..7)
            .map(|i| 0.5f32.powi(i) * self.noise(scale_point(p, 2.0f32.powi(i))))
            .fold(0.0, |a, b| a + b)
            .abs()
    }
}

impl Texture for Perlin {
    fn value(&self, _: (f32, f32), p: Point3) -> Color {
        // self.turbulance(p) * Color::white()

        // A marble-like texture.
        (self.freq * p.z + 10.0 * self.turbulance(p))
            .sin()
            .mul_add(0.5, 0.5)
            * Color::white()
    }
}

pub struct Image {
    data: Vec<Color>,
    width: u32,
    height: u32,
}

impl Image {
    pub fn from_file(path: &str) -> Result<Image, &str> {
        let decoder = png::Decoder::new(File::open(path).unwrap());
        let (info, mut reader) = decoder.read_info().unwrap();
        // Allocate the output buffer.
        let mut buf = vec![0; info.buffer_size()];
        // Read the next frame. An APNG might contain multiple frames.
        reader.next_frame(&mut buf).unwrap();

        if info.bit_depth != png::BitDepth::Eight {
            return Err("non 8-bit image");
        }

        let num_channels = match info.color_type {
            png::ColorType::Grayscale => 1,
            png::ColorType::RGB => 3,
            png::ColorType::RGBA => 4,
            png::ColorType::Indexed => return Err("Unhandled ColorType::Indexed"),
            png::ColorType::GrayscaleAlpha => return Err("Unhandled ColorType::GrayscaleAlpha"),
        };

        let num_pixels = info.width * info.height;
        assert_eq!(num_pixels * num_channels, buf.len() as u32);

        let color_data: Vec<Color> = match num_channels {
            1 => buf
                .iter()
                .map(|gray_u8| Color::gray(*gray_u8 as f32 / 255.0))
                .collect(),
            3 | 4 => buf
                .chunks(num_channels as usize)
                .map(|rgba| Color::rgb(rgba[0], rgba[1], rgba[2]))
                .collect(),
            _ => panic!("num channels ({}) should be 1, 3, or 4", num_channels),
        };
        Ok(Image {
            data: color_data,
            width: info.width,
            height: info.height,
        })
    }
}

impl Texture for Image {
    fn value(&self, uv: (f32, f32), _: Point3) -> Color {
        let (u, v) = uv;
        let u = u.clamp(0.0, 1.0);
        let v = v.clamp(0.0, 1.0);

        let col = (u * self.width as f32) as usize % self.width as usize;
        let row = (v * self.height as f32) as usize % self.height as usize;

        let index = row * self.width as usize + col;
        self.data[index]
    }
}
