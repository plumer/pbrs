use geometry::camera::Camera;
use geometry::ray;
use light::SamplableShape;
use math::hcm::{point3, vec3, Point3, Vec3};
use radiometry::color::Color;

use crate::Scene;
use material as mtl;
use shape::Sphere;
use std::f32::consts::PI;
use std::sync::Arc;
use texture as tex;
use tlas::instance::Instance;

const WIDTH: u32 = 800;
const HEIGHT: u32 = 800;

fn rand_f32() -> f32 {
    rand::random::<f32>()
}

// Functions that build the scenes: environment light and collection of objects.
// ------------------------------------------------------------------------------------------------

pub fn blue_sky(r: ray::Ray) -> Color {
    let bg_color_top = Color::new(0.5, 0.7, 1.0);
    let bg_color_bottom = Color::white();
    let y = (r.dir.hat().y + 1.0) * 0.5;
    bg_color_top * y + bg_color_bottom * (1.0 - y)
}

pub fn dark_room(r: ray::Ray) -> Color {
    let bg_color_top = Color::gray(0.1);
    let bg_color_bottom = Color::gray(0.1);
    let y = (r.dir.hat().y + 1.0) * 0.5;
    bg_color_top * y + bg_color_bottom * (1.0 - y)
}

pub fn dusk(r: ray::Ray) -> Color {
    let horizon = Color::rgb(245, 174, 82);
    let dome = Color::rgb(109, 150, 204);
    let tilt = r.dir.hat().y.acos();
    if tilt > PI * 0.25 {
        dome
    } else if tilt > 0.0 {
        let t = tilt / (PI * 0.25);
        dome * t + horizon * (1.0 - t)
    } else {
        Color::gray(0.2)
    }
}

#[allow(dead_code)]
#[allow(unused_mut)]
pub fn mixed_spheres() -> Scene {
    let mut camera = Camera::new((WIDTH, HEIGHT), math::new_deg(25.0));
    camera.look_at(point3(13.0, 2.0, 3.0), point3(0.0, 0.0, 0.0), Vec3::Y);

    let mut spheres = vec![
        Sphere::from_raw((0.0, -1000.0, 1.0), 1000.0),
        Sphere::from_raw((0.0, 1.0, 0.0), 1.0),
        Sphere::from_raw((-4.0, 1.0, 0.0), 1.0),
        Sphere::from_raw((4.0, 1.0, 0.0), 1.0),
    ];
    let (real, imag) = gold_fresnel();
    let mut mtls: Vec<Arc<dyn mtl::Material>> = vec![
        Arc::new(mtl::Lambertian::solid(Color::new(0.5, 0.5, 0.5))),
        Arc::new(mtl::Dielectric::new(1.5)),
        Arc::new(mtl::Lambertian::solid(Color::new(0.4, 0.2, 0.1))),
        Arc::new(mtl::Metal::from_ior(real, imag, 0.0)),
    ];

    let metal_iors = [
        gold_fresnel(),
        silver_fresnel(),
        copper_fresnel(),
        aluminium_fresnel(),
    ];

    for a in -11..11 {
        for b in -11..11 {
            let choose_mtl = rand_f32();
            let center = point3(a as f32, 0.2 + rand_f32().powi(3) * 0.1, b as f32)
                + 0.9 * Vec3::new(rand_f32(), 0.0, rand_f32());

            if (center - point3(4.0, 0.2, 0.0)).norm() > 0.9 {
                spheres.push(Sphere::new(center, 0.2));
                let mtl: Arc<dyn mtl::Material> = if choose_mtl < 0.8 {
                    let albedo = Color::new(rand_f32(), rand_f32(), rand_f32());
                    Arc::new(mtl::Lambertian::solid(albedo))
                } else if choose_mtl < 0.95 {
                    let (real, imag) = metal_iors[(rand::random::<u8>() % 4) as usize];
                    Arc::new(mtl::Metal::from_ior(real, imag, rand_f32() * 0.5))
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
    Scene::new(*tlas::build_bvh(boxed_instances), camera).with_fn_env_light(blue_sky)
}

pub fn two_perlin_spheres() -> Scene {
    let perlin_tex = tex::Perlin::with_freq(4.0);

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
    cam.look_at(point3(13.0, 2.0, -3.0), Point3::ORIGIN, Vec3::Y);

    Scene::new(*tlas::build_bvh(instances), cam).with_fn_env_light(blue_sky)
}

pub fn earth() -> Scene {
    let earth_tex = Arc::new(tex::Image::from_file("assets/earthmap.png").unwrap());
    let earth_mtl = Arc::new(material::Lambertian::textured(earth_tex));

    let globe = Arc::new(Sphere::new(Point3::ORIGIN, 2.0));
    let instances = vec![Box::new(Instance::new(globe, earth_mtl))];

    let mut cam = Camera::new((WIDTH, HEIGHT), math::new_deg(20.0));
    cam.look_at(point3(13.0, 2.0, -3.0), Point3::ORIGIN, Vec3::Y);

    Scene::new(*tlas::build_bvh(instances), cam).with_fn_env_light(blue_sky)
}

pub fn quad_light() -> Scene {
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

    let light_quad = shape::ParallelQuad::new_xy((3.0, 5.0), (1.0, 3.0), 2.1);
    let light_sphere = Sphere::from_raw((0.0, 7.0, 0.0), 2.0);
    instances.extend(vec![
        Box::new(Instance::new(Arc::new(light_quad), light.clone())),
        Box::new(Instance::new(Arc::new(light_sphere), light.clone())),
    ]);
    let area_lights = vec![
        light::DiffuseAreaLight::new(light_power, SamplableShape::from(light_quad)),
        light::DiffuseAreaLight::new(light_power, SamplableShape::from(light_sphere)),
    ];

    let mut cam = Camera::new((WIDTH, HEIGHT), math::new_deg(20.0));
    cam.look_at(point3(26.0, 3.0, -6.0), point3(0.0, 2.0, 0.0), Vec3::Y);

    Scene::new(*tlas::build_bvh(instances), cam)
        .with_fn_env_light(dark_room)
        .with_lights(vec![], area_lights)
}

pub fn quad() -> Scene {
    let xy_quad = shape::ParallelQuad::new_xy((-0.5, 0.5), (-0.3, 0.6), 2.5);
    let lam = Arc::new(mtl::Lambertian::solid(Color::new(0.2, 0.3, 0.7)));
    let instances = vec![Box::new(Instance::new(Arc::new(xy_quad), lam))];

    let cam = Camera::new((WIDTH, HEIGHT), math::new_deg(45.0));

    Scene::new(*tlas::build_bvh(instances), cam).with_fn_env_light(blue_sky)
}

pub fn cornell_box() -> Scene {
    use shape::ParallelQuad as Quad;
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
        SamplableShape::Quad(Quad::new_xz((213.0, 343.0), 554.0, (227.0, 332.0))),
    )];

    let shapes: Vec<Arc<dyn shape::Shape>> = vec![
        Arc::new(Quad::new_yz(555.0, (0.0, 555.0), (0.0, 555.0))), // green
        Arc::new(Quad::new_yz(0.0, (0.0, 555.0), (0.0, 555.0))),   // red
        Arc::new(Quad::new_xz((213.0, 343.0), 554.0, (227.0, 332.0))), // light
        Arc::new(Quad::new_xz((0.0, 555.0), 0.0, (0.0, 555.0))),   // white floor
        Arc::new(Quad::new_xz((0.0, 555.0), 555.0, (0.0, 555.0))), // white ceiling
        Arc::new(Quad::new_xy((0.0, 555.0), (0.0, 555.0), 555.0)), // white back
        Arc::new(shape::Cuboid::from_points(
            Point3::ORIGIN,
            point3(165.0, 165.0, 165.0),
        )),
        Arc::new(shape::Cuboid::from_points(
            Point3::ORIGIN,
            point3(165.0, 330.0, 165.0),
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
    instances[6].transform = geometry::transform::identity()
        .rotate_y(math::new_deg(15.0))
        .translate(Vec3::new(265.0, 0.0, 105.0));
    instances[7].transform = geometry::transform::identity()
        .rotate_y(math::new_deg(-18.0))
        .translate(Vec3::new(130.0, 0.0, 225.0));

    println!("{:?}, {:?}", instances[6].transform, instances[7].transform);
    // std::process::exit(0);

    let mut cam = Camera::new((600, 600), math::new_deg(40.0));
    cam.look_at(
        point3(278.0, 278.0, -800.0),
        point3(278.0, 278.0, 0.0),
        Vec3::Y,
    );

    Scene::new(*tlas::build_bvh(instances), cam).with_lights(vec![], area_lights)
}

pub fn plates() -> Scene {
    let mut instances = vec![];
    let r = 20.0;
    // Builds the background.
    let wall = shape::ParallelQuad::new_xy((-r, r), (0.0, r), 0.0);
    let floor = shape::ParallelQuad::new_xz((-r, r), 0.0, (-r, 0.0));
    let matte = mtl::Lambertian::solid(Color::gray(0.4));

    let wall_instance = Instance::from_raw(wall, matte.clone());
    let floor_instance = Instance::from_raw(floor, matte.clone());
    instances.push(wall_instance);
    instances.push(floor_instance);

    // axis: y = 10, z = 0
    let lights_pos = point3(0.0, r, -0.4 * r);
    let camera_pos = point3(0.0, 0.4 * r, -2.8 * r);
    let (left, right) = (-r * 0.7, r * 0.7);
    {
        let plates_pos_yz = vec![
            (0.6 * r, -0.2 * r),
            (0.45 * r, -0.3 * r),
            (0.3 * r, -0.45 * r),
            (0.2 * r, -0.6 * r),
        ];
        let plate_width = 0.16 * r;
        let plates = plates_pos_yz
            .into_iter()
            .map(|(py, pz)| {
                let pl = vec3(0.0f32, lights_pos.y - py, lights_pos.z - pz);
                let pc = vec3(0.0f32, camera_pos.y - py, camera_pos.z - pz);
                let normal = (pl.hat() + pc.hat()).hat();
                let tangent = vec3(0.0, normal.z, -normal.y).hat() * (plate_width * 0.5);
                let t00 = point3(left, py, pz) + tangent;
                let t01 = t00 - tangent * 2.0;
                let t10 = point3(right, py, pz) + tangent;
                let t11 = t10 - tangent * 2.0;
                assert!(tangent.dot(normal).abs() < 1e-5);
                shape::TriangleMesh::from_soa(
                    vec![t00, t01, t10, t11],
                    vec![normal, normal, normal, normal],
                    vec![(0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (1.0, 1.0)],
                    vec![(0, 1, 2), (2, 1, 3)],
                )
            })
            .collect::<Vec<_>>();
        let materials = [8e-5, 3e-4, 8e-4, 3e-3]
            .iter()
            .map(|rough| mtl::Glossy::new(Color::gray(0.9), *rough))
            .collect::<Vec<_>>();

        instances.extend(
            plates
                .into_iter()
                .zip(materials.into_iter())
                .map(|(plate, mtl)| Instance::from_raw(plate, mtl)),
        );
    }

    let mut area_lights = vec![];
    {
        // Adds lights to the scenes.
        let num_lights = 4;
        let (light_xpos, _spacing) = math::float::linspace((left * 0.9, right * 0.9), num_lights);
        let light_sizes = [0.1 * r, 0.06 * r, 0.03 * r, 0.01 * r];
        let light_colors = [
            Color::new(1.0, 0.8, 0.8),
            Color::new(1.0, 1.0, 0.8),
            Color::new(0.8, 1.0, 0.8),
            Color::new(0.8, 0.8, 1.0),
        ];
        let light_spheres = light_xpos
            .iter()
            .enumerate()
            .map(|(i, x)| Sphere::new(lights_pos.with_x(*x), light_sizes[i]));

        area_lights.extend(
            light_spheres
                .clone()
                .zip(light_colors.iter())
                .map(|(s, c)| light::DiffuseAreaLight::new(*c, SamplableShape::from(s))),
        );
        instances.extend(
            light_spheres
                .zip(light_colors.iter())
                .map(|(sphere, color)| Instance::from_raw(sphere, mtl::DiffuseLight::new(*color))),
        );
        assert_eq!(area_lights.len(), num_lights as usize);
    }
    // Wraps up the shape instances.
    let instances: Vec<_> = instances.into_iter().map(|i| Box::new(i)).collect();

    let camera = Camera::new((1000, 800), math::Angle::pi() * 0.19)
        .looking_at(camera_pos, camera_pos + Vec3::Z, Vec3::Y)
        // .translate(vec3(-0.4 * r, 0.0, 0.0))
        ;

    Scene::new(*tlas::build_bvh(instances), camera)
        // .with_env_light(|r| blue_sky(r) * 0.4)
        .with_lights(vec![], area_lights)
}

pub fn everything() -> Scene {
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
                    point3(x0, 0.0, z0),
                    point3(x1, y1, z1),
                )),
                ground.clone(),
            ));
        }
    }

    let light = mtl::DiffuseLight::new(Color::gray(7.0));
    let light_quad = shape::ParallelQuad::new_xz((123.0, 423.0), 554.0, (147.0, 412.0));
    let area_lights = vec![light::DiffuseAreaLight::new(
        Color::gray(7.0),
        SamplableShape::Quad(light_quad),
    )];

    instances.push(Instance::from_raw(light_quad, light));

    let glass_ball = Sphere::from_raw((260.0, 150.0, 45.0), 50.0);
    let glass_mtl = mtl::Dielectric::new(1.5);
    instances.push(Instance::from_raw(glass_ball, glass_mtl));

    let metal_ball = Sphere::from_raw((0.0, 150.0, 145.0), 50.0);
    let (real, imag) = silver_fresnel();
    let metal_mtl = mtl::Metal::from_ior(real, imag, 1.0);
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
    let pp_trans = geometry::transform::identity()
        .rotate_y(math::new_deg(15.0))
        .translate(Vec3::new(-100.0, 270.0, 395.0));
    instances.push(Instance::from_raw(ping_pong_balls, matte_white).with_transform(pp_trans));

    let mut cam = Camera::new((800, 800), math::new_deg(40.0));
    cam.look_at(
        point3(478.0, 278.0, -600.0),
        point3(278.0, 278.0, 0.0),
        Vec3::Y,
    );

    let instances: Vec<_> = instances.into_iter().map(|i| Box::new(i)).collect();

    Scene::new(*tlas::build_bvh(instances), cam)
        .with_fn_env_light(dark_room)
        .with_lights(vec![], area_lights)
}

pub fn env_mapped() -> Scene {
    let sphere = Sphere::new(Point3::ORIGIN, 2.0);
    let sphere_mtl = mtl::Mirror::new(Color::white());
    let mut instances = vec![Box::new(Instance::from_raw(sphere, sphere_mtl))];
    // TODO: add more spheres with different materials
    for (i, roughness) in [0.001f32, 0.003, 0.01, 0.03].iter().enumerate() {
        let (real, imag) = gold_fresnel();
        let metal_mtl = mtl::Metal::from_ior(real, imag, *roughness);
        // let glossy_mtl = mtl::Glossy::new(Color::white(), *roughness);
        let sphere = Sphere::new(point3(i as f32 * 6.0 - 9.0, 6.0, 0.0), 2.0);
        instances.push(Box::new(Instance::from_raw(sphere, metal_mtl)));
    }
    let camera = Camera::new((1280, 800), math::new_deg(60.0)).looking_at(
        point3(0.0, 0.0, -24.0),
        Point3::ORIGIN,
        Vec3::Y,
    );

    let earth_map = tex::Image::from_file("assets/venice_dawn_1_2k.png").unwrap();

    Scene::new(*tlas::build_bvh(instances), camera).with_env_map(earth_map, Color::ONE)
}

pub fn silver_fresnel() -> (Color, Color) {
    (
        Color::new(0.155184, 0.116681, 0.138360),
        Color::new(4.828131, 3.122411, 2.147082),
    )
}

pub fn aluminium_fresnel() -> (Color, Color) {
    (
        Color::new(1.656937, 0.880173, 0.521201),
        Color::new(9.224230, 6.269670, 4.836996),
    )
}

pub fn gold_fresnel() -> (Color, Color) {
    (
        Color::new(0.143176, 0.373096, 1.443834),
        Color::new(3.982675, 2.387439, 1.602465),
    )
}

pub fn copper_fresnel() -> (Color, Color) {
    (
        Color::new(0.195470, 0.925682, 1.102186),
        Color::new(3.910869, 2.451263, 2.142653),
    )
}
