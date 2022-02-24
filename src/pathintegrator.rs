use geometry::ray;
use math::{float::Float, prob::Prob};
use radiometry::color::Color;
use rand::Rng;
use scene::Scene;

use crate::{bsdf::BSDF, directlighting};

pub fn path_integrator(scene: &Scene, mut ray: ray::Ray, depth: i32) -> Color {
    let mut rng = rand::thread_rng();
    let mut radiance = Color::black();
    let mut specular_bounce = false;
    let mut beta = Color::ONE;
    for bounces in 0..depth {
        // Intersects ray with scene.
        let hit = scene.tlas.intersect(&mut ray);

        // Possibly add emitted light at intersection.
        if bounces == 0 || specular_bounce {
            let mtl_emission = hit.map(|(_, mtl)| mtl.emission());
            radiance += beta * mtl_emission.unwrap_or(scene.eval_env_light(ray));
        }

        // Terminates path if ray escaped or max_depth was reached.
        if hit.is_none() {
            break;
        }

        // Computes BSDF and skip over medium boundaries.
        let (hit, mtl) = hit.unwrap();
        let bxdfs = mtl.bxdfs_at(&hit);
        // assert!(!bxdfs.is_empty(), "mtl = {}", mtl.summary());

        // Samples illumination from lights to find path contribution.
        radiance += beta * directlighting::uniform_sample_one_light(&hit, &**mtl, scene, false);

        // Sample BSDF to get new path direction.
        let shading_point = BSDF::new_frame(&hit).with_bxdfs(&bxdfs);
        if false && shading_point.world_to_local(hit.wo).0.z.dist_to(1.0) < 1e-4 {
            eprintln!(
                "ray = {:.5}, hit = {}, normal = {:.5}",
                ray, hit, hit.normal
            );
        }
        // TODO: replace rnd2 with values from sampler.
        let rnd2 = rng.gen::<(f32, f32)>();
        let (f, wi, pr) = shading_point.sample(-ray.dir, rnd2);
        if f.is_black() || pr.is_zero() {
            break;
        }
        if (wi.x + wi.z + wi.y).abs().dist_to(1.0) < 1e-4 {
            // eprintln!("wi = {:.5}, uv = {:?}, pr = {:?}, f = {f}", wi, rnd2, pr);
        }

        specular_bounce = matches!(pr, Prob::Mass(_));
        let pr = match pr {
            Prob::Mass(pmf) => pmf,
            Prob::Density(pdf) => pdf,
        };

        beta = beta * f * wi.dot(hit.normal) * pr.recip();
        ray = hit.spawn_ray(wi);

        // Terminates the path probabilistically with Russian roulette.
        if bounces > 3 {
            let q = (1.0 - beta.luminance()).max(0.05);
            if rng.gen::<f32>() < q {
                break;
            }
            beta = beta * (1.0 - q).recip();
        }
    }
    radiance
}
