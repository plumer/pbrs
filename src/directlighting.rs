use std::ops::Deref;

use geometry::ray;
use math::{float::Float, prob::Prob};
use radiometry::color::Color;
use rand::Rng;
use shape::Interaction;

use crate::bsdf::BSDF;
use light::{DeltaLight, DiffuseAreaLight, Light};
use material::Material;
use scene::Scene;

pub fn direct_lighting_integrator(scene: &Scene, mut ray: ray::Ray, _depth: i32) -> Color {
    if let Some((hit, mtl)) = scene.tlas.intersect(&mut ray) {
        // Computes scattering functions for surface interaction.
        // ------------------------------------------------------
        // bsdf <- isect.compute_bsdf(mtl)
        // if bsdf.is_none()
        //  return <recursive_call>(scene, isect.spawn_ray(ray.d))

        // Computes eitted light if ray hits an area light source.
        // radiance += mtl.emission()
        if !mtl.emission().is_black() {
            mtl.emission()
        } else {
            uniform_sample_one_light(&hit, mtl.deref(), scene, false)
        }
    } else {
        scene.eval_env_light(ray)
    }
}

#[allow(dead_code)]
pub fn direct_lighting_debug_integrator(scene: &Scene, mut ray: ray::Ray, _depth: i32) -> Color {
    if let Some((hit, mtl)) = scene.tlas.intersect(&mut ray) {
        uniform_sample_one_light(&hit, mtl.deref(), scene, true)
    } else {
        scene.eval_env_light(ray)
    }
}

fn uniform_sample_one_light(
    hit: &Interaction, mtl: &dyn Material, scene: &Scene, _debug: bool,
) -> Color {
    let num_lights =
        scene.delta_lights.len() + scene.area_lights.len() + (scene.has_env_light() as usize);
    if num_lights == 0 {
        return Color::black();
    }
    let light_pdf = 1.0f32 / num_lights as f32;
    let mut rng = rand::thread_rng();
    let chosen_index = rng.gen_range(0..num_lights);
    let rnd2_light = (rng.gen::<f32>(), rng.gen::<f32>());
    let rnd2_scatter = (rng.gen::<f32>(), rng.gen::<f32>());
    let one_light_incident_radiance = match chosen_index {
        x if x < scene.delta_lights.len() => {
            let chosen_light = &scene.delta_lights[chosen_index];
            estimate_direct_delta_light(hit, mtl, chosen_light, rnd2_light, scene)
        }
        x if x >= scene.delta_lights.len() && x < scene.area_lights.len() => {
            let chosen_light = &scene.area_lights[x - scene.delta_lights.len()];
            estimate_direct_area_light(hit, mtl, rnd2_scatter, chosen_light, rnd2_light, scene)
        }
        _ => {
            let bxdfs = mtl.bxdfs_at(hit);
            assert!(!bxdfs.is_empty());
            let bsdf = BSDF::new_frame(hit).with_bxdfs(&bxdfs);
            let (f, wi, pr) = bsdf.sample(hit.wo, rnd2_scatter);
            let incident_ray = hit.spawn_ray(wi);

            let incident_radiance = match scene.tlas.occludes(&incident_ray) {
                false => scene.eval_env_light(incident_ray),
                _ => Color::black(),
            };
            let pr = match pr {
                Prob::Density(d) => d,
                Prob::Mass(m) => m,
            };
            incident_radiance * f * wi.dot(hit.normal).abs() * pr.weak_recip()
        }
    };
    one_light_incident_radiance * (1.0 / light_pdf)
}

fn estimate_direct_delta_light(
    hit: &Interaction, mtl: &dyn Material, light: &DeltaLight, rnd2_light: (f32, f32),
    scene: &Scene,
) -> Color {
    // Uses multiple-importance sampling technique to combine the contribution of 2 different
    // sampling strategies:
    //                        f(X_1)                f(X_2)
    // Estimate = weight_1 ---------- + weight_2 ----------
    //                       p1(X_1)               p2(X_2)
    // where
    //  - X_1 and X_2 are sampled from different distributions;
    //  - p1(X) and p2(X) are the corresponding pdf of the distributions;
    //  - weight_1 + weight_2 is approximately 1 (somewhat).
    // It can be easily generalized to more than 2 distributions.
    //
    // In this method, the 2 different sampling strategies come from the strategy of sampling the
    // light and sampling the BSDF. The weights are determined by the power-2-heuristic:
    //
    // weight_i(X_i) = p_i(X_i)^2 / sum_i(p_i(X_i)^2)
    //
    // Since the light is delta light, the 2nd sampling strategy (by BSDF) will always make a zero
    // contribution. This part is skipped.

    let bxdfs = mtl.bxdfs_at(hit);
    assert!(!bxdfs.is_empty());
    let bsdf = BSDF::new_frame(hit).with_bxdfs(&bxdfs);

    // Computes weight, estimated function value, and the probability.
    // Returns `None` on: 0 prob of bsdf, either Li or bsdf is black, or light is occluded.
    let by_light = || -> Option<(f32, Color, f32)> {
        let (light_radiance, wi, light_pr, mut vis_test_ray) =
            light.sample_incident_radiance(hit, rnd2_light);
        let bsdf_value = bsdf.eval(hit.wo, wi) * hit.normal.dot(hit.wo).abs();
        if !light_pr.is_positive() || light_radiance.is_black() || bsdf_value.is_black() {
            return None;
        }
        let scatter_pdf = bsdf.pdf(hit.wo, wi);
        scene.tlas.intersect(&mut vis_test_ray)?;
        let (weight, pr) = match light_pr {
            Prob::Mass(pmf) => (1.0, pmf),
            Prob::Density(pdf) => (square_heuristic(1.0, pdf, 1.0, scatter_pdf), pdf),
        };
        Some((weight, bsdf_value * light_radiance, pr))
    };

    if let Some((weight, f, pr)) = by_light() {
        f * weight * pr.weak_recip()
    } else {
        Color::black()
    }
}

fn estimate_direct_area_light(
    hit: &Interaction, mtl: &dyn Material, rnd2_scatter: (f32, f32), light: &DiffuseAreaLight,
    rnd2_light: (f32, f32), scene: &Scene,
) -> Color {
    let mut radiance_d = Color::black();

    // Uses multiple-importance sampling technique to combine the contribution of 2 different
    // sampling strategies:
    //                        f(X_1)                f(X_2)
    // Estimate = weight_1 ---------- + weight_2 ----------
    //                       p1(X_1)               p2(X_2)
    // where
    //  - X_1 and X_2 are sampled from different distributions;
    //  - p1(X) and p2(X) are the corresponding pdf of the distributions;
    //  - weight_1 + weight_2 is approximately 1 (somewhat).
    // It can be easily generalized to more than 2 distributions.
    //
    // In this method, the 2 different sampling strategies come from the strategy of sampling the
    // light and sampling the BSDF. The weights are determined by the power-2-heuristic:
    //
    // weight_i(X_i) = p_i(X_i)^2 / sum_i(p_i(X_i)^2)

    let bxdfs = mtl.bxdfs_at(hit);
    let bsdf = BSDF::new_frame(hit).with_bxdfs(&bxdfs);

    let (light_radiance, wi, light_pr, vis_test_ray) =
        light.sample_incident_radiance(hit, rnd2_light);
    assert!(light_pr.is_density());
    if light_pr.is_positive() && !light_radiance.is_black() {
        let light_pdf = light_pr.density();
        // Evaluates f(wo, wi) * |cos(theta)|.
        let bsdf_value = bsdf.eval(hit.wo, wi) * hit.normal.dot(hit.wo).abs();
        let scatter_pdf = bsdf.pdf(hit.wo, wi);
        // Adds light's contribution to reflected radiance, if both BSDF value and Li are non-black,
        // and the incident ray isn't occluded. If-statement shortcut is used to avoid unnecessary
        // checking for ray occlusion.
        if !bsdf_value.is_black() && scatter_pdf > 0.0 && !scene.tlas.occludes(&vis_test_ray) {
            let weight = square_heuristic(1.0, light_pdf, 1.0, scatter_pdf);
            radiance_d += bsdf_value * light_radiance * weight * light_pdf.weak_recip();
        }
    }

    // Samples BSDF with MIS.
    let by_bsdf = || -> Option<(f32, Color, f32)> {
        let (bsdf_value, wi, bsdf_pr) = bsdf.sample(hit.wo, rnd2_scatter);
        let bsdf_value = bsdf_value * hit.normal.dot(wi).abs();

        if bsdf_value.is_black() || !bsdf_pr.is_positive() {
            return None;
        }
        let (incident_radiance, light_pdf, vis_test_ray) = light.radiance_to(hit, wi)?;
        if incident_radiance.is_black() || light_pdf <= 0.0 || scene.tlas.occludes(&vis_test_ray) {
            return None;
        }

        let (weight, pr) = match bsdf_pr {
            Prob::Mass(pmf) => (1.0, pmf),
            Prob::Density(pdf) => (square_heuristic(1.0, pdf, 1.0, light_pdf), pdf),
        };

        Some((weight, bsdf_value * incident_radiance, pr))
    };

    if let Some((weight, f, pr)) = by_bsdf() {
        radiance_d += weight * f * pr.weak_recip();
    }
    radiance_d
}

fn square_heuristic(nf: f32, f_pdf: f32, ng: f32, g_pdf: f32) -> f32 {
    power_heuristic::<2>(nf, f_pdf, ng, g_pdf)
}

fn power_heuristic<const BETA: i32>(nf: f32, f_pdf: f32, ng: f32, g_pdf: f32) -> f32 {
    let f = nf * f_pdf;
    let g = ng * g_pdf;
    f.powi(BETA) / (f.powi(BETA) + g.powi(BETA))
}

pub fn material_visualizer(scene: &Scene, mut ray: ray::Ray, _depth: i32) -> Color {
    let pal: [Color; 10] = [
        Color::rgb(232, 207, 59),
        Color::rgb(124, 188, 126),
        Color::rgb(30, 68, 176),
        Color::rgb(15, 142, 205),
        Color::rgb(44, 180, 172),
        Color::rgb(216, 39, 252),
        Color::rgb(143, 112, 252),
        Color::gray(0.3),
        Color::gray(0.9),
        Color::black(),
    ];
    if let Some((_, mtl)) = scene.tlas.intersect(&mut ray) {
        let index = match mtl.summary() {
            x if x.contains("Lambertian") => 8,
            x if x.contains("Metal") => 7,
            x if x.contains("Fourier") => 6,
            x if x.contains("Mirror") => 5,
            x if x.contains("Dielectric") => 4,
            x if x.contains("DiffuseLight") => 3,
            x if x.contains("uber") => 2,
            x if x.contains("substrate") => 1,
            x if x.contains("plastic") => 0,
            _ => 9,
        };
        pal[index as usize]
    } else {
        let (x, y, _) = math::hcm::Point3::from(ray.dir).as_triple();

        let parity = (x * 50.0).floor() as i32 + (y * 50.0).floor() as i32;
        if parity % 2 == 0 {
            Color::gray(0.9)
        } else {
            Color::gray(0.7)
        }
    }
}

pub fn normal_visualizer(scene: &Scene, mut ray: ray::Ray, _depth: i32) -> Color {
    fn vec3_to_color(v: math::hcm::Vec3) -> Color {
        Color::new(v.x, v.y, v.z)
    }

    let hit_info = scene.tlas.intersect(&mut ray);

    match hit_info {
        None => scene.eval_env_light(ray),
        Some((hit, mtl)) => {
            let (_, albedo) = mtl.scatter(-ray.dir, &hit);
            (albedo + vec3_to_color(hit.normal)) * 0.5
            // albedo
            // (Color::from(hit.normal) + Color::white()) * 0.5
        }
    }
}
