mod sample_shape;

use geometry::ray::Ray;
use geometry::Interaction;
use math::prob::Prob;
use math::{float::Float, hcm};
use radiometry::color::Color;
use shape::Shape;
use std::f32::consts::{self, PI};

pub use sample_shape::{SamplableShape, ShapeSample};

pub type EnvLight = fn(Ray) -> Color;

fn spawn_ray_to(p0: &Interaction, p1: hcm::Point3) -> Ray {
    let mut r = p0.spawn_ray(p1 - p0.pos);
    r.t_max = p1.distance_to(p0.pos) / r.dir.norm_squared();
    r
}

pub trait Light {
    /// Computes the radiance that the light emits to a given point regardless of occlusion.
    /// - Returns the incident direction, the probability that the incident direction gets sampled,
    ///   and the ray to test the visibility.
    fn sample_incident_radiance(
        &self, target: &Interaction, u: (f32, f32),
    ) -> (Color, hcm::Vec3, Prob, Ray);

    fn power(&self) -> Color;
}

// Various kinds of lights.

#[derive(Debug, Clone, Copy)]
pub enum DeltaLight {
    Point {
        position: hcm::Point3,
        intensity: Color,
    },
    Distant {
        world_radius: f32,
        incident_direction: hcm::Vec3,
        radiance: Color,
    },
}

impl DeltaLight {
    /// Creates a point light with the given position and intensity of the light.
    pub fn point(position: hcm::Point3, intensity: Color) -> Self {
        Self::Point {
            position,
            intensity,
        }
    }

    /// Creates a distant light with given direction and radiance.
    /// Usually used to model massively point lights that are very far away (e.g., sun light).
    ///
    /// The `world_radius` is the radius of the bounding sphere of the entire scene. It is used for
    /// computing a visibility tester ray. If not sure, use `f32::INFINITY` for the radius.
    pub fn distant(world_radius: f32, incident_direction: hcm::Vec3, radiance: Color) -> Self {
        Self::Distant {
            world_radius,
            incident_direction,
            radiance,
        }
    }
}

#[rustfmt::skip]
impl Light for DeltaLight {
    fn sample_incident_radiance(
        &self, target: &Interaction, _u: (f32, f32),
    ) -> (Color, hcm::Vec3, Prob, Ray) {
        match self.clone() {
            Self::Point {position, intensity} => {
                let radiance = intensity * position.squared_distance_to(target.pos).weak_recip();
                let wi = (position - target.pos).hat();
                let visibility_ray = spawn_ray_to(target, position);
                (radiance, wi, Prob::Mass(1.0), visibility_ray)
            }
            Self::Distant {world_radius, incident_direction, radiance} => {
                let outside_world = target.pos + world_radius * 2.0 * incident_direction;
                let visibility_ray = spawn_ray_to(target, outside_world);
                (
                    radiance,
                    incident_direction,
                    Prob::Mass(1.0),
                    visibility_ray,
                )
            }
        }
    }

    fn power(&self) -> Color {
        match self.clone() {
            Self::Point {position: _, intensity} => intensity * 4.0 * PI,
            Self::Distant {world_radius, radiance, ..} => {
                let area = consts::PI * world_radius.powi(2);
                area * radiance
            }
        }
    }
}

/// Implements a basic area light source with a uniform spatial and directi onal radiance
/// distribution. Owns a `shape` that defines its surface.
pub struct DiffuseAreaLight {
    emit_radiance: Color,
    shape: SamplableShape,
    area: f32,
}

impl DiffuseAreaLight {
    pub fn new(emit_radiance: Color, shape: SamplableShape) -> Self {
        let area = shape.area();
        Self {
            emit_radiance,
            shape,
            area,
        }
    }

    /// Evaluates the area light's emitted radiance in the given outgoing direction.
    /// Parameters:
    ///  - `from` is the point being evaluated, assumed to be on the surface.
    /// Corresponds to `AreaLight::L(Interaction, Vec3)` in C++ version.
    fn radiance_from(&self, from: &Interaction, wo: hcm::Vec3) -> Color {
        if from.normal.dot(wo).is_sign_positive() {
            self.emit_radiance
        } else {
            Color::black()
        }
    }

    /// Computes the radiance along direction `wi` towards `target`.
    ///
    /// Returns `None` if the ray `target` + t`wi` doesn't hit the underlying shape of the area
    /// light (regardless of occlusion).
    /// Otherwise, returns the incident radiance, together with the PDF that `wi` is covered, and
    /// a ray to test the visibility from `target` to the light shape.
    pub fn radiance_to(&self, target: &Interaction, wi: hcm::Vec3) -> Option<(Color, f32, Ray)> {
        let light_hit = self.shape.intersect(&target.spawn_ray(wi))?;
        let light_pdf = self.pdf_li(target, wi)?;
        let vis_test_ray = target.spawn_limited_ray_to(light_hit.pos);
        Some((self.emit_radiance, light_pdf, vis_test_ray))
    }
    /// Computes the probability density that the `target` intersection is illuminated by the
    /// light at angle `wi`.
    pub fn pdf_li(&self, target: &Interaction, wi: hcm::Vec3) -> Option<f32> {
        self.shape.pdf_at(target, wi)
    }
}

impl Light for DiffuseAreaLight {
    /// Computes the radiance that the light emits to a given point regardless of occlusion.
    /// - Returns the incident direction, the probability that the incident direction gets sampled,
    ///   and the ray to test the visibility.
    fn sample_incident_radiance(
        &self, target: &Interaction, u: (f32, f32),
    ) -> (Color, hcm::Vec3, Prob, Ray) {
        let point_on_light = self.shape.sample_towards(target, u);
        let wi = (point_on_light.pos - target.pos).hat();
        let radiance = self.radiance_from(&point_on_light, -wi);
        let pdf = self.shape.pdf_at(target, wi).unwrap_or(0.0);
        //.expect("Shoul be nonzero");
        (
            radiance,
            wi,
            Prob::Density(pdf),
            spawn_ray_to(target, point_on_light.pos),
        )
    }

    fn power(&self) -> Color {
        self.emit_radiance * self.area * std::f32::consts::PI
    }
}
