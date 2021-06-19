use std::f32::consts;

use crate::hcm;
use crate::image::Color;
use crate::ray::Ray;
use crate::shape::Interaction;
use crate::shape::Shape;

pub enum HemiPdf {
    Delta,
    Regular(f32),
}

fn spawn_ray_to(p0: &Interaction, p1: hcm::Point3) -> Ray {
    Ray::new(p0.pos, p1 - p0.pos)
}

pub trait Light {
    /// Computes the radiance arriving at a given point assuming no occlusion, and returns it with
    /// the probabability of sampling it.
    /// @return  The incident direction, the pdf, and the ray to test the visibility.
    fn sample_incident_radiance(
        &self,
        isect: &Interaction,
        u: (f32, f32),
    ) -> (Color, hcm::Vec3, HemiPdf, Ray);

    fn power(&self) -> Color;
}

pub trait AreaLight: Light {
    /// Evaluates the area light's emitted radiance in the given outgoing direction.
    /// Parameters:
    /// * `intr` is the point being evaluated, assumed to be on the surface.
    fn radiance(&self, intr: &Interaction, wo: hcm::Vec3) -> Color;
}

// Various kinds of lights.

struct PointLight {
    position: hcm::Point3,
    intensity: Color,
}

impl Light for PointLight {
    fn sample_incident_radiance(
        &self,
        isect: &Interaction,
        _u: (f32, f32),
    ) -> (Color, hcm::Vec3, HemiPdf, Ray) {
        let radiance = self.intensity / self.position.squared_distance_to(isect.pos);
        let wi = (self.position - isect.pos).hat();
        let visibility_ray = spawn_ray_to(isect, self.position);
        (radiance, wi, HemiPdf::Delta, visibility_ray)
    }

    fn power(&self) -> Color {
        self.intensity
    }
}

struct DistantLight {
    _world_center: hcm::Point3,
    world_radius: f32,
    incident_direction: hcm::Vec3,
    radiance: Color,
}

impl Light for DistantLight {
    fn sample_incident_radiance(
        &self,
        isect: &Interaction,
        u: (f32, f32),
    ) -> (Color, hcm::Vec3, HemiPdf, Ray) {
        let outside_world = isect.pos + self.world_radius * 2.0 * self.incident_direction;
        let visibility_ray = spawn_ray_to(isect, outside_world);
        (
            self.radiance,
            self.incident_direction,
            HemiPdf::Delta,
            visibility_ray,
        )
    }

    fn power(&self) -> Color {
        let area = consts::PI * self.world_radius.powi(2);
        area * self.radiance
    }
}

/// Implements a basic area light source with a uniform spatial and directional radiance
/// distribution. Owns a `shape` that defines its surface.
struct DiffuseAreaLight {
    emit_radiance: Color,
    shape: Box<dyn Shape>,
    area: f32,
}

impl Light for DiffuseAreaLight {
    fn sample_incident_radiance(
        &self,
        isect: &Interaction,
        u: (f32, f32),
    ) -> (Color, hcm::Vec3, HemiPdf, Ray) {
        todo!()
    }
    fn power(&self) -> Color {
        self.emit_radiance * self.area * std::f32::consts::PI
    }
}

impl AreaLight for DiffuseAreaLight {
    fn radiance(&self, intr: &Interaction, wo: hcm::Vec3) -> Color {
        if intr.normal.dot(wo).is_sign_positive() {
            self.emit_radiance
        } else {
            Color::black()
        }
    }
}
