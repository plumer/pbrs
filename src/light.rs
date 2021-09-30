use std::f32::consts;

use math::hcm;
use math::prob::Prob;
use radiometry::color::Color;
use crate::ray::Ray;
use crate::shape::{self, Interaction, Shape};

fn spawn_ray_to(p0: &Interaction, p1: hcm::Point3) -> Ray {
    Ray::new(p0.pos, p1 - p0.pos)
}

pub trait Light {
    /// Computes the radiance that the light emits to a given point assuming no occlusion.
    /// - Returns the incident direction, the probability that the incident direction gets sampled,
    ///   and the ray to test the visibility.
    fn sample_incident_radiance(
        &self,
        target: &Interaction,
        u: (f32, f32),
    ) -> (Color, hcm::Vec3, Prob, Ray);

    fn power(&self) -> Color;
}

pub trait AreaLight: Light {
    /// Evaluates the area light's emitted radiance in the given outgoing direction.
    /// Parameters:
    /// - `from` is the point being evaluated, assumed to be on the surface.
    fn radiance_from(&self, from: &Interaction, wo: hcm::Vec3) -> Color;
}

// Various kinds of lights.

struct PointLight {
    position: hcm::Point3,
    intensity: Color,
}

impl Light for PointLight {
    fn sample_incident_radiance(
        &self,
        target: &Interaction,
        _u: (f32, f32),
    ) -> (Color, hcm::Vec3, Prob, Ray) {
        let radiance = self.intensity / self.position.squared_distance_to(target.pos);
        let wi = (self.position - target.pos).hat();
        let visibility_ray = spawn_ray_to(target, self.position);
        (radiance, wi, Prob::Mass(1.0), visibility_ray)
    }

    fn power(&self) -> Color {
        self.intensity
    }
}

pub struct DistantLight {
    _world_center: hcm::Point3,
    world_radius: f32,
    incident_direction: hcm::Vec3,
    radiance: Color,
}

impl Light for DistantLight {
    fn sample_incident_radiance(
        &self,
        target: &Interaction,
        _u: (f32, f32),
    ) -> (Color, hcm::Vec3, Prob, Ray) {
        let outside_world = target.pos + self.world_radius * 2.0 * self.incident_direction;
        let visibility_ray = spawn_ray_to(target, outside_world);
        (
            self.radiance,
            self.incident_direction,
            Prob::Mass(1.0),
            visibility_ray,
        )
    }

    fn power(&self) -> Color {
        let area = consts::PI * self.world_radius.powi(2);
        area * self.radiance
    }
}

/// A trait where points can be sampled from the surface of a shape.
pub trait ShapeSample: Shape {
    /// Consumes a 2D uniform [0, 1) random variable and produces a point on the shape surface,
    /// using a sampling distribution uniform to surface area, with local geometric information
    /// about the sampled point.
    fn sample(&self, rnd2: (f32, f32)) -> Interaction;

    /// Produces a random point on the shape surface towards a point in space. A reference point is
    /// used to ensure that only some portion of the shape potentially visible from the point is
    /// sampled.
    ///
    /// A 2D uniform [0, 1) random variable is used with a sampling distribution uniform to surface
    /// area.
    ///
    fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction;

    /// PDF of `wi` lies in the solid angle that this shape casts onto the reference point.
    ///
    /// Assuming the shape is fairly far away from the reference point, the solid angle is
    /// distance divided by perpendicular area.
    fn pdf_at(&self, reference: &Interaction, wi: hcm::Vec3) -> f32 {
        let ray = Ray::new(reference.pos, wi);
        match self.intersect(&ray) {
            None => 0.0,
            Some(hit) => {
                reference.pos.distance_to(hit.pos) / (hit.normal.dot(-wi).abs() * self.area())
            }
        }
    }

    fn area(&self) -> f32;
}

/// Implements a basic area light source with a uniform spatial and directi onal radiance
/// distribution. Owns a `shape` that defines its surface.
pub struct DiffuseAreaLight {
    emit_radiance: Color,
    shape: Box<dyn ShapeSample>,
    area: f32,
}

impl DiffuseAreaLight {
    pub fn new(emit_radiance: Color, shape: Box<dyn ShapeSample>) -> Self {
        let area = shape.area();
        Self{emit_radiance, shape, area}
    }
}

impl Light for DiffuseAreaLight {
    fn sample_incident_radiance(
        &self,
        target: &Interaction,
        u: (f32, f32),
    ) -> (Color, hcm::Vec3, Prob, Ray) {
        let point_on_light = self.shape.sample_towards(target, u);
        let wi = (point_on_light.pos - target.pos).hat();
        let radiance = self.radiance_from(target, -wi);
        let pdf = self.shape.pdf_at(target, wi);
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

impl AreaLight for DiffuseAreaLight {
    fn radiance_from(&self, from: &Interaction, wo: hcm::Vec3) -> Color {
        if from.normal.dot(wo).is_sign_positive() {
            self.emit_radiance
        } else {
            Color::black()
        }
    }
}

/// Implementations of ShapeSample for various shapes

impl ShapeSample for shape::Sphere {
    fn sample(&self, rnd2: (f32, f32)) -> Interaction {
        let (u, v) = rnd2;
        let theta = 2.0 * std::f32::consts::PI * u;
        let phi = (2.0 * v - 1.0).acos();
        let dir = hcm::Vec3::new(
            phi.sin() * theta.cos(),
            phi.sin() * theta.sin(),
            2.0 * v - 1.0,
        );
        Interaction::new(self.center() + self.radius() * dir, 0.0, rnd2, dir)
    }

    fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction {
        let wc = self.center() - target.pos; // Target to sphere center.
        if wc.norm_squared() < self.radius().powi(2) {
            return self.sample(rnd2);
        }
        // Viewing from the reference position to the sphere center,
        let sin_theta_max_2 = self.radius().powi(2) / wc.norm_squared();
        let cos_theta_max = (1.0 - sin_theta_max_2).max(0.0).sqrt();

        let (u, v) = rnd2;
        // |   1 - u   | u
        // +-----------+----*----+
        let cos_theta = (1.0 - u) + u * cos_theta_max;
        let sin_theta_2 = (1.0 - cos_theta.powi(2)).max(0.0);
        let phi = v * 2.0 * std::f32::consts::PI;

        let dc = wc.norm();
        let ds = dc * cos_theta
            - (self.radius().powi(2) - wc.norm_squared() * sin_theta_2)
                .max(0.0)
                .sqrt();
        let cos_alpha =
            (wc.norm_squared() + self.radius().powi(2) - ds.powi(2)) / (2.0 * dc * self.radius());
        let sin_alpha = (1.0 - cos_alpha.powi(2)).max(0.0).sqrt();
        let normal_object_space = hcm::spherical_direction(sin_alpha, cos_alpha, hcm::Radian(phi));
        let (wcx, wcy) = hcm::make_coord_system(wc.hat());
        let point_on_sphere =
            hcm::Mat3::from_vectors(wcx, wcy, wc) * normal_object_space * self.radius()
                + self.center();

        Interaction::new(point_on_sphere, 0.0, rnd2, normal_object_space)
    }

    fn area(&self) -> f32 {
        self.radius().powi(2) * 4.0 * std::f32::consts::PI
    }

    fn pdf_at(&self, reference: &Interaction, wi: hcm::Vec3) -> f32 {
        let ref_to_center = self.center() - reference.pos;
        if ref_to_center.norm_squared() < self.radius().powi(2) {
            1.0 / self.area()
        } else {
            let sin_theta_max_2 = self.radius().powi(2) / ref_to_center.norm_squared();
            let cos_theta_max = (1.0 - sin_theta_max_2).max(0.0).sqrt();
            let cos_theta = ref_to_center.dot(wi) / (ref_to_center.norm() * wi.norm());
            let uniform_cone_pdf =
                |cos_theta_max| 1.0 / (2.0 * std::f32::consts::PI * (1.0 - cos_theta_max));
            if cos_theta > cos_theta_max {
                // Theta is less than theta_max, `wi` inside the cone
                uniform_cone_pdf(cos_theta_max)
            } else {
                0.0
            }
        }
    }
}

impl ShapeSample for shape::IsolatedTriangle {
    fn sample(&self, rnd2: (f32, f32)) -> Interaction {
        let (u, v) = rnd2;
        let (u, v) = if u + v > 1.0 {
            (1.0 - v, 1.0 - u)
        } else {
            (u, v)
        };
        let position = self.p0 + (self.p1 - self.p0) * u + (self.p2 - self.p0) * v;
        let normal = (self.p0 - self.p1).cross(self.p2 - self.p1).hat();
        Interaction::new(position, 0.0, (u, v), normal)
    }
    fn sample_towards(&self, _target: &Interaction, rnd2: (f32, f32)) -> Interaction {
        self.sample(rnd2)
    }
    fn area(&self) -> f32 {
        (self.p0 - self.p1).cross(self.p2 - self.p1).norm() * 0.5
    }
}
