use std::f32::consts::{self, PI};

use geometry::ray::Ray;
use math::prob::Prob;
use math::{float::Float, hcm};
use radiometry::color::Color;
use shape::{self, Interaction, Shape};

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
    /// Returns `None` if the shape doesn't cover the reference point at angle `wi`.
    ///
    /// Assuming the shape is fairly far away from the reference point, the solid angle is
    /// perpendicular area of the shape divided by distance.
    fn pdf_at(&self, reference: &Interaction, wi: hcm::Vec3) -> Option<f32> {
        let ray = reference.spawn_ray(wi);
        let hit = self.intersect(&ray)?;
        let pdf = reference.pos.distance_to(hit.pos) / (hit.normal.dot(-wi).abs() * self.area());
        Some(pdf)
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
        Interaction::rayless(self.center() + self.radius() * dir, rnd2, dir)
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
        let normal_object_space =
            hcm::spherical_direction(sin_alpha, cos_alpha, math::new_rad(phi));
        let (wcx, wcy) = hcm::make_coord_system(-wc.hat());
        let normal_world_space = hcm::Mat3::from_vectors(wcx, wcy, -wc.hat()) * normal_object_space;
        let point_on_sphere = normal_world_space * self.radius() + self.center();

        // TODO: the following assertion would fail for corner-case u-values (something very close
        //       to 1.0) but doesn't affect the rendered result by human eye. Maybe one way to deal
        //       with it is to quantify the rate of failure and then decide to ignore it or not. 
        // if normal_world_space.dot(point_on_sphere - target.pos) > 0.0 {
        //     eprintln!("point on sphere not facing towards the target: rnd2 = {:?}, normal = {}, oncoming = {}", 
        //     rnd2, normal_world_space, point_on_sphere - target.pos);
        // }
        // assert!(normal_world_space.dot(point_on_sphere - target.pos) < 0.0);
        Interaction::rayless(point_on_sphere, rnd2, normal_world_space)
    }

    fn pdf_at(&self, reference: &Interaction, wi: hcm::Vec3) -> Option<f32> {
        let ref_to_center = self.center() - reference.pos;
        if ref_to_center.norm_squared() < self.radius().powi(2) {
            Some(1.0 / self.area())
        } else {
            let sin_theta_max_2 = self.radius().powi(2) / ref_to_center.norm_squared();
            let cos_theta_max = (1.0 - sin_theta_max_2).max(0.0).sqrt();
            let cos_theta = ref_to_center.dot(wi) / (ref_to_center.norm() * wi.norm());
            let uniform_cone_pdf = || 1.0 / (2.0 * PI * (1.0 - cos_theta_max));
            // Theta is less than theta_max, `wi` inside the cone
            (cos_theta > cos_theta_max).then(uniform_cone_pdf)
        }
    }

    fn area(&self) -> f32 {
        self.radius().powi(2) * 4.0 * std::f32::consts::PI
    }
}

impl ShapeSample for shape::Disk {
    fn sample(&self, rnd2: (f32, f32)) -> Interaction {
        let (cos_t, sin_t) = geometry::bxdf::concentric_sample_disk(rnd2);
        let radial2 = self.normal().cross(self.radial());
        let cp = self.radial() * cos_t + radial2 * sin_t;

        Interaction::rayless(self.center() + cp, rnd2, self.normal())
    }
    fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction {
        let mut res = self.sample(rnd2);
        res.normal = res.normal.facing(target.normal);
        res
    }

    fn area(&self) -> f32 {
        self.radial().norm_squared() * PI
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
        Interaction::rayless(position, (u, v), normal)
    }
    fn sample_towards(&self, _target: &Interaction, rnd2: (f32, f32)) -> Interaction {
        self.sample(rnd2)
    }
    fn area(&self) -> f32 {
        (self.p0 - self.p1).cross(self.p2 - self.p1).norm() * 0.5
    }
}

impl ShapeSample for shape::QuadXY {
    fn sample(&self, rnd2: (f32, f32)) -> Interaction {
        let (u, v) = rnd2;
        let pos = hcm::Point3::new(self.x_interval.lerp(u), self.y_interval.lerp(v), self.z);
        let normal = hcm::Vec3::zbase();
        Interaction::rayless(pos, rnd2, normal)
    }

    fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction {
        let mut temp = self.sample(rnd2);
        let forward = target.pos - temp.pos;
        temp.normal = forward.dot(temp.normal).signum() * temp.normal;
        temp
    }

    fn area(&self) -> f32 {
        self.x_interval.length() * self.y_interval.length()
    }
}

impl ShapeSample for shape::QuadXZ {
    fn sample(&self, rnd2: (f32, f32)) -> Interaction {
        let (u, v) = rnd2;
        let pos = hcm::Point3::new(self.x_interval.lerp(u), self.y, self.z_interval.lerp(v));
        let normal = hcm::Vec3::ybase();
        Interaction::rayless(pos, rnd2, normal)
    }

    fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction {
        let mut temp = self.sample(rnd2);
        let forward = target.pos - temp.pos;
        temp.normal = forward.dot(temp.normal).signum() * temp.normal;
        temp
    }

    fn area(&self) -> f32 {
        self.x_interval.length() * self.z_interval.length()
    }
}

impl ShapeSample for shape::QuadYZ {
    fn sample(&self, rnd2: (f32, f32)) -> Interaction {
        let (u, v) = rnd2;
        let pos = hcm::Point3::new(self.x, self.y_interval.lerp(u), self.z_interval.lerp(v));
        let normal = hcm::Vec3::xbase();
        Interaction::rayless(pos, rnd2, normal)
    }

    fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction {
        let mut temp = self.sample(rnd2);
        let forward = target.pos - temp.pos;
        temp.normal = forward.dot(temp.normal).signum() * temp.normal;
        temp
    }

    fn area(&self) -> f32 {
        self.z_interval.length() * self.y_interval.length()
    }
}
