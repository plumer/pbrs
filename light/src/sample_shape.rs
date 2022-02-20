use geometry::{bvh::BBox, ray::Ray};
use geometry::{transform::Transform, Interaction};
use math::hcm;
use shape::{self, ParallelQuad, Shape};
use std::f32::consts::PI;

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

pub enum SamplableShape {
    Sphere(shape::Sphere),
    Disk(shape::Disk),
    Triangle(shape::IsolatedTriangle),
    Quad(shape::ParallelQuad),
}

impl SamplableShape {
    pub fn transformed_by(self, t: geometry::AffineTransform) -> Self {
        /// Verifies that the transform is orthogonal and uniformly-scaled. Panics if not; returns
        /// the scale and the rotation otherwise.
        fn eigen(t: geometry::AffineTransform) ->(f32, hcm::Mat3) {
            let tx = t.apply(hcm::Vec3::X);
            let ty = t.apply(hcm::Vec3::Y);
            let tz = t.apply(hcm::Vec3::Z);
            
            let scale = tx.cross(ty).dot(tz).cbrt();
            assert!(scale > 0.0);
            let rotation = hcm::Mat3::from_cols(tx, ty, tz) * scale.recip();
            if (rotation * rotation.transpose() - hcm::Mat3::IDENTITY).frobenius_norm_squared() > 1e-3 {
                panic!("not good rotation");
            }
            (scale, rotation)
        }

        match self {
            Self::Triangle(tri) => {
                let p0 = t.apply(tri.p0);
                let p1 = t.apply(tri.p1);
                let p2 = t.apply(tri.p2);
                Self::Triangle(shape::IsolatedTriangle::new(p0, p1, p2))
            }
            Self::Disk(disk) => {
                let (scale, rotation) = eigen(t);
                let new_center = t.apply(disk.center());
                let new_normal = rotation * disk.normal();
                let new_radial = rotation * disk.radial() * scale;
                Self::Disk(shape::Disk::new(new_center, new_normal, new_radial))
            }
            Self::Sphere(sphere) => {
                let (scale, _) = eigen(t);
                let new_center = t.apply(sphere.center());
                let new_radius = sphere.radius() * scale;
                Self::Sphere(shape::Sphere::new(new_center, new_radius))
            }
            Self::Quad(quad) => {
                let origin = t.apply(quad.origin);
                let side_u = t.apply(quad.side_u);
                let side_v = t.apply(quad.side_v);
                Self::Quad(ParallelQuad {
                    origin,
                    side_u,
                    side_v,
                })
            }
        }
    }
}

macro_rules! impl_shape_method {
    ($method_name: ident, $return_type: ty) => {
        fn $method_name(&self) -> $return_type {
            match self {
                Self::Disk(d) => d.$method_name(),
                Self::Sphere(s) => s.$method_name(),
                Self::Triangle(tri) => tri.$method_name(),
                Self::Quad(quad) => quad.$method_name(),
            }
        }
    };
    ($method_name:ident, {$($arg_name:ident : $type:ty),+}, $return_type:ty) => {
        fn $method_name(&self, $($arg_name: $type),+) -> $return_type {
            match self {
                Self::Disk(d) => d.$method_name($($arg_name),+),
                Self::Sphere(s) => s.$method_name($($arg_name),+),
                Self::Triangle(tri) => tri.$method_name($($arg_name),+),
                Self::Quad(quad) => quad.$method_name($($arg_name),+),
            }
        }
    };
}

impl Shape for SamplableShape {
    impl_shape_method!(summary, String);
    impl_shape_method!(bbox, BBox);
    impl_shape_method!(occludes, { r: &Ray }, bool);
    impl_shape_method!(intersect, { r: &Ray }, Option<Interaction>);
}
impl ShapeSample for SamplableShape {
    impl_shape_method!(sample, { rnd2: (f32, f32) }, Interaction);
    impl_shape_method!(sample_towards, {target : &Interaction, rnd2 : (f32, f32)}, Interaction);
    impl_shape_method!(pdf_at, {target: &Interaction, wi: hcm::Vec3}, Option<f32>);
    impl_shape_method!(area, f32);
    // fn sample(&self, rnd2: (f32, f32)) -> Interaction {
    //     match self {
    //         Self::Disk(d) => d.sample(rnd2),
    //         Self::Sphere(s) => s.sample(rnd2),
    //         Self::Triangle(tri) => tri.sample(rnd2),
    //     }
    // }
    // fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction {
    //     match self {
    //         Self::Disk(d) => d.sample_towards(target, rnd2),
    //         Self::Sphere(s) => s.sample_towards(target, rnd2),
    //         Self::Triangle(tri) => tri.sample_towards(target, rnd2),
    //     }
    // }
    // fn pdf_at(&self, target: &Interaction, wi: hcm::Vec3) -> Option<f32> {
    //     match self {
    //         Self::Disk(d) => d.pdf_at(target, wi),
    //         Self::Sphere(s) => s.pdf_at(target, wi),
    //         Self::Triangle(tri) => tri.pdf_at(target, wi),
    //     }
    // }
    // fn area(&self) -> f32 {
    //     match self {
    //         Self::Disk(d) => d.area(),
    //         Self::Sphere(s) => s.area(),
    //         Self::Triangle(tri) => tri.area(),
    //     }
    // }
}

impl From<shape::Sphere> for SamplableShape {
    fn from(s: shape::Sphere) -> Self {
        Self::Sphere(s)
    }
}
impl From<shape::Disk> for SamplableShape {
    fn from(disk: shape::Disk) -> Self {
        Self::Disk(disk)
    }
}
impl From<shape::IsolatedTriangle> for SamplableShape {
    fn from(tri: shape::IsolatedTriangle) -> Self {
        Self::Triangle(tri)
    }
}
impl From<shape::ParallelQuad> for SamplableShape {
    fn from(q: shape::ParallelQuad) -> Self {
        Self::Quad(q)
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
        let normal_world_space = hcm::Mat3::from_cols(wcx, wcy, -wc.hat()) * normal_object_space;
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

impl ShapeSample for shape::ParallelQuad {
    fn sample(&self, rnd2: (f32, f32)) -> Interaction {
        let (u, v) = rnd2;
        let position = self.origin + u * self.side_u + v * self.side_v;
        let normal = self.side_u.cross(self.side_v);
        Interaction::rayless(position, rnd2, normal)
    }
    fn sample_towards(&self, _target: &Interaction, rnd2: (f32, f32)) -> Interaction {
        self.sample(rnd2)
    }
    fn area(&self) -> f32 {
        self.side_u.cross(self.side_v).norm()
    }
}

// impl ShapeSample for shape::QuadXY {
//     fn sample(&self, rnd2: (f32, f32)) -> Interaction {
//         let (u, v) = rnd2;
//         let pos = hcm::Point3::new(self.x_interval.lerp(u), self.y_interval.lerp(v), self.z);
//         let normal = hcm::Vec3::Z;
//         Interaction::rayless(pos, rnd2, normal)
//     }

//     fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction {
//         let mut temp = self.sample(rnd2);
//         let forward = target.pos - temp.pos;
//         temp.normal = forward.dot(temp.normal).signum() * temp.normal;
//         temp
//     }

//     fn area(&self) -> f32 {
//         self.x_interval.length() * self.y_interval.length()
//     }
// }

// impl ShapeSample for shape::QuadXZ {
//     fn sample(&self, rnd2: (f32, f32)) -> Interaction {
//         let (u, v) = rnd2;
//         let pos = hcm::Point3::new(self.x_interval.lerp(u), self.y, self.z_interval.lerp(v));
//         let normal = hcm::Vec3::Y;
//         Interaction::rayless(pos, rnd2, normal)
//     }

//     fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction {
//         let mut temp = self.sample(rnd2);
//         let forward = target.pos - temp.pos;
//         temp.normal = forward.dot(temp.normal).signum() * temp.normal;
//         temp
//     }

//     fn area(&self) -> f32 {
//         self.x_interval.length() * self.z_interval.length()
//     }
// }

// impl ShapeSample for shape::QuadYZ {
//     fn sample(&self, rnd2: (f32, f32)) -> Interaction {
//         let (u, v) = rnd2;
//         let pos = hcm::Point3::new(self.x, self.y_interval.lerp(u), self.z_interval.lerp(v));
//         let normal = hcm::Vec3::X;
//         Interaction::rayless(pos, rnd2, normal)
//     }

//     fn sample_towards(&self, target: &Interaction, rnd2: (f32, f32)) -> Interaction {
//         let mut temp = self.sample(rnd2);
//         let forward = target.pos - temp.pos;
//         temp.normal = forward.dot(temp.normal).signum() * temp.normal;
//         temp
//     }

//     fn area(&self) -> f32 {
//         self.z_interval.length() * self.y_interval.length()
//     }
// }
