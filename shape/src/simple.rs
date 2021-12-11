use geometry::bvh::BBox;
use math::float::{self, Interval};
use math::hcm::{Point3, Vec3};
use std::f32::consts::{FRAC_1_PI, PI};

use crate::{Interaction, Shape};
use geometry::ray::Ray;

#[derive(Debug, Clone)]
pub struct Sphere {
    center: Point3,
    radius: f32,
}

impl Sphere {
    pub fn new(center: Point3, radius: f32) -> Sphere {
        Sphere { center, radius }
    }
    pub fn from_raw(center: (f32, f32, f32), radius: f32) -> Sphere {
        let (x, y, z) = center;
        let has_nan = x.is_nan() || y.is_nan() || z.is_nan() || radius.is_nan();
        assert!(!has_nan);
        Self::new(Point3::new(x, y, z), radius)
    }
    pub fn center(&self) -> Point3 {
        self.center
    }
    pub fn radius(&self) -> f32 {
        self.radius
    }
}

#[derive(Debug, Clone)]
pub struct Disk {
    center: Point3,
    normal: Vec3,
    radial: Vec3,
    // radius: f32,
}

impl Disk {
    pub fn new(center: Point3, normal: Vec3, radial: Vec3) -> Self {
        let normal = normal.hat();
        assert!(radial.norm_squared().is_finite());
        assert!(radial.dot(normal).abs() < 1e-6);
        Self {
            center,
            normal,
            radial,
        }
    }
    /// Makes a new Disk with arbitrary rotation around the normal axis.
    pub fn new_anyspin(center: Point3, normal: Vec3, radius: f32) -> Self {
        let (radial, _) = math::hcm::make_coord_system(normal);
        Self::new(center, normal, radial * radius)
    }
    pub fn center(&self) -> Point3 {
        self.center
    }
    pub fn normal(&self) -> Vec3 {
        self.normal
    }
    pub fn radial(&self) -> Vec3 {
        self.radial
    }
}

#[derive(Clone)]
pub struct QuadXY {
    pub x_interval: Interval,
    pub y_interval: Interval,
    pub z: f32,
}

impl QuadXY {
    pub fn from_raw(x_interval: (f32, f32), y_interval: (f32, f32), z: f32) -> Self {
        let (x0, x1) = x_interval;
        let (y0, y1) = y_interval;
        Self {
            x_interval: Interval::new(x0, x1),
            y_interval: Interval::new(y0, y1),
            z,
        }
    }

    #[allow(dead_code)]
    pub fn new(x_interval: Interval, y_interval: Interval, z: f32) -> Self {
        Self {
            x_interval,
            y_interval,
            z,
        }
    }
}

#[derive(Clone)]
pub struct QuadXZ {
    pub x_interval: Interval,
    pub z_interval: Interval,
    pub y: f32,
}

impl QuadXZ {
    pub fn from_raw(x_interval: (f32, f32), z_interval: (f32, f32), y: f32) -> Self {
        let (x0, x1) = x_interval;
        let (z0, z1) = z_interval;
        Self {
            x_interval: Interval::new(x0, x1),
            z_interval: Interval::new(z0, z1),
            y,
        }
    }
}

#[derive(Clone)]
pub struct QuadYZ {
    pub y_interval: Interval,
    pub z_interval: Interval,
    pub x: f32,
}

impl QuadYZ {
    pub fn from_raw(y_interval: (f32, f32), z_interval: (f32, f32), x: f32) -> Self {
        let (y0, y1) = y_interval;
        let (z0, z1) = z_interval;
        Self {
            y_interval: Interval::new(y0, y1),
            z_interval: Interval::new(z0, z1),
            x,
        }
    }
}

#[derive(Debug)]
pub struct Cuboid {
    min: Point3,
    max: Point3,
}

impl Cuboid {
    pub fn from_points(p0: Point3, p1: Point3) -> Self {
        let (xmin, xmax) = float::min_max(p0.x, p1.x);
        let (ymin, ymax) = float::min_max(p0.y, p1.y);
        let (zmin, zmax) = float::min_max(p0.z, p1.z);
        Self {
            min: Point3::new(xmin, ymin, zmin),
            max: Point3::new(xmax, ymax, zmax),
        }
    }
}

#[derive(Clone, Copy)]
pub struct IsolatedTriangle {
    pub p0: Point3,
    pub p1: Point3,
    pub p2: Point3,
}

impl IsolatedTriangle {
    pub fn new(p0: Point3, p1: Point3, p2: Point3) -> Self {
        Self { p0, p1, p2 }
    }
}

// Implementation of the `Shape` trait for the shape implementations.

impl Shape for Sphere {
    fn summary(&self) -> String {
        format!("Sphere{{ {}, radius = {} }}", self.center, self.radius)
    }
    fn bbox(&self) -> BBox {
        let half_diagonal = Vec3::new(1.0, 1.0, 1.0) * self.radius;
        BBox::new(self.center - half_diagonal, self.center + half_diagonal)
    }
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        // r = o + td
        // sphere: (p-c)(p-c) = radius^2
        // (td + o - c)^2 = radius^2
        // t^2 d^2 + (o-c)^2 + 2t d * (o-c) = radius^2
        // delta = 4(d*(o-c))^2 - 4d^2((o-c)^2 - radius^2)
        let Ray { origin, dir, .. } = r.clone();

        let dir = dir.hat();

        let f = origin - self.center; // vector connecting the sphere center to ray origin.
        let q = f.dot(dir) * dir; // r.o + q gives the point closest to sphere center.
        let delta = self.radius * self.radius - (f - q).norm_squared();
        let (t_low, t_high) = if delta < 0.0 {
            return None;
        } else {
            let c = f.norm_squared() - self.radius * self.radius;
            let neg_b = -dir.dot(f);
            let t0 = neg_b + neg_b.signum() * delta.sqrt();
            let t1 = c / t0;
            if t0 < t1 {
                (t0, t1)
            } else {
                (t1, t0)
            }
        };
        let _is_from_inside = t_low * t_high < 0.0;
        // Keeps only the roots that are within [0, r.t_max).
        let truncated_roots = (r.truncated_t(t_low), r.truncated_t(t_high));
        let best_t: Option<f32> = match truncated_roots {
            (None, None) => None,
            (None, Some(high)) => Some(high),
            (Some(low), _) => Some(low), // It doesn't matter if the greater one is valid or not.
        };
        // print!("{:?}", best_t);
        let ray_t = best_t?;

        let pos = origin + dir * ray_t;
        let normal = (pos - self.center).hat();
        // When ray intersects from inside, the reflected ray should be spawn from inside.
        let pos = self.center + normal * self.radius * 1.00001;

        // Computes UV coordinate of the pos on the sphere.
        let theta = normal.y.acos();
        let phi = normal.z.atan2(normal.x) + PI;
        let uv = (phi / (2.0 * PI), theta / PI);

        // For an intersection at (x, y, z), the projection upon xy-plane is (x, y),
        // or in complex number: x + yi.
        // multiply by i (rotates 90-degrees from x to y) equals xi - y, so dpdu is (-y, x).
        // In case both x and y are zero, use (1, 0, 0).
        let dpdu = Vec3::new(-normal.y, normal.x, 0.0)
            .try_hat()
            .unwrap_or(Vec3::xbase());

        assert!(
            pos.distance_to(self.center) >= self.radius,
            "{} >= {}",
            pos.distance_to(self.center),
            self.radius
        );

        Some(Interaction::new(pos, ray_t, uv, normal, -r.dir).with_dpdu(dpdu))
    }
    fn occludes(&self, r: &Ray) -> bool {
        // r = o + td
        // sphere: (p-c)(p-c) = radius^2
        // (td + o - c)^2 = radius^2
        // t^2 d^2 + (o-c)^2 + 2t d * (o-c) = radius^2
        // delta = 4(d*(o-c))^2 - 4d^2((o-c)^2 - radius^2)
        let Ray { origin, dir, .. } = r.clone();

        let dir = dir.hat();

        let f = origin - self.center; // vector connecting the sphere center to ray origin.
        let q = f.dot(dir) * dir; // r.o + q gives the point closest to sphere center.
        let delta = self.radius.powi(2) - (f - q).norm_squared();
        let (t_low, t_high) = if delta < 0.0 {
            return false;
        } else {
            let c = f.norm_squared() - self.radius.powi(2);
            let neg_b = -dir.dot(f);
            let t0 = neg_b + neg_b.signum() * delta.sqrt();
            let t1 = c / t0;
            (t0, t1)
        };
        // Keeps only the roots that are within [0, r.t_max).
        let (root1, root2) = (r.truncated_t(t_low), r.truncated_t(t_high));
        root1.is_some() || root2.is_some()
    }
}

impl Shape for Disk {
    fn summary(&self) -> String {
        format!(
            "Disk{{ {}, normal = {}, radius = {}}}",
            self.center, self.normal, self.radial
        )
    }
    fn bbox(&self) -> BBox {
        let (v1, v2) = math::hcm::make_coord_system(self.normal);
        let (v1, v2) = (v1 * self.radial.norm(), v2 * self.radial.norm());
        geometry::bvh::union(
            BBox::new(self.center + v1 + v2, self.center + v1 - v2),
            BBox::new(self.center - v1 - v2, self.center - v1 + v2),
        )
    }
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        // Ray:    r = o + td
        // Plane: (p-c) dot n = 0
        //        (o + td - c) dot n = (o-c) dot n + t d dot n = 0
        //        (c - o) dot n = t * d dot n
        let t = (self.center - r.origin).dot(self.normal) / r.dir.dot(self.normal);
        let isect_point = r.position_at(t);
        (isect_point.squared_distance_to(self.center) <= self.radial.norm_squared()).then(|| {
            let cp = isect_point - self.center;
            // Removes the component from cp parallel to the normal.
            let cp = cp - cp.dot(self.normal) * self.normal;
            assert!(cp.dot(self.normal).abs() < 1e-6);
            let normal = self.normal * self.normal.dot(-r.dir).signum();
            let tangent = (normal.cross(cp)).hat();
            let u = self.radial.cross(cp).dot(normal).atan2(self.radial.dot(cp));
            let u = (u * FRAC_1_PI + 1.0).fract();
            let v = cp.norm() / self.radial.norm();
            Interaction::new(self.center + cp, t, (u, v), normal, -r.dir).with_dpdu(tangent)
        })
    }

    fn occludes(&self, r: &Ray) -> bool {
        let t = (self.center - r.origin).dot(self.normal) / r.dir.dot(self.normal);
        let isect_point = r.position_at(t);
        isect_point.squared_distance_to(self.center) <= self.radial.norm_squared()
    }
}

impl Shape for QuadXY {
    fn summary(&self) -> String {
        let (xmin, xmax) = self.x_interval.as_pair();
        let (ymin, ymax) = self.y_interval.as_pair();
        format!("QuadXY{{[{}, {}]x[{}, {}]}}", xmin, xmax, ymin, ymax)
    }
    fn bbox(&self) -> BBox {
        let (xmin, xmax) = self.x_interval.as_pair();
        let (ymin, ymax) = self.y_interval.as_pair();
        BBox::new(
            Point3::new(xmin, ymin, self.z - f32::EPSILON),
            Point3::new(xmax, ymax, self.z + f32::EPSILON),
        )
    }

    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        let t = (self.z - r.origin.z) / r.dir.z;
        let t = r.truncated_t(t)?;
        let Point3 { x, y, .. } = r.origin + t * r.dir;
        if self.x_interval.contains(x) && self.y_interval.contains(y) {
            let (xmin, _) = self.x_interval.as_pair();
            let (ymin, _) = self.y_interval.as_pair();
            let u = (x - xmin) / self.x_interval.length();
            let v = (y - ymin) / self.y_interval.length();

            let pos = Point3::new(x, y, self.z);
            let normal = Vec3::zbase() * -r.dir.z.signum();

            Some(Interaction::new(pos, t, (u, v), normal, -r.dir).with_dpdu(Vec3::xbase()))
        } else {
            None
        }
    }

    fn occludes(&self, r: &Ray) -> bool {
        let t = (self.z - r.origin.z) / r.dir.z;
        if let Some(t) = r.truncated_t(t) {
            let (x, y, _) = r.position_at(t).as_triple();
            self.x_interval.contains(x) && self.y_interval.contains(y)
        } else {
            false
        }
    }
}

impl Shape for QuadXZ {
    fn summary(&self) -> String {
        let (xmin, xmax) = self.x_interval.as_pair();
        let (zmin, zmax) = self.z_interval.as_pair();
        format!("QuadXZ{{[{}, {}]x[{}, {}]}}", xmin, xmax, zmin, zmax)
    }
    fn bbox(&self) -> BBox {
        let (xmin, xmax) = self.x_interval.as_pair();
        let (zmin, zmax) = self.z_interval.as_pair();
        BBox::new(
            Point3::new(xmin, self.y * float::ONE_MINUS_EPSILON, zmin),
            Point3::new(xmax, self.y * float::ONE_PLUS_EPSILON, zmax),
        )
    }
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        let t = (self.y - r.origin.y) / r.dir.y;
        let t = r.truncated_t(t)?;
        let Point3 { x, y: _, z } = r.position_at(t);
        if self.x_interval.contains(x) && self.z_interval.contains(z) {
            let u = (x - self.x_interval.min) / self.x_interval.length();
            let v = (z - self.z_interval.min) / self.z_interval.length();

            let pos = Point3::new(x, self.y, z);
            let normal = Vec3::ybase() * -r.dir.y.signum();

            Some(Interaction::new(pos, t, (u, v), normal, -r.dir).with_dpdu(Vec3::xbase()))
        } else {
            None
        }
    }
    fn occludes(&self, r: &Ray) -> bool {
        let t = (self.y - r.origin.y) / r.dir.y;
        if let Some(t) = r.truncated_t(t) {
            let (x, _, z) = r.position_at(t).as_triple();
            self.x_interval.contains(x) && self.z_interval.contains(z)
        } else {
            false
        }
    }
}

impl Shape for QuadYZ {
    fn summary(&self) -> String {
        let (zmin, zmax) = self.z_interval.as_pair();
        let (ymin, ymax) = self.y_interval.as_pair();
        format!("QuadXY{{[{}, {}]x[{}, {}]}}", ymin, ymax, zmin, zmax)
    }
    fn bbox(&self) -> BBox {
        let (ymin, ymax) = self.y_interval.as_pair();
        let (zmin, zmax) = self.z_interval.as_pair();

        BBox::new(
            Point3::new(self.x * float::ONE_MINUS_EPSILON, ymin, zmin),
            Point3::new(self.x * float::ONE_PLUS_EPSILON, ymax, zmax),
        )
    }
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        let t = (self.x - r.origin.x) / r.dir.x;
        let t = r.truncated_t(t)?;
        let Point3 { x: _, y, z } = r.position_at(t);
        if self.y_interval.contains(y) && self.z_interval.contains(z) {
            let u = (y - self.y_interval.min) / self.y_interval.length();
            let v = (z - self.z_interval.min) / self.z_interval.length();

            let pos = Point3::new(self.x, y, z);
            let normal = Vec3::xbase() * -r.dir.x.signum();

            Some(Interaction::new(pos, t, (u, v), normal, -r.dir).with_dpdu(Vec3::ybase()))
        } else {
            None
        }
    }
    fn occludes(&self, r: &Ray) -> bool {
        let t = (self.x - r.origin.x) / r.dir.x;
        if let Some(t) = r.truncated_t(t) {
            let (_, y, z) = r.position_at(t).as_triple();
            self.y_interval.contains(y) && self.z_interval.contains(z)
        } else {
            false
        }
    }
}

impl Shape for Cuboid {
    fn summary(&self) -> String {
        format!("Cuboid{{{} <-> {}}}", self.min, self.max)
    }
    fn bbox(&self) -> BBox {
        BBox::new(self.min, self.max)
    }

    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        #[derive(Debug, Clone, Copy)]
        struct HitInfo {
            t: f32,
            bound: f32,
            axis: usize,
        }
        impl HitInfo {
            fn new(t: f32, bound: f32, axis: usize) -> Self {
                HitInfo { t, bound, axis }
            }
        }

        let mut hit_min = HitInfo::new(0.0f32, f32::INFINITY, 0);
        let mut hit_max = HitInfo::new(r.t_max, -f32::INFINITY, 0);

        // println!("ray = {}, cuboid = {:?}", r, self);
        for axis in 0..3 {
            let inv_dir = 1.0 / r.dir[axis];
            let mut t0 = (self.min[axis] - r.origin[axis]) * inv_dir;
            let mut t1 = (self.max[axis] - r.origin[axis]) * inv_dir;
            let mut hit_0 = HitInfo::new(t0, self.min[axis], axis);
            let mut hit_1 = HitInfo::new(t1, self.max[axis], axis);
            // println!("axis = {}, hit0 = {:?}, hit1 = {:?}", axis, hit_0, hit_1);
            if t0 > t1 {
                std::mem::swap(&mut hit_0, &mut hit_1);
                std::mem::swap(&mut t0, &mut t1);
            }
            std::mem::drop(t0);
            std::mem::drop(t1);
            // Shrinks [t_min, t_max] by intersecting it with [t0, t1].
            if t0 > hit_min.t {
                hit_min = hit_0;
            }
            if t1 < hit_max.t {
                hit_max = hit_1;
            }
            if hit_max.t < hit_min.t {
                return None;
            }
        }
        let t_interval = Interval::new(hit_min.t, hit_max.t);
        let HitInfo {
            t,
            bound: axis_value,
            axis,
        } = if t_interval.contains(0.0) {
            hit_max
        } else {
            hit_min
        };
        if axis_value.is_infinite() {
            return None;
        }
        assert!(
            !axis_value.is_infinite(),
            "hmin {:?} hmax {:?}",
            &hit_min,
            &hit_max
        );
        let mut hit_pos = r.position_at(t);
        hit_pos[axis] = axis_value;
        let mut normal = Vec3::zero();
        normal[axis] = r.dir[axis].signum() * -1.0;
        let mut tangent = Vec3::zero();
        let tangent_axis = (axis + 1) % 3;
        tangent[tangent_axis] = 1.0;
        Some(Interaction::new(hit_pos, t, (0.5, 0.5), normal, -r.dir).with_dpdu(tangent))
    }
    fn occludes(&self, r: &Ray) -> bool {
        // Reuses the method from BBox which returns a bool.
        self.bbox().intersect(r)
    }
}

impl Shape for IsolatedTriangle {
    fn summary(&self) -> String {
        format!("Triangle boxed by {}", self.bbox())
    }
    fn bbox(&self) -> BBox {
        BBox::new(self.p0, self.p1).union(self.p2)
    }
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        intersect_triangle(self.p0, self.p1, self.p2, r)
    }
    fn occludes(&self, r: &Ray) -> bool {
        intersect_triangle_pred(self.p0, self.p1, self.p2, r)
    }
}

#[rustfmt::skip]
/// Computes ray-triangle intersection.  The `uv` property of the resulting `Interaction` (if any)
/// is computed such that `p = p0 + u*(p1-p0) + v*(p2-p0)` where `p` is the point of intersection.
pub fn intersect_triangle(p0: Point3, p1: Point3, p2: Point3, r: &Ray) -> Option<Interaction> {
    let normal = (p0 - p1).cross(p2 - p1);
    // TODO(zixun): remove triangles (index triplets) from plymeshes that have zero area.
    if normal.is_zero() {
        // Degenerate triangle.
        return None;
    }
    let normal = normal.hat();
    let normal = normal.dot(-r.dir).signum() * normal;
    assert!(normal.dot(r.dir) <= 0.0);
    // The equation for the plane of the triangle would be:
    // (p - p0).dot(normal) = 0. Plugging in the ray equation $p = o + td$, we have
    // (o + td - p0).dot(normal) = 0  =>  t*dot(d, normal) = dot(p0-o, normal)
    let t = normal.dot(p0 - r.origin) / normal.dot(r.dir);
    let t = r.truncated_t(t)?;
    let p = r.position_at(t);
    // Computes the barycentric coordinates of p with regard to the triangle.
    let b2 = (p - p0).cross(p - p1).dot(normal);
    let b0 = (p - p1).cross(p - p2).dot(normal);
    let b1 = (p - p2).cross(p - p0).dot(normal);
    if b0.is_nan() || b1.is_nan() || b2.is_nan() {
        panic!("some Nans: {}, {}, {}", b0, b1, b2);
    }
    let (b0, b1, b2) = match (b0 > 0.0, b1 > 0.0, b2 > 0.0) {
        (true, true, true) | (false, false, false) => {
            let total_area = b0 + b1 + b2;
            (b0 / total_area, b1 / total_area, b2 / total_area)
        }
        _ => return None,
    };
    let hit_pos = float::barycentric_lerp((p0, p1, p2), (b0, b1, b2));
    if hit_pos.has_nan() {
        // println!("normal = {:.3}, p0 - r.origin = {:.3}, r.dir = {:.3}", normal, p0 - r.origin, r.dir);
        // println!("t = {} / {} = {}. p = {}", normal.dot(p0 - r.origin), normal.dot(r.dir), t, p);
        // println!("p-p0 x p-p1 = {:.3}", (p-p0).cross(p-p1));
        // println!("p-p1 x p-p2 = {:.3}", (p-p1).cross(p-p2));
        // println!("p-p2 x p-p0 = {:.3}", (p-p2).cross(p-p0));
        return None;
    }
    // Now an intersection is truly found.
    // hit_pos = p0 * b0 +            p1 * b1 + p2 * b2 
    //         = p0 * (1 - b1 - b2) + p1 * b1 + p2 * b2
    //         = p0 + (p1 - p0) * b1 + (p2 - p0) * b2
    Some(Interaction::new(hit_pos, t, (b1, b2), normal, -r.dir))
}

pub fn intersect_triangle_pred(p0: Point3, p1: Point3, p2: Point3, r: &Ray) -> bool {
    let normal = (p0 - p1).cross(p2 - p1);
    if normal.is_zero() {
        return false;
    }
    let normal = normal.hat();
    let t = normal.dot(p0 - r.origin) / normal.dot(r.dir);
    if let Some(t) = r.truncated_t(t) {
        let p = r.position_at(t);
        let b0 = (p - p0).cross(p - p1).dot(normal);
        let b1 = (p - p1).cross(p - p2).dot(normal);
        let b2 = (p - p2).cross(p - p0).dot(normal);
        let has_nans = b0.is_nan() || b1.is_nan() || b2.is_nan();
        assert!(!has_nans);
        match (b0 > 0.0, b1 > 0.0, b2 > 0.0) {
            (true, true, true) | (false, false, false) => true,
            _ => false,
        }
    } else {
        false
    }
}
