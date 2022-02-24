use geometry::bvh::BBox;
use math::float::{self, Inside, Interval};
use math::hcm::{point3, vec3, Point3, Vec3};
use std::f32::consts::{FRAC_1_PI, PI};

use crate::{Interaction, Shape};
use geometry::ray::Ray;

#[derive(Debug, Clone, Copy)]
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

#[derive(Debug, Clone, Copy)]
pub struct ParallelQuad {
    pub origin: Point3,
    pub side_u: Vec3,
    pub side_v: Vec3,
}

impl ParallelQuad {
    pub fn new_xy(x_range: (f32, f32), y_range: (f32, f32), z: f32) -> Self {
        let (x0, x1) = x_range;
        let (y0, y1) = y_range;
        Self {
            origin: point3(x0, y0, z),
            side_u: vec3(x1 - x0, 0.0, 0.0),
            side_v: vec3(0.0, y1 - y0, 0.0),
        }
    }
    pub fn new_xz(x_range: (f32, f32), y: f32, z_range: (f32, f32)) -> Self {
        let (x0, x1) = x_range;
        let (z0, z1) = z_range;
        Self {
            origin: point3(x0, y, z0),
            side_u: vec3(x1 - x0, 0.0, 0.0),
            side_v: vec3(0.0, 0.0, z1 - z0),
        }
    }
    pub fn new_yz(x: f32, y_range: (f32, f32), z_range: (f32, f32)) -> Self {
        let (z0, z1) = z_range;
        let (y0, y1) = y_range;
        Self {
            origin: point3(x, y0, z0),
            side_u: vec3(0.0, 0.0, z1 - z0),
            side_v: vec3(0.0, y1 - y0, 0.0),
        }
    }
}

impl Shape for ParallelQuad {
    fn bbox(&self) -> BBox {
        let bu = BBox::new(self.origin, self.origin + self.side_u);
        let bv = BBox::new(
            self.origin + self.side_v,
            self.origin + self.side_u + self.side_v,
        );
        geometry::bvh::union(bu, bv)
    }
    fn summary(&self) -> String {
        format!(
            "Parallelogram({:.3} + u{:.3} + v{:.3}",
            self.origin, self.side_u, self.side_v
        )
    }
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        let normal = self.side_u.cross(self.side_v).facing(r.dir);
        // Let p be the point on the plane containing the quad, then
        // (p - origin) dot normal = 0
        // With p = r.o + r.d * t, (r.o + t * r.d - self.origin) dot normal = 0
        // (r.o - self.origin) dot normal + t * r.d dot normal = 0
        let t = (self.origin - r.origin).dot(normal) / r.dir.dot(normal);
        let t = r.truncated_t(t)?;
        let coarse_hit = r.position_at(t);
        // p - o = au + bv = d
        // cross(a, a)*u + cross(a, b)*v = cross(a, d) where cross(a, a) = 0.
        //     cross(a, d)           cross(b, d)
        // v = -----------,      u = -----------
        //     cross(a, b)           cross(b, a)
        let (a, b, d) = (self.side_u, self.side_v, coarse_hit - self.origin);

        let v = a.cross(d).norm() / a.cross(b).norm();
        let u = b.cross(d).norm() / b.cross(a).norm();
        (v.inside((0.0, 1.0)) && u.inside((0.0, 1.0))).then(|| {
            let accurate_hit = self.origin + u * a + b * v;
            assert!(
                accurate_hit.distance_to(coarse_hit) < 1e-3,
                "hit pos = {:.3}(coarse) / {:.3}(accurate), self = {:?}, uv = {:?}",
                coarse_hit,
                accurate_hit,
                self,
                (u, v)
            );
            Interaction::new(accurate_hit, t, (u, v), normal.hat(), -r.dir).with_dpdu(self.side_u)
        })
    }
    fn occludes(&self, r: &Ray) -> bool {
        let normal = self.side_u.cross(self.side_v);
        let t = r.dir.dot(normal) / (self.origin - r.origin).dot(normal);
        let t = match r.truncated_t(t) {
            None => return false,
            Some(t) => t,
        };
        let coarse_hit = r.position_at(t);
        let (a, b, d) = (self.side_u, self.side_v, coarse_hit - self.origin);

        let v = a.cross(d).norm() / a.cross(b).norm();
        let u = b.cross(d).norm() / b.cross(a).norm();
        v.inside((0.0, 1.0)) && u.inside((0.0, 1.0))
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

        let f = r.origin - self.center; // vector connecting the sphere center to ray origin.
        let a = r.dir.norm_squared();
        let b_prime = -f.dot(r.dir);
        let delta = self.radius * self.radius - (f + b_prime / a * r.dir).norm_squared();
        let (t_low, t_high) = if delta < 0.0 {
            return None;
        } else {
            let c = f.norm_squared() - self.radius * self.radius;
            let q = b_prime + b_prime.signum() * (delta*a).sqrt();
            let (t0, t1) = (c / q, q / a);
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

        let pos = r.position_at(ray_t);
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
            .unwrap_or(Vec3::X);

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

        let f = r.origin - self.center; // vector connecting the sphere center to ray origin.
        let a = r.dir.norm_squared();
        let b_prime = -f.dot(r.dir);
        let delta = self.radius * self.radius - (f + b_prime / a * r.dir).norm_squared();
        let (t0, t1) = if delta < 0.0 {
            return false;
        } else {
            let c = f.norm_squared() - self.radius * self.radius;
            let q = b_prime + b_prime.signum() * (delta*a).sqrt();
            (c / q, q / a)
        };
        // Keeps only the roots that are within [0, r.t_max).
        let (root1, root2) = (r.truncated_t(t0), r.truncated_t(t1));
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
        let t = r.truncated_t(t)?;
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
        let mut normal = Vec3::ZERO;
        normal[axis] = r.dir[axis].signum() * -1.0;
        let mut tangent = Vec3::ZERO;
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
        intersect_triangle(self.p0, self.p1, self.p2, r).map(|i| i.with_dpdu(self.p1 - self.p0))
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
        eprintln!("some Nans: {}, {}, {}", b0, b1, b2);
        eprintln!("points = [{p0}, {p1}, {p2}], normal = {normal}");
        return None;
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
