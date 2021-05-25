use std::{
    f32::consts::PI,
    fmt::{Display, Formatter, Result},
    ops::Range,
};

use partition::partition;

use crate::{assert_le, float};
use crate::hcm::{Point3, Vec3};
use crate::ray::Ray;
use crate::{bvh, bvh::BBox, float::Interval};

#[derive(Debug, Clone, Copy)]
pub struct Interaction {
    pub pos: Point3,
    pub ray_t: f32,
    pub uv: (f32, f32),
    pub normal: Vec3,
    // pub _albedo: Vec3,
    // pub dpdu: Vec3,
    // pub dpdv: Vec3
}

impl Interaction {
    pub fn new(pos: Point3, ray_t: f32, uv: (f32, f32), normal: Vec3) -> Interaction {
        Interaction {
            pos,
            ray_t,
            uv,
            normal,
        }
    }
}

impl Display for Interaction {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let (u, v) = self.uv;
        write!(
            f,
            "pos = {}, t = {:.2}, uv = ({:.2}, {:.2}), normal = {}",
            self.pos, self.ray_t, u, v, self.normal
        )
    }
}

/// Represents the characteristics of a shape: has a bounding box, and can interact with ray.
pub trait Shape: Send + Sync {
    fn intersect(&self, r: &Ray) -> Option<Interaction>;
    fn bbox(&self) -> BBox;
}

// Definition of various shapes and their building methods.
// ------------------------------------------------------------------------------------------------

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
        let has_nan = x.is_nan() || y.is_nan()|| z.is_nan() || radius.is_nan();
        assert!(!has_nan);
        Self::new(Point3::new(x, y, z), radius)
    }
}

pub struct QuadXY {
    x_interval: Interval,
    y_interval: Interval,
    z: f32,
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

pub struct QuadXZ {
    x_interval: Interval,
    z_interval: Interval,
    y: f32,
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

pub struct QuadYZ {
    y_interval: Interval,
    z_interval: Interval,
    x: f32,
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

enum IsoBvhNodeContent {
    Children([Box<IsoBvhNode>; 2]),
    Leaf(Range<usize>),
}

struct IsoBvhNode {
    bbox: BBox,
    content: IsoBvhNodeContent,
}

/// A collection of `Shape`s of same type and organized with a bounding-volume hierarchy.
pub struct IsoBlas<T>
where
    T: Shape,
{
    shapes: Vec<T>,
    bbox: BBox,
    bvh_root: Option<IsoBvhNode>,
}

use IsoBvhNodeContent::Children;
use IsoBvhNodeContent::Leaf;

impl<T> IsoBlas<T>
where
    T: Shape,
{
    fn new_with_only_shapes(shapes: Vec<T>) -> Self {
        IsoBlas {
            shapes,
            bbox: BBox::empty(),
            bvh_root: None,
        }
    }

    #[allow(dead_code)]
    pub fn build(shapes: Vec<T>) -> Self {
        let mut raw = Self::new_with_only_shapes(shapes);
        let tree = raw.recursive_build(0..raw.shapes.len());
        raw.bbox = tree.bbox;
        raw.bvh_root = Some(tree);
        raw
    }

    fn recursive_build(&mut self, range: Range<usize>) -> IsoBvhNode {
        if range.len() <= 4 {
            return Self::new_leaf(&self, range);
        }

        let mut bboxes: Vec<_> = self.shapes[range.clone()]
            .iter()
            .map(|s| s.bbox())
            .collect();
        let centroid_bbox = bboxes
            .iter()
            .fold(BBox::empty(), |sum, b| sum.union(b.midpoint()));
        assert!(centroid_bbox.area() > 0.0);
        let split_axis = centroid_bbox.diag().max_dimension();

        // Computes the plane "axis = pivot_value" that will be used to partition the shapes.
        // ----------------------------------------------------------------------------------

        // Sorts the bounding boxes according to coordinate value on the computed axis.
        bboxes.sort_by(|b0, b1| {
            let axis_pos_0 = b0.midpoint()[split_axis];
            let axis_pos_1 = b1.midpoint()[split_axis];
            axis_pos_0.partial_cmp(&axis_pos_1).unwrap()
        });

        let bbox_area_sum: f32 = bboxes.iter().map(|b| b.area()).sum();
        let surface_area_heuristic_pivot = bbox_area_sum * 0.5;

        let mut partial_sum = 0.0;
        let mut split_index = 0;
        for i in 0..bboxes.len() {
            partial_sum += bboxes[i].area();
            if partial_sum >= surface_area_heuristic_pivot {
                split_index = i;
                break;
            }
        }

        let pivot_value = bboxes[split_index].midpoint()[split_axis];

        // Partitions the set of shapes w.r.t. their bounding box midpoint coordinate on the split axis.
        let (left, right) = partition(&mut self.shapes[range.clone()], |s| {
            s.bbox().midpoint()[split_axis] < pivot_value
        });
        let mid_point = left.len() + range.start;
        // This assertion isn't necessary: assert_eq!(mid_point - range.start, left.len());
        assert_eq!(
            range.end - mid_point,
            right.len(),
            "left = {:?}, right = {:?}, whole_range = {:?}, mid_point = {}",
            left.len(),
            right.len(),
            range,
            mid_point
        );

        let left_child = self.recursive_build(range.start..mid_point);
        let right_child = self.recursive_build(mid_point..range.end);

        self.new_internal(Box::new(left_child), Box::new(right_child))
    }

    fn intersect_tree(&self, tree: &IsoBvhNode, r: &Ray) -> Option<Interaction> {
        match &tree.content {
            Leaf(range) => {
                let mut hit: Option<Interaction> = None;
                // Ranges are not `Copy`: https://github.com/rust-lang/rust/pull/27186
                for shape in self.shapes[range.clone()].iter() {
                    match (shape.intersect(r), hit) {
                        (None, _) => (), // Doesn't update `hit` if no new interaction
                        (Some(isect), None) => hit = Some(isect),
                        (Some(new_isect), Some(old_isect)) => {
                            if new_isect.ray_t < old_isect.ray_t {
                                hit = Some(old_isect);
                            }
                        }
                    }
                }
                hit
            }
            Children([left, right]) => {
                let mut ray = r.clone();
                let left_isect = self.intersect_tree(&**left, &ray);
                if let Some(isect) = left_isect {
                    ray.set_extent(isect.ray_t);
                }
                let right_isect = self.intersect_tree(&**right, &ray);
                match (left_isect, right_isect) {
                    (None, None) => None,
                    (Some(l), None) => Some(l),
                    (None, Some(r)) => Some(r),
                    (Some(l), Some(r)) => {
                        if l.ray_t < r.ray_t {
                            Some(l)
                        } else {
                            Some(r)
                        }
                    }
                }
            }
        }
    }

    fn new_leaf(&self, range: Range<usize>) -> IsoBvhNode {
        assert_le!(range.end, self.shapes.len());
        let bbox = self.shapes[range.clone()]
            .iter()
            .fold(BBox::empty(), |b, shape| bvh::union(b, shape.bbox()));
        IsoBvhNode {
            bbox,
            content: Leaf(range),
        }
    }

    fn new_internal(&self, c0: Box<IsoBvhNode>, c1: Box<IsoBvhNode>) -> IsoBvhNode {
        let bbox = bvh::union(c0.bbox, c1.bbox);
        IsoBvhNode {
            bbox,
            content: Children([c0, c1]),
        }
    }
}

// Implementation of shapes with regard to trait `Shape`.
// ------------------------------------------------------------------------------------------------

impl Shape for Sphere {
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
        match best_t {
            None => None,
            Some(ray_t) => {
                let pos = origin + dir * ray_t;
                let normal = (pos - self.center).hat();
                // When ray intersects from inside, the reflected ray should be spawn from inside.
                let pos = self.center + normal * self.radius * 1.00001;

                // Computes UV coordinate of the pos on the sphere.
                let theta = normal.y.acos();
                let phi = normal.z.atan2(normal.x) + PI;
                let uv = (phi / (2.0 * PI), theta / PI);

                assert!(
                    pos.distance_to(self.center) >= self.radius,
                    "{} >= {}",
                    pos.distance_to(self.center),
                    self.radius
                );
                Some(Interaction::new(pos, ray_t, uv, normal))
            }
        }
    }
}

impl Shape for QuadXY {
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

            Some(Interaction::new(pos, t, (u, v), normal))
        } else {
            None
        }
    }
}

impl Shape for QuadXZ {
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

            Some(Interaction::new(pos, t, (u, v), normal))
        } else {
            None
        }
    }
}

impl Shape for QuadYZ {
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

            Some(Interaction::new(pos, t, (u, v), normal))
        } else {
            None
        }
    }
}

impl Shape for Cuboid {
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
        Some(Interaction::new(hit_pos, t, (0.5, 0.5), normal))
    }
}

impl<T> Shape for IsoBlas<T>
where
    T: Shape,
{
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        let tree = &self.bvh_root.as_ref()?;
        self.intersect_tree(&tree, r)
    }

    fn bbox(&self) -> BBox {
        self.bbox
    }
}
