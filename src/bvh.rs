use std::{
    fmt::{Debug, Display, Formatter, Result},
    sync::Arc
};

use crate::{
    hcm::{Point3},
    instance::Instance,
    ray::Ray,
    shape::Interaction,
    material::Material,
    float::min_max
};

#[derive(Debug, Clone, Copy)]
pub struct BBox {
    min: Point3,
    max: Point3,
}

impl BBox {
    pub fn empty() -> BBox {
        BBox {
            min: Point3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            max: Point3::new(-f32::INFINITY, -f32::INFINITY, -f32::INFINITY),
        }
    }
    pub fn new(p0: Point3, p1: Point3) -> BBox {
        let (xmin, xmax) = min_max(p0.x, p1.x);
        let (ymin, ymax) = min_max(p0.y, p1.y);
        let (zmin, zmax) = min_max(p0.z, p1.z);
        BBox {
            min: Point3::new(xmin, ymin, zmin),
            max: Point3::new(xmax, ymax, zmax),
        }
    }

    pub fn union(self, p: Point3) -> BBox {
        let mut result = self;
        for i in 0..3 {
            result.min[i] = self.min[i].min(p[i]);
            result.max[i] = self.max[i].max(p[i]);
        }
        result
    }

    pub fn midpoint(self) -> Point3 {
        (self.max - self.min) * 0.5 + self.min
    }

    pub fn intersect(&self, r: &Ray) -> bool {
        let (mut t_min, mut t_max) = (0.0f32, r.t_max);
        for axis in 0..3 {
            let inv_dir = 1.0 / r.dir[axis];
            let t0 = (self.min[axis] - r.origin[axis]) * inv_dir;
            let t1 = (self.max[axis] - r.origin[axis]) * inv_dir;
            let (t0, t1) = min_max(t0, t1);
            // Shrinks [t_min, t_max] by intersecting it with [t0, t1].
            t_min = t_min.max(t0);
            t_max = t_max.min(t1);
            if t_max < t_min {
                return false;
            }
        }
        return true;
    }

    pub fn encloses(&self, other: Self) -> bool {
        for axis in 0..3 {
            if self.min[axis] > other.min[axis] {
                return false;
            }
            if self.max[axis] < other.max[axis] {
                return false;
            }
        }
        true
    }
    #[allow(dead_code)]
    pub fn contains(&self, p: Point3) -> bool {
        for axis in 0..3 {
            if self.min[axis] > p[axis] {
                return false;
            }
            if self.max[axis] < p[axis] {
                return false;
            }
        }
        true
    }
}

impl Display for BBox {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "box[{} -> {}]", self.min, self.max)
    }
}

pub fn union(b0: BBox, b1: BBox) -> BBox {
    b0.union(b1.min).union(b1.max)
}

#[derive(Debug)]
enum BvhNodeContent {
    Children([Box<BvhNode>; 2]),
    Leaf(Box<Instance>),
}
pub struct BvhNode {
    bbox: BBox,
    content: BvhNodeContent,
}

impl Debug for BvhNode {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let content_string = match &self.content {
            BvhNodeContent::Children([l, r]) => {
                format!("\n{:?}\n{:?}", l, r)
            }
            BvhNodeContent::Leaf(_) => "leaf".to_string(),
        };
        write!(
            f,
            "{{ bbox = {} {}\n }}",
            self.bbox,
            content_string.replace("\n", "\n  ")
        )
    }
}

impl BvhNode {
    pub fn new_leaf(instance: Box<Instance>) -> BvhNode {
        BvhNode {
            bbox: instance.shape.bbox(),
            content: BvhNodeContent::Leaf(instance),
        }
    }
    pub fn new_internal(c0: Box<BvhNode>, c1: Box<BvhNode>) -> BvhNode {
        BvhNode {
            bbox: union(c0.bbox, c1.bbox),
            content: BvhNodeContent::Children([c0, c1]),
        }
    }
    pub fn height(&self) -> u32 {
        match &self.content {
            BvhNodeContent::Children([left, right]) => left.height().max(right.height()) + 1,
            BvhNodeContent::Leaf(_) => 1,
        }
    }
    pub fn geometric_sound(&self) -> bool {
        match &self.content {
            BvhNodeContent::Children([left, right]) => {
                assert!(self.bbox.encloses(left.bbox));
                assert!(self.bbox.encloses(right.bbox));
            }
            BvhNodeContent::Leaf(inst) => assert!(self.bbox.encloses(inst.shape.bbox())),
        }
        true
    }

    /// Shoots a ray toward the BVH and computes the closest hit interation.
    ///
    /// This method has a similar interface to the same method in `Shape` trait, but the `Shape`
    /// trait is not implemented on `BvhNode` on purpose.
    pub fn intersect(&self, ray: & mut Ray) -> Option<(Interaction, &Arc<dyn Material>)> {
        if self.bbox.intersect(ray) == false {
            return None;
        }
        match &self.content {
            BvhNodeContent::Leaf(inst) => inst.intersect(ray),
            BvhNodeContent::Children([left, right]) => {
                let left_isect = left.intersect(ray);
                if let Some((isect, _)) = left_isect {
                    ray.set_extent(isect.ray_t);
                }
                let right_isect = right.intersect(ray);
                match (left_isect, right_isect) {
                    (None, None) => None,
                    (Some(l), None) => Some(l),
                    (None, Some(r)) => Some(r),
                    (Some(l), Some(r)) => {
                        if l.0.ray_t < r.0.ray_t {
                            Some(l)
                        } else {
                            Some(r)
                        }
                    }
                }
            }
        }
    }
}

pub fn build_bvh(mut instances: Vec<Box<Instance>>) -> Box<BvhNode> {
    assert!(!instances.is_empty(), "empty instances");

    if instances.len() == 1 {
        let single_instance = instances.pop().unwrap();
        Box::new(BvhNode::new_leaf(single_instance))
    } else {
        let num_all = instances.len();
        let bbox_all = instances
            .iter()
            .map(|i| i.shape.bbox())
            .fold(BBox::empty(), |b1, b2| union(b1, b2));
        let span = bbox_all.max - bbox_all.min;
        let max_span_axis = span.max_dimension();
        let split_plane = bbox_all.midpoint()[max_span_axis];
        let (mut left, mut right): (Vec<_>, Vec<_>) = instances
            .into_iter()
            .partition(|inst| inst.shape.bbox().midpoint()[max_span_axis] < split_plane);
            
        if left.is_empty() {
            for _ in 0..num_all/2 {
                left.push(right.pop().unwrap());
            }
        } else if right.is_empty() {
            for _ in 0..num_all / 2 {
                right.push(left.pop().unwrap());
            }
        }

        assert!(left.len() < num_all, "{:?}", left);
        assert!(right.len() < num_all, "{:?}", right);

        let left_bvh = build_bvh(left);
        let right_bvh = build_bvh(right);
        Box::new(BvhNode::new_internal(left_bvh, right_bvh))
    }
}
