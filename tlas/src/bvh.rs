use std::fmt::{Debug, Formatter, Result};
use std::sync::Arc;

use crate::instance::Instance;
use geometry::bvh::{self, BBox};
use geometry::ray::Ray;
use material::Material;
use shape::Interaction;

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
            bbox: instance.bbox(),
            content: BvhNodeContent::Leaf(instance),
        }
    }
    pub fn new_internal(c0: Box<BvhNode>, c1: Box<BvhNode>) -> BvhNode {
        BvhNode {
            bbox: bvh::union(c0.bbox, c1.bbox),
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
            BvhNodeContent::Leaf(inst) => assert!(self.bbox.encloses(inst.bbox())),
        }
        true
    }

    /// Shoots a ray toward the BVH and computes the closest hit interation.
    ///
    /// This method has a similar interface to the same method in `Shape` trait, but the `Shape`
    /// trait is not implemented on `BvhNode` on purpose.
    pub fn intersect(&self, ray: &mut Ray) -> Option<(Interaction, &Arc<dyn Material>)> {
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

    pub fn occludes(&self, ray: &Ray) -> bool {
        let mut stack = Vec::new();

        stack.reserve(64);
        stack.push(self);
        while let Some(node) = stack.pop() {
            if node.bbox.intersect(ray) {
                continue;
            }
            match &node.content {
                BvhNodeContent::Leaf(inst) if inst.occludes(ray) => return true,
                BvhNodeContent::Children([left, right]) => {
                    stack.push(left);
                    stack.push(right);
                }
                _ => (),
            }
        }
        return false;
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
            .map(|i| i.bbox())
            .fold(BBox::empty(), |b1, b2| bvh::union(b1, b2));
        let span = bbox_all.diag();
        let max_span_axis = span.max_dimension();
        let split_plane = bbox_all.midpoint()[max_span_axis];
        let (mut left, mut right): (Vec<_>, Vec<_>) = instances
            .into_iter()
            .partition(|inst| inst.bbox().midpoint()[max_span_axis] < split_plane);

        if left.is_empty() {
            for _ in 0..num_all / 2 {
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
