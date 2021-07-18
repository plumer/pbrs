use std::ops::Range;
use crate::geometry::bvh::{BBox, self};
use crate::geometry::shape::*;
use crate::geometry::hcm::*;
use crate::geometry::float;
use partition::partition;

enum IsoBvhNodeContent {
    Children([Box<IsoBvhNode>; 2]),
    Leaf(Range<usize>),
}

struct IsoBvhNode {
    bbox: BBox,
    content: IsoBvhNodeContent,
}

impl IsoBvhNode {
    fn height(&self) -> usize {
        match &self.content {
            Children([left, right]) => std::cmp::max(left.height(), right.height()) + 1,
            Leaf(_) => 1,
        }
    }
    fn count(&self) -> usize {
        match &self.content {
            Children([left, right]) => left.count() + right.count() + 1,
            Leaf(_) => 1,
        }
    }
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
        let num_shapes = raw.shapes.len();
        // let tree = raw.recursive_build(0..raw.shapes.len());
        let tree = recursive_build(&mut raw.shapes, 0..num_shapes, |s: &T| s.bbox());
        raw.bbox = tree.bbox;
        raw.bvh_root = Some(tree);
        raw
    }
}

struct Triangle {
    indices: (usize, usize, usize),
    bbox: BBox,
}
impl Triangle {
    fn new(indices: (usize, usize, usize), bbox: BBox) -> Self {
        let (i, j, k) = indices;
        Triangle {
            indices: (i.into(), j.into(), k.into()),
            bbox,
        }
    }
}

pub struct TriangleMesh {
    positions: Vec<Point3>,
    normals: Vec<Vec3>,
    uvs: Vec<(f32, f32)>,
    triangles: Vec<Triangle>,

    // Members that are filled in during the building step.
    bvh_root: Option<IsoBvhNode>,
}

impl TriangleMesh {
    pub fn build_from_raw(raw: &TriangleMeshRaw) -> Self {
        assert_eq!(raw.indices.len() % 3, 0);
        let triangles = raw
            .indices
            .chunks_exact(3)
            .map(|ijk| {
                let (i, j, k) = (ijk[0] as usize, ijk[1] as usize, ijk[2] as usize);
                let (p0, p1, p2) = (
                    raw.vertices[i].pos,
                    raw.vertices[j].pos,
                    raw.vertices[k].pos,
                );
                let bbox = BBox::new(p0, p1).union(p2);
                Triangle::new((i, j, k), bbox)
            })
            .collect::<Vec<_>>();
        let mut mesh = Self {
            positions: vec![],
            normals: vec![],
            uvs: vec![],
            triangles,
            bvh_root: None,
        };
        for vertex in raw.vertices.iter() {
            let Vertex { pos, normal, uv } = vertex.clone();
            mesh.positions.push(pos);
            mesh.normals.push(normal);
            mesh.uvs.push(uv);
        }
        let num_triangles = mesh.triangles.len();
        mesh.bvh_root = Some(recursive_build(
            &mut mesh.triangles,
            0..num_triangles,
            |t: &Triangle| t.bbox,
        ));
        mesh
    }

    pub fn from_soa(
        positions: Vec<Point3>,
        normals: Vec<Vec3>,
        uvs: Vec<(f32, f32)>,
        indices: Vec<(usize, usize, usize)>,
    ) -> Self {
        let triangles = indices
            .into_iter()
            .map(|(i, j, k)| {
                let bbox = BBox::new(positions[i], positions[j]).union(positions[k]);
                Triangle::new((i, j, k), bbox)
            })
            .collect::<Vec<_>>();
        let mut mesh = Self {
            positions,
            normals,
            uvs,
            triangles,
            bvh_root: None,
        };
        let num_triangles = mesh.triangles.len();
        mesh.bvh_root = Some(recursive_build(
            &mut mesh.triangles,
            0..num_triangles,
            |t: &Triangle| t.bbox,
        ));
        mesh
    }

    fn intersect_triangle(&self, tri: &Triangle, r: &Ray) -> Option<Interaction> {
        let (i, k, j) = tri.indices;
        let p0 = self.positions[i];
        let p1 = self.positions[j];
        let p2 = self.positions[k];
        let normal = (p0 - p1).cross(p2 - p1).hat();
        // The equation for the plane of the triangle would be:
        // (p - p0).dot(normal) = 0. Plugging in the ray equation $p = o + td$, we have
        // (o + td - p0).dot(normal) = 0  =>  t*dot(d, normal) = dot(p0-o, normal)
        let t = normal.dot(p0 - r.origin) / normal.dot(r.dir);
        let t = r.truncated_t(t)?;
        let p = r.position_at(t);
        // Computes the barycentric coordinates of p with regard to the triangle.
        let b0 = (p - p0).cross(p - p1).dot(normal);
        let b1 = (p - p1).cross(p - p2).dot(normal);
        let b2 = (p - p2).cross(p - p0).dot(normal);
        let (b0, b1, b2) = match (b0 > 0.0, b1 > 0.0, b2 > 0.0) {
            (true, true, true) | (false, false, false) => {
                let total_area = b0 + b1 + b2;
                (b0 / total_area, b1 / total_area, b2 / total_area)
            }
            _ => return None,
        };
        // Now an intersection is truly found.
        Some(Interaction {
            pos: float::barycentric_lerp((p0, p1, p2), (b1, b2, b0)),
            normal,
            ray_t: t,
            // TODO
            uv: (0.0, 0.0),
        })
    }

    pub fn bvh_shape_summary(&self) -> String {
        let bvh = self.bvh_root.as_ref().unwrap();
        format!("height = {}, node count = {}", bvh.height(), bvh.count())
    }

    // fn positions(&self, ijk: (i32, i32, i32)) -> (Point3, Point3, Point3) {
    //     let (i, j, k) = ijk;
    //     let p = &self.positions;
    //     (p[i as usize], p[j as usize], p[k as usize])
    // }
}

#[derive(Clone, Copy)]
pub struct Vertex {
    pub pos: Point3,
    pub normal: Vec3,
    pub uv: (f32, f32),
}

pub struct TriangleMeshRaw {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<i32>,
}

impl Vertex {
    pub fn zero() -> Self {
        Vertex {
            pos: Point3::origin(),
            normal: Vec3::zero(),
            uv: (0.0, 0.0),
        }
    }
}


impl<T> Shape for IsoBlas<T>
where
    T: Shape,
{
    fn summary(&self) -> String {
        if self.shapes.is_empty() {
            String::from("EmptyBlas")
        } else {
            let shape_summary = self.shapes[0].summary();
            let shape_name = match shape_summary.find('{') {
                None => &shape_summary,
                Some(end) => &shape_summary[0..end],
            };
            format!("Blas of {} {}s", self.shapes.len(), shape_name)
        }
    }
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        let tree = self.bvh_root.as_ref()?;
        intersect_bvh(&self.shapes, tree, r, |s: &T, r| s.intersect(r))
    }

    fn bbox(&self) -> BBox {
        self.bbox
    }
}

impl Shape for TriangleMesh {
    fn summary(&self) -> String {
        format!(
            "TriangleMesh{{{} triangles, {} vertices, bbox = {}, bvh = {}}}",
            self.triangles.len(),
            self.positions.len(),
            self.bbox(),
            self.bvh_shape_summary()
        )
    }
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        let tree = self.bvh_root.as_ref()?;
        // self.intersect_tree(tree, r)
        intersect_bvh(&self.triangles, tree, r, |tri: &Triangle, r| {
            self.intersect_triangle(tri, r)
        })
    }
    fn bbox(&self) -> BBox {
        match &self.bvh_root {
            None => self
                .triangles
                .iter()
                .fold(BBox::empty(), |b, t| bvh::union(b, t.bbox)),
            Some(node) => node.bbox,
        }
    }
}

fn recursive_build<S, F>(shapes: &mut Vec<S>, range: Range<usize>, box_getter: F) -> IsoBvhNode
where
    F: Fn(&S) -> BBox + Copy,
{
    if range.len() <= 4 {
        let bbox = shapes[range.clone()]
            .iter()
            .fold(BBox::empty(), |b, shape| bvh::union(b, box_getter(shape)));
        return IsoBvhNode {
            bbox,
            content: Leaf(range),
        };
    }

    let mut bboxes: Vec<_> = shapes[range.clone()]
        .iter()
        .map(|s| box_getter(s))
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
    let (left, right) = partition(&mut shapes[range.clone()], |s| {
        box_getter(s).midpoint()[split_axis] <= pivot_value
    });
    let mut mid_point = left.len() + range.start;
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

    if left.len() == 0 || right.len() == 0 {
        shapes[range.clone()].select_nth_unstable_by(range.len() / 2, |s0, s1| {
            box_getter(s0).midpoint()[split_axis]
                .partial_cmp(&box_getter(s1).midpoint()[split_axis])
                .unwrap()
        });
        mid_point = range.start + range.len() / 2;
    }
    assert!(mid_point != range.start && mid_point != range.end);

    let left_child = recursive_build(shapes, range.start..mid_point, box_getter);
    let right_child = recursive_build(shapes, mid_point..range.end, box_getter);

    IsoBvhNode {
        bbox: bvh::union(left_child.bbox, right_child.bbox),
        content: Children([Box::new(left_child), Box::new(right_child)]),
    }
}

fn intersect_bvh<S, F>(
    shapes: &Vec<S>,
    tree: &IsoBvhNode,
    r: &Ray,
    shape_intersector: F,
) -> Option<Interaction>
where
    F: Fn(&S, &Ray) -> Option<Interaction> + Copy,
{
    match &tree.content {
        Leaf(range) => {
            let mut hit: Option<Interaction> = None;
            // Ranges are not `Copy`: https://github.com/rust-lang/rust/pull/27186
            for shape in shapes[range.clone()].iter() {
                match (shape_intersector(shape, r), hit) {
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
            let left_isect = intersect_bvh(shapes, &*left, &ray, shape_intersector);
            if let Some(isect) = left_isect {
                ray.set_extent(isect.ray_t);
            }
            let right_isect = intersect_bvh(shapes, &*right, &ray, shape_intersector);
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
