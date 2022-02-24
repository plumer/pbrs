use crate::Interaction;
use crate::Shape;
use geometry::bvh::{self, BBox};
use geometry::ray::Ray;
use math::float::barycentric_lerp;
use math::hcm::{Point3, Vec3};
use partition::partition;
use std::ops::Range;

enum IsoBvhNodeContent {
    Children([Box<IsoBvhNode>; 2], usize),
    Leaf(Range<usize>),
}

struct IsoBvhNode {
    bbox: BBox,
    content: IsoBvhNodeContent,
}

impl IsoBvhNode {
    fn height(&self) -> usize {
        match &self.content {
            Children([left, right], _) => std::cmp::max(left.height(), right.height()) + 1,
            Leaf(_) => 1,
        }
    }
    fn count(&self) -> usize {
        match &self.content {
            Children([left, right], _) => left.count() + right.count() + 1,
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
    T: Shape + std::fmt::Debug,
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

#[derive(Debug)]
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
        positions: Vec<Point3>, normals: Vec<Vec3>, uvs: Vec<(f32, f32)>,
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
        let (p0, p1, p2) = (self.positions[i], self.positions[j], self.positions[k]);
        let hit = crate::intersect_triangle(p0, p1, p2, r)?;

        let (b0, b1, b2) = (1.0 - hit.uv.0 - hit.uv.1, hit.uv.0, hit.uv.1);
        let hit_by_uv = p0 + (p1 - p0) * b1 + (p2 - p0) * b2;
        assert!(hit_by_uv.squared_distance_to(hit.pos) < 1e-6);
        // Interpolates the normal vector using existing uv coordinates.
        let (n0, n1, n2) = (self.normals[i], self.normals[j], self.normals[k]);
        let bclerp_normal = barycentric_lerp((n0, n1, n2), (b0, b1, b2))
            .try_hat()
            .unwrap_or(hit.normal)
            .facing(r.dir);
        // Interpolates the UV coordinate using existing uv coordinates.
        let bclerp_uv = (
            barycentric_lerp((self.uvs[i].0, self.uvs[j].0, self.uvs[k].0), (b0, b1, b2)),
            barycentric_lerp((self.uvs[i].1, self.uvs[j].1, self.uvs[k].1), (b0, b1, b2)),
        );
        // Uses uv coordinates to solve the tangent.
        let (u0, v0) = self.uvs[i];
        let (u1, v1) = self.uvs[j];
        let (u2, v2) = self.uvs[k];
        let (u1, v1) = (u1 - u0, v1 - v0);
        let (u2, v2) = (u2 - u0, v2 - v0);
        let dpdu = ((p2 - p0) * v2 - (p1 - p0) * v1) / (u1 * v2 - u2 * v1);
        let dpdu = if dpdu.norm_squared().is_finite() {
            dpdu
        } else {
            p1 - p0
        };
        let dpdu = (dpdu - dpdu.projected_onto(bclerp_normal)).hat();
        if dpdu.dot(bclerp_normal).abs() >= 1e-3 {
            eprintln!(
                "dpdu({:.5}) still not perp with normal({:.5}), dot product = {}",
                dpdu,
                bclerp_normal,
                dpdu.dot(bclerp_normal)
            );
            None
        } else {
            Some(
                Interaction::new(hit.pos, hit.ray_t, bclerp_uv, bclerp_normal, hit.wo)
                    .with_dpdu(dpdu),
            )
        }
    }
    fn intersect_triangle_pred(&self, tri: &Triangle, r: &Ray) -> bool {
        let (i, k, j) = tri.indices;
        crate::intersect_triangle_pred(self.positions[i], self.positions[j], self.positions[k], r)
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

#[derive(Debug, Clone, Copy)]
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
            pos: Point3::ORIGIN,
            normal: Vec3::ZERO,
            uv: (0.0, 0.0),
        }
    }
}

impl std::fmt::Display for Vertex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let prec = f.precision().unwrap_or(2);
        write!(
            f,
            "{} {} ({:.p$}, {:.p$})",
            self.pos,
            self.normal,
            self.uv.0,
            self.uv.1,
            p = prec
        )
    }
}

impl<T> Shape for IsoBlas<T>
where
    T: Shape,
{
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        let tree = self.bvh_root.as_ref()?;
        intersect_bvh(&self.shapes, tree, r, |s: &T, r| s.intersect(r))
    }
    fn occludes(&self, r: &Ray) -> bool {
        let tree = self.bvh_root.as_ref().unwrap();
        intersect_bvh_pred(&self.shapes, tree, r, |s, r| s.occludes(r))
    }

    fn bbox(&self) -> BBox {
        self.bbox
    }

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
}

impl Shape for TriangleMesh {
    fn intersect(&self, r: &Ray) -> Option<Interaction> {
        let tree = self.bvh_root.as_ref()?;
        // self.intersect_tree(tree, r)
        intersect_bvh(&self.triangles, tree, r, |tri: &Triangle, r| {
            let result = self.intersect_triangle(tri, r);
            if let Some(hit) = result {
                assert!(!hit.pos.has_nan());
                assert!(hit.has_valid_frame());
            }
            result
        })
    }
    fn occludes(&self, r: &Ray) -> bool {
        let tree = self.bvh_root.as_ref().unwrap();
        intersect_bvh_pred(&self.triangles, tree, r, |tri, r| {
            self.intersect_triangle_pred(tri, r)
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
    fn summary(&self) -> String {
        format!(
            "TriangleMesh{{{} triangles, {} vertices, bbox = {}, bvh = {}}}",
            self.triangles.len(),
            self.positions.len(),
            self.bbox(),
            self.bvh_shape_summary()
        )
    }
}

fn recursive_build<S, F>(shapes: &mut Vec<S>, range: Range<usize>, box_getter: F) -> IsoBvhNode
where
    S: std::fmt::Debug,
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
    let split_axis = centroid_bbox.diag().max_dimension();
    if centroid_bbox.diag()[split_axis] < 1e-8 {
        eprintln!("Creating a tiny leaf node with {} shapes", range.len());
        return IsoBvhNode {
            bbox: bboxes
                .iter()
                .fold(BBox::empty(), |b0, b1| bvh::union(b0, *b1)),
            content: Leaf(range),
        };
    }

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
        content: Children([Box::new(left_child), Box::new(right_child)], split_axis),
    }
}

fn intersect_bvh<S, F>(
    shapes: &Vec<S>, tree: &IsoBvhNode, r: &Ray, shape_intersector: F,
) -> Option<Interaction>
where
    F: Fn(&S, &Ray) -> Option<Interaction> + Copy,
{
    if !tree.bbox.intersect(r) {
        return None;
    }

    let mut node_stack = Vec::new();
    node_stack.reserve(60);

    node_stack.push(tree);

    let mut outer_hit =
        Interaction::new(Point3::ORIGIN, f32::INFINITY, (0.0, 0.0), Vec3::Z, Vec3::Z);
    let mut ray = r.clone();
    while let Some(node) = node_stack.pop() {
        if !node.bbox.intersect(&ray) {
            continue;
        }
        match &node.content {
            Leaf(range) => {
                // let mut hit: Option<Interaction> = None;
                // Ranges are not `Copy`: https://github.com/rust-lang/rust/pull/27186
                for shape in shapes[range.clone()].iter() {
                    if let Some(new_isect) = shape_intersector(shape, &ray) {
                        if new_isect.ray_t < outer_hit.ray_t {
                            outer_hit = new_isect;
                        }
                    }
                }
            }
            Children([left, right], axis) => {
                // which one is farther away?
                if ray.dir[*axis] > 0.0 {
                    // Ray roughly intersects left first.
                    node_stack.push(right);
                    node_stack.push(left);
                } else {
                    node_stack.push(left);
                    node_stack.push(right);
                }
            }
        }
        ray.t_max = outer_hit.ray_t;
    }

    if outer_hit.ray_t < f32::INFINITY {
        Some(outer_hit)
    } else {
        None
    }
}

fn intersect_bvh_pred<S, F>(
    shapes: &[S], tree: &IsoBvhNode, r: &Ray, shape_intersect_pred: F,
) -> bool
where
    F: Fn(&S, &Ray) -> bool + Copy,
{
    let mut node_stack = Vec::new();
    node_stack.push(tree);

    while let Some(node) = node_stack.pop() {
        if !node.bbox.intersect(r) {
            continue;
        }
        match &node.content {
            Leaf(range) => {
                if shapes[range.clone()]
                    .iter()
                    .any(|shape| shape_intersect_pred(shape, r))
                {
                    return true;
                }
            }
            Children([left, right], axis) => {
                if r.dir[*axis] > 0.0 {
                    // Ray roughly intersects left first.
                    node_stack.push(right);
                    node_stack.push(left);
                } else {
                    node_stack.push(left);
                    node_stack.push(right);
                }
            }
        }
    }

    return false;
}

#[test]
fn tricky_triangle() {
    use math::hcm::{point3, vec3};
    let points = vec![
        point3(10.3457699, 27.3706398, -21.2291069),
        point3(10.3457699, 13.3905125, -21.1700611),
        point3(7.22226286, 13.3905125, -21.1700611),
    ];
    let normals = vec![
        Vec3 {
            x: 0.0,
            y: 0.00419999985,
            z: 1.0,
        };
        3
    ];
    let uvs = vec![(0.0, 0.0); 3];

    let mesh = TriangleMesh::from_soa(points, normals, uvs, vec![(0, 1, 2)]);
    let ray = geometry::ray::Ray::new(
        point3(0.0, 23.0, 30.0),
        vec3(0.219424784, -0.0887561888, -1.08688462),
    );

    mesh.intersect(&ray);
}
