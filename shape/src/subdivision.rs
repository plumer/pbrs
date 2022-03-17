use std::collections::{HashMap, VecDeque};
use std::f32::consts::PI;
use std::hash::Hash;

use crate::blas::TriangleMesh;
use math::float::lerp;
use math::hcm::{Point3, Vec3};

// static NEXT: [usize; 3] = [1, 2, 0];
// static PREV: [usize; 3] = [2, 0, 1];

#[derive(Debug, Clone)]
enum NeighborList {
    Boundary(Vec<usize>),
    Circular(Vec<usize>),
}

impl NeighborList {
    // fn regular(&self) -> bool {
    //     match self {
    //         Self::Boundary(l) => l.len() == 4,
    //         Self::Circular(l) => l.len() == 6,
    //     }
    // }
    fn boundary_neighbors(&self) -> Option<[usize; 2]> {
        match self {
            Self::Boundary(l) => {
                assert!(l.len() >= 2);
                Some([l[0], l[l.len() - 1]])
            }
            Self::Circular(_) => None,
        }
    }
    ///
    /// ```
    /// let circular_list = subdivision::NeighborList::Circular(vec![5, 6, 7, 8]);
    /// let triples = circular_list.fan_triples();
    /// assert_eq!(triples, vec![(5, 6, 7), (6, 7, 8), (7, 8, 5), (8, 5, 6)]);
    /// ```
    fn fan_triples(&self) -> Vec<(usize, usize, usize)> {
        match self {
            Self::Boundary(l) => l
                .windows(3)
                .map(|ijk| (ijk[0], ijk[1], ijk[2]))
                .collect::<Vec<_>>(),
            Self::Circular(l) => {
                assert!(l.len() >= 3);
                let extended_l = l.iter().cycle().take(l.len() + 2).collect::<Vec<_>>();
                extended_l
                    .windows(3)
                    .map(|ijk| (*ijk[0], *ijk[1], *ijk[2]))
                    .collect::<Vec<_>>()
            }
        }
    }
}

/// Undirected edge.
#[derive(Debug, Clone, Copy, Hash, PartialEq, Eq)]
struct Edge {
    s: usize,
    t: usize,
}
impl Edge {
    fn new(u: usize, v: usize) -> Self {
        Edge {
            s: u.min(v),
            t: u.max(v),
        }
    }
}
fn edge(u: usize, v: usize) -> Edge {
    Edge::new(u, v)
}

pub fn loop_subdivide(
    positions: &Vec<Point3>, index_triples: &[(usize, usize, usize)],
) -> TriangleMesh {
    let mut tri_list_map = vec![Vec::new(); positions.len()];
    for t in index_triples.iter() {
        let (i, j, k) = *t;
        if i == j || j == k || k == i {
            continue;
        }
        tri_list_map[i].push(*t);
        tri_list_map[j].push(*t);
        tri_list_map[k].push(*t);
    }

    // Build a list of indices to neighboring vertices, oriented in either CW or CCW order.
    // ------------------------------------------------------------------------------------
    let mut nb_list_map = Vec::new();
    for vi in 0..tri_list_map.len() {
        let tri_list = &tri_list_map[vi];
        let nb_list = build_nb_list(tri_list, vi);
        nb_list_map.push(nb_list);
    }

    // p_center * (1-n*beta) + sum(ni * beta)
    let weighted_one_ring = |center: usize, neighbors_index: &[usize], beta: f32| {
        let center_p = positions[center];
        (neighbors_index.iter())
            .map(|i| (positions[*i as usize] - center_p) * beta)
            .sum::<Vec3>()
            + center_p
    };
    fn beta(valence: usize) -> f32 {
        if valence == 3 {
            3.0 / 16.0
        } else {
            3.0 / (8.0 * valence as f32)
        }
    }

    // Computes the positions of new points inserted onto each edge, and the updated positions of
    // each existing vertex (the original positions are left untouched, and a new copy is made).
    let mut updated_positions = Vec::new();
    let mut inserted_vertices = HashMap::new();
    let mut edge_vert_ids = HashMap::new();
    for vi in 0..nb_list_map.len() {
        // Builds the updated version of existing vertices.
        let updated_vert_pos = match &nb_list_map[vi] {
            NeighborList::Boundary(list) => {
                // v' = (1 - 2\beta) + beta (v1 + v2), where v1 and v2 are the neighbor vertices
                // along the boundary.
                let v12 = [list[0], *list.last().unwrap()];
                weighted_one_ring(vi, &v12, 1.0 / 8.0)
            }
            NeighborList::Circular(list) => {
                // v' = (1 - n\beta) + sum(\beta * ni) where ni is the i-th neighbor.
                weighted_one_ring(vi, &list, beta(list.len()))
            }
        };
        updated_positions.push(updated_vert_pos);

        // Builds the new vertices along each edge.
        if let Some(boundary_neighbors) = nb_list_map[vi].boundary_neighbors() {
            for n in boundary_neighbors {
                let e = edge(vi, n);
                assert_eq!(
                    inserted_vertices.contains_key(&e),
                    edge_vert_ids.contains_key(&e)
                );
                if inserted_vertices.contains_key(&e) {
                    continue;
                }
                assert!(
                    !inserted_vertices.contains_key(&edge(n, vi)),
                    "edges = {:?}, edge = {:?}",
                    inserted_vertices.keys().collect::<Vec<_>>(),
                    (n, vi)
                );
                let new_pos = lerp(positions[n as usize], positions[vi], 0.5);
                assert!(inserted_vertices.insert(e, new_pos).is_none());
                assert!(edge_vert_ids
                    .insert(e, edge_vert_ids.len() + positions.len())
                    .is_none());
            }
        }

        //        u    q
        //        /\‾‾/         (u, vi): the current edge where a new edge will be inserted,
        //       /__\/           u and vi are weighted by 3/8; p and q are weighted by 1/8.
        //      p    vi         (p, q): vertices on the opposite site.
        for (p, u, q) in nb_list_map[vi].fan_triples() {
            let e = edge(vi, u);
            assert_eq!(
                inserted_vertices.contains_key(&e),
                edge_vert_ids.contains_key(&e),
                "#inserted_vertices - #edge_vert_ids = {:?}",
                inserted_vertices.len() as isize - edge_vert_ids.len() as isize
            );
            if inserted_vertices.contains_key(&e) {
                continue;
            }
            assert!(!inserted_vertices.contains_key(&edge(u, vi)));
            let opposite_component = lerp(positions[p as usize], positions[q as usize], 0.5);
            let inner_component = lerp(positions[vi], positions[u as usize], 0.5);
            let new_edge_vert = lerp(opposite_component, inner_component, 0.75);
            assert!(inserted_vertices.insert(e, new_edge_vert).is_none());
            assert!(edge_vert_ids
                .insert(e, edge_vert_ids.len() + positions.len())
                .is_none());
        }

        // TODO: computes the normal for each vertex, using the loop subdivision at its limit.
    }
    let mut all_vertices = updated_positions;
    all_vertices.resize(all_vertices.len() + inserted_vertices.len(), Point3::ORIGIN);
    for (edge, v) in inserted_vertices.into_iter() {
        let vert_id = edge_vert_ids.get(&edge).unwrap();
        all_vertices[*vert_id] = v;
    }

    let mut subdiv_index_triples = Vec::new();
    for t in index_triples.iter() {
        let (i, j, k) = *t;
        let vij = *edge_vert_ids.get(&edge(i, j)).unwrap();
        let vjk = *edge_vert_ids.get(&edge(j, k)).unwrap();
        let vki = *edge_vert_ids.get(&edge(k, i)).unwrap();

        // eprintln!(
        //     "new triples: {:?}",
        //     [(i, vij, vki), (vij, j, vjk), (vki, vij, vjk), (vki, vjk, k)]
        // );

        //     k
        //    / \
        //   ki-jk
        //  / \ / \
        // i___ij__j
        subdiv_index_triples.extend([(i, vij, vki), (vij, j, vjk), (vki, vij, vjk), (vki, vjk, k)]);
    }
    // println!("all new triples: {:?}", subdiv_index_triples);
    let normals = geometry::compute_normals(&all_vertices, &subdiv_index_triples);
    let uvs = vec![(0.0, 0.0); normals.len()];
    TriangleMesh::from_soa(all_vertices, normals, uvs, subdiv_index_triples)
}

#[allow(dead_code)]
fn build_normals(positions: &Vec<Point3>, index_triples: &[(usize, usize, usize)]) -> Vec<Vec3> {
    let mut tri_list_map = vec![Vec::new(); positions.len()];
    for t in index_triples.iter() {
        let (i, j, k) = *t;
        tri_list_map[i as usize].push(*t);
        tri_list_map[j as usize].push(*t);
        tri_list_map[k as usize].push(*t);
    }
    // Build a list of indices to neighboring vertices, oriented in either CW or CCW order.
    // ------------------------------------------------------------------------------------
    let mut nb_list_map = Vec::new();
    for vi in 0..tri_list_map.len() {
        let tri_list = &tri_list_map[vi];
        let nb_list = build_nb_list(tri_list, vi);
        nb_list_map.push(nb_list);
    }
    nb_list_map
        .into_iter()
        .enumerate()
        .map(|(center_i, neighbors)| match neighbors {
            NeighborList::Boundary(l) => {
                // 2               −2      (1, 1)
                // 3               −1      (0, 1, 0)
                // 4 (regular)     −2      (−1, 2, 2, −1)
                assert!(l.len() >= 2);
                let pc = positions[center_i];
                let pnb = l.iter().map(|i| positions[*i]).collect::<Vec<_>>();
                let s = pnb[l.len() - 1] - pnb[0];
                let t = match l.len() {
                    2 => pnb[0] - pc + pnb[1] - pc,
                    3 => pnb[1] - pc,
                    4 => (pnb[1] - pnb[0]) + (pnb[1] - pc) + (pnb[2] - pc) + (pnb[2] - pnb[3]),
                    valence => {
                        let theta = PI / (valence - 1) as f32;
                        let (sin_t, cos_t) = theta.sin_cos();
                        let a = 2.0 * cos_t - 2.0;
                        (1..valence - 1)
                            .map(|k| Vec3::from(pnb[k]) * a * (k as f32 * theta).sin())
                            .sum::<Vec3>()
                            + sin_t * (Vec3::from(pnb[0]) + Vec3::from(pnb[valence - 1]))
                    }
                };
                s.cross(t).try_hat().unwrap_or(Vec3::X)
            }
            NeighborList::Circular(l) => {
                let (mut s, mut t) = (Vec3::ZERO, Vec3::ZERO);
                for i in 0..l.len() {
                    let angle_rad = 2.0 * PI * i as f32 / l.len() as f32;
                    let nb_v = Vec3::from(positions[l[i] as usize]);
                    s += angle_rad.cos() * nb_v;
                    t += angle_rad.sin() * nb_v;
                }
                s.cross(t).try_hat().unwrap_or(Vec3::Y)
            }
        })
        .collect::<Vec<_>>()
}

fn build_nb_list(tri_list: &[(usize, usize, usize)], center: usize) -> NeighborList {
    let neighbors_of = |indices: &(usize, usize, usize)| -> (usize, usize) {
        let (i, j, k) = *indices;
        if center == i {
            (j, k)
        } else if center == j {
            (i, k)
        } else if center == k {
            (i, j)
        } else {
            panic!()
        }
    };
    let mut opposite_edges = tri_list.iter().map(neighbors_of).collect::<Vec<_>>();
    fn find_any_endpoint(edge_list: &Vec<(usize, usize)>, index: usize) -> Option<usize> {
        for i in 0..edge_list.len() {
            if edge_list[i].0 == index || edge_list[i].1 == index {
                return Some(i);
            }
        }
        None
    }
    let mut edge_list = VecDeque::new();
    edge_list.push_back(opposite_edges.pop().unwrap());
    while !opposite_edges.is_empty() {
        let head = edge_list.front().unwrap().0;
        let tail = edge_list.back().unwrap().1;

        let front = find_any_endpoint(&opposite_edges, head);
        if let Some(i_front) = front {
            let (s, t) = opposite_edges.swap_remove(i_front);
            if s == head {
                edge_list.push_front((t, s));
            } else {
                assert!(t == head);
                edge_list.push_front((s, t));
            }
        }
        let back = find_any_endpoint(&opposite_edges, tail);
        if let Some(i_back) = back {
            let (s, t) = opposite_edges.swap_remove(i_back);
            if s == tail {
                edge_list.push_back((s, t));
            } else {
                assert!(t == tail);
                edge_list.push_back((t, s));
            }
        }
        if front.or(back).is_none() {
            panic!("no progress. center = {center}, tri_list = {:?}, edge_list = {:?}, opposite_edges = {:?}",
                    tri_list, edge_list,opposite_edges);
        }
    }

    let mut list = edge_list.iter().map(|(s, _)| *s).collect::<Vec<_>>();
    list.push(edge_list.back().unwrap().1);
    assert!(!list.is_empty());
    if edge_list.front().unwrap().0 == edge_list.back().unwrap().1 {
        NeighborList::Circular(list[1..].to_vec())
    } else {
        NeighborList::Boundary(list)
    }
}

#[cfg(test)]
mod test {
    use math::hcm;
    use std::collections::HashMap;

    use super::loop_subdivide;
    use super::{Edge, NeighborList};

    #[test]
    fn edge_equality_test() {
        let e0 = Edge::new(0, 4);
        let e1 = Edge::new(4, 0);

        assert_eq!(e0, e1);
        let mut hashmap = HashMap::new();
        hashmap.insert(e0, 'y');
        assert!(hashmap.get(&e0) == Some(&'y'));
        assert!(hashmap.contains_key(&e1));
    }

    #[test]
    fn neighbor_list_test() {
        let circular_list = NeighborList::Circular(vec![5, 6, 7, 8]);
        let triples = circular_list.fan_triples();
        assert_eq!(triples, vec![(5, 6, 7), (6, 7, 8), (7, 8, 5), (8, 5, 6)]);
        assert!(circular_list.boundary_neighbors().is_none());

        let boundary_list = NeighborList::Boundary(vec![5, 6, 7, 8]);
        assert_eq!(boundary_list.fan_triples(), vec![(5, 6, 7), (6, 7, 8)]);
        assert_eq!(boundary_list.boundary_neighbors(), Some([5, 8]));
    }

    #[test]
    fn test_quad3x3() {
        let side_length = 4;
        // let num_points = side_length * side_length;
        let mut index_triples = Vec::new();
        let mut positions = Vec::new();
        for r in 0..side_length - 1 {
            for c in 0..side_length - 1 {
                let ll = r * side_length + c;
                let lr = ll + 1;
                let tl = ll + side_length;
                let tr = tl + 1;
                index_triples.push((tl, ll, lr));
                index_triples.push((tl, lr, tr));
            }
        }
        for r in 0..side_length {
            for c in 0..side_length {
                positions.push(hcm::point3(c as f32, r as f32, 0.5));
            }
        }
        loop_subdivide(&positions, &index_triples);
    }

    #[test]
    fn test_killeroo() {
        let indices = vec![
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 2, 12, 13, 14, 3, 15, 16, 17, 18, 19, 16, 20,
            21, 22, 23, 24, 25, 22, 26, 20, 18, 16, 27, 26, 28, 20, 29, 30, 31, 32, 15, 33, 9, 11,
            34, 31, 35, 29, 36, 37, 38, 39, 40, 28, 36, 38, 41, 25, 42, 43, 41, 38, 44, 45, 46, 27,
            11, 10, 47, 2, 48, 4, 49, 50, 36, 51, 49, 36, 51, 36, 41, 52, 6, 53, 54, 55, 56, 14, 4,
            3, 57, 6, 52, 58, 59, 60, 7, 6, 57, 61, 2, 14, 57, 62, 7, 24, 63, 64, 65, 24, 23, 66,
            67, 68, 69, 55, 70, 71, 59, 72, 40, 73, 28, 74, 73, 40, 16, 15, 27, 75, 27, 15, 75, 15,
            32, 32, 76, 50, 77, 78, 79, 80, 75, 32, 45, 75, 81, 45, 27, 75, 32, 50, 80, 80, 81, 75,
            82, 42, 79, 49, 80, 50, 83, 80, 49, 84, 45, 81, 84, 46, 45, 85, 86, 79, 87, 42, 82, 86,
            87, 82, 80, 83, 88, 81, 80, 89, 89, 84, 81, 83, 49, 90, 88, 89, 80, 91, 84, 89, 92, 85,
            78, 79, 86, 82, 78, 85, 79, 34, 22, 21, 88, 83, 90, 76, 32, 33, 93, 88, 90, 59, 94, 60,
            65, 95, 96, 97, 8, 98, 98, 85, 92, 87, 43, 42, 98, 8, 85, 65, 96, 63, 99, 5, 100, 57,
            52, 65, 43, 87, 23, 86, 101, 87, 62, 101, 86, 102, 100, 66, 57, 65, 23, 62, 57, 101,
            78, 103, 92, 101, 57, 23, 48, 72, 104, 58, 60, 105, 106, 107, 108, 87, 101, 23, 85, 62,
            86, 7, 62, 8, 8, 62, 85, 39, 28, 26, 56, 74, 40, 109, 22, 34, 55, 54, 70, 104, 59, 110,
            69, 111, 112, 31, 106, 35, 113, 22, 109, 66, 114, 67, 39, 26, 115, 116, 117, 118, 25,
            24, 119, 77, 79, 108, 72, 59, 104, 112, 56, 55, 110, 67, 114, 47, 120, 121, 20, 28,
            122, 5, 110, 114, 69, 112, 55, 77, 108, 107, 112, 74, 56, 117, 115, 118, 91, 89, 95,
            108, 79, 42, 58, 105, 67, 52, 95, 65, 123, 77, 107, 119, 35, 106, 52, 91, 95, 100, 5,
            114, 5, 104, 110, 70, 124, 125, 65, 63, 24, 0, 12, 111, 6, 8, 97, 48, 104, 4, 1, 48, 2,
            70, 125, 0, 99, 3, 5, 0, 125, 1, 4, 104, 5, 126, 98, 103, 96, 127, 128, 13, 99, 129,
            130, 14, 13, 1, 53, 48, 91, 53, 1, 53, 131, 48, 90, 51, 93, 127, 93, 132, 126, 97, 98,
            93, 127, 88, 69, 0, 111, 1, 46, 91, 69, 70, 0, 125, 46, 1, 2, 4, 14, 53, 91, 52, 89,
            88, 95, 103, 78, 123, 84, 91, 46, 27, 46, 18, 73, 133, 28, 95, 88, 127, 125, 18, 46,
            118, 115, 134, 95, 127, 96, 78, 77, 123, 106, 25, 119, 108, 42, 106, 13, 3, 99, 47, 10,
            120, 130, 135, 14, 106, 42, 25, 6, 97, 136, 61, 137, 12, 33, 17, 138, 19, 17, 16, 17,
            139, 138, 61, 12, 2, 38, 140, 44, 102, 66, 68, 17, 118, 139, 34, 11, 109, 141, 142,
            143, 19, 18, 144, 17, 33, 15, 124, 70, 145, 140, 11, 146, 109, 11, 140, 147, 94, 148,
            38, 109, 140, 116, 144, 145, 37, 109, 38, 116, 118, 17, 116, 19, 144, 18, 124, 144, 19,
            116, 17, 67, 110, 58, 100, 129, 99, 149, 13, 129, 102, 68, 149, 149, 129, 102, 150, 13,
            149, 130, 13, 150, 68, 67, 151, 68, 151, 149, 145, 144, 124, 152, 150, 149, 136, 97,
            126, 135, 61, 14, 130, 150, 152, 149, 151, 153, 135, 130, 152, 152, 149, 153, 153, 151,
            105, 137, 135, 152, 11, 47, 146, 154, 153, 105, 47, 121, 146, 155, 153, 154, 152, 153,
            155, 117, 40, 39, 54, 56, 40, 70, 54, 145, 154, 12, 137, 137, 152, 155, 30, 103, 123,
            54, 40, 117, 60, 154, 105, 117, 39, 115, 103, 30, 126, 154, 137, 155, 34, 21, 9, 29,
            126, 30, 145, 54, 116, 148, 156, 142, 94, 154, 60, 105, 151, 67, 147, 154, 94, 116, 54,
            117, 10, 143, 157, 141, 143, 10, 142, 141, 122, 126, 29, 136, 147, 12, 154, 147, 158,
            112, 9, 141, 10, 21, 141, 9, 122, 141, 20, 12, 147, 111, 74, 158, 73, 111, 147, 112,
            20, 141, 21, 28, 133, 122, 25, 43, 23, 71, 48, 131, 159, 136, 29, 72, 48, 71, 158, 133,
            73, 71, 131, 159, 64, 59, 71, 160, 161, 162, 163, 162, 161, 128, 64, 63, 128, 163, 64,
            164, 163, 128, 164, 162, 163, 136, 159, 131, 128, 63, 96, 165, 162, 164, 53, 6, 136,
            61, 135, 137, 64, 94, 59, 94, 64, 161, 158, 148, 133, 161, 160, 156, 133, 148, 122,
            142, 166, 143, 147, 148, 158, 92, 103, 98, 24, 64, 71, 148, 161, 156, 119, 24, 159,
            148, 94, 161, 100, 114, 66, 156, 160, 166, 148, 142, 122, 159, 29, 35, 35, 119, 159,
            112, 158, 74, 159, 24, 71, 64, 163, 161, 100, 102, 129, 156, 166, 142, 160, 162, 166,
            107, 31, 123, 107, 106, 31, 123, 31, 30, 22, 115, 26, 18, 125, 124, 113, 115, 22, 134,
            115, 113, 118, 134, 139, 37, 134, 113, 109, 37, 113, 139, 134, 37, 110, 59, 58, 139,
            37, 167, 138, 139, 167, 33, 138, 167, 37, 36, 167, 76, 33, 167, 76, 167, 36, 76, 36,
            50, 131, 53, 136, 49, 51, 90,
        ];
        let index_triples = indices
            .chunks(3)
            .map(|ijk| (ijk[0], ijk[1], ijk[2]))
            .collect::<Vec<(usize, usize, usize)>>();
        let num_points = indices.iter().max().unwrap().clone() as usize + 1;
        let positions = vec![hcm::Point3::ORIGIN; num_points];
        loop_subdivide(&positions, &index_triples);
    }
    #[test]
    fn index_remap() {
        let indices = [
            610, 641, 131, 587, 667, 621, 248, 12, 406, 653, 936, 941, 610, 131, 824, 727, 668,
            587, 177, 794, 85, 100, 781, 794, 933, 350, 613, 494, 1135, 1252, 613, 557, 933, 100,
            794, 427, 557, 943, 933, 925, 892, 1134, 404, 177, 179, 653, 941, 453, 1134, 1137, 925,
            299, 791, 220, 517, 859, 943, 299, 220, 241, 1252, 1268, 547, 241, 220, 211, 403, 72,
            427, 941, 936, 928, 131, 250, 667, 416, 415, 299, 281, 416, 299, 281, 299, 241, 659,
            248, 722, 823, 614, 623, 668, 667, 587, 263, 248, 659, 1211, 1273, 910, 12, 248, 263,
            162, 131, 668, 263, 305, 12, 1135, 1249, 1243, 439, 1135, 494, 1240, 1275, 528, 867,
            614, 644, 985, 1273, 110, 859, 646, 943, 510, 646, 859, 794, 177, 427, 399, 427, 177,
            399, 177, 404, 404, 414, 415, 325, 365, 418, 391, 399, 404, 403, 399, 396, 403, 427,
            399, 404, 415, 391, 391, 396, 399, 480, 1268, 418, 416, 391, 415, 328, 391, 416, 217,
            403, 396, 217, 72, 403, 424, 346, 418, 430, 1268, 480, 346, 430, 480, 391, 328, 337,
            396, 391, 549, 549, 217, 396, 328, 416, 409, 337, 549, 391, 656, 217, 549, 395, 424,
            365, 418, 346, 480, 365, 424, 418, 453, 613, 350, 337, 328, 409, 414, 404, 179, 435,
            337, 409, 1273, 913, 910, 439, 538, 1027, 190, 406, 189, 189, 424, 395, 430, 547, 1268,
            189, 406, 424, 439, 1027, 1249, 597, 621, 1247, 263, 659, 439, 547, 430, 494, 346, 348,
            430, 305, 348, 346, 863, 1247, 1240, 263, 439, 494, 305, 263, 348, 365, 156, 395, 348,
            263, 494, 250, 110, 515, 1211, 910, 1196, 1257, 1254, 1255, 430, 348, 494, 424, 305,
            346, 12, 305, 406, 406, 305, 424, 517, 943, 557, 623, 510, 859, 793, 613, 453, 614,
            823, 644, 515, 1273, 488, 867, 847, 448, 1134, 1257, 1137, 1225, 613, 793, 1240, 580,
            1275, 517, 557, 577, 884, 616, 586, 1252, 1135, 1242, 325, 418, 1255, 110, 1273, 515,
            448, 623, 614, 488, 1275, 580, 928, 909, 1193, 933, 943, 639, 621, 488, 580, 867, 448,
            614, 325, 1255, 1254, 448, 510, 623, 616, 577, 586, 656, 549, 538, 1255, 418, 1268,
            1211, 1196, 1275, 659, 538, 439, 975, 325, 1254, 1242, 1137, 1257, 659, 656, 538, 1247,
            621, 580, 621, 515, 488, 644, 776, 173, 439, 1249, 1135, 610, 824, 847, 248, 406, 190,
            250, 515, 667, 641, 250, 131, 644, 173, 610, 597, 587, 621, 610, 173, 641, 667, 515,
            621, 934, 189, 156, 1027, 660, 1060, 727, 597, 1010, 759, 668, 727, 641, 722, 250, 656,
            722, 641, 722, 826, 250, 409, 281, 435, 660, 435, 16, 934, 190, 189, 435, 660, 337,
            867, 610, 847, 641, 72, 656, 867, 644, 610, 173, 72, 641, 131, 667, 668, 722, 656, 659,
            549, 337, 538, 156, 365, 975, 217, 656, 72, 427, 72, 100, 646, 545, 943, 538, 337, 660,
            173, 100, 72, 586, 577, 1195, 538, 660, 1027, 365, 325, 975, 1257, 1252, 1242, 1255,
            1268, 1257, 727, 587, 597, 928, 936, 909, 759, 633, 668, 1257, 1268, 1252, 248, 190,
            868, 162, 869, 824, 179, 85, 1174, 781, 85, 794, 85, 757, 1174, 162, 824, 131, 220,
            766, 211, 863, 1240, 528, 85, 586, 757, 453, 941, 793, 673, 786, 1163, 781, 100, 739,
            85, 179, 177, 776, 644, 929, 766, 941, 735, 793, 941, 766, 882, 913, 1171, 220, 793,
            766, 884, 739, 929, 791, 793, 220, 884, 586, 85, 884, 781, 739, 100, 776, 739, 781,
            884, 85, 1275, 488, 1211, 1247, 1010, 597, 815, 727, 1010, 863, 528, 815, 815, 1010,
            863, 287, 727, 815, 759, 727, 287, 528, 1275, 1265, 528, 1265, 815, 929, 739, 776, 879,
            287, 815, 868, 190, 934, 633, 162, 668, 759, 287, 879, 815, 1265, 864, 633, 759, 879,
            879, 815, 864, 864, 1265, 1196, 869, 633, 879, 941, 928, 735, 573, 864, 1196, 928,
            1193, 735, 930, 864, 573, 879, 864, 930, 616, 859, 517, 823, 623, 859, 644, 823, 929,
            573, 824, 869, 869, 879, 930, 892, 156, 975, 823, 859, 616, 910, 573, 1196, 616, 517,
            577, 156, 892, 934, 573, 869, 930, 453, 350, 653, 925, 934, 892, 929, 823, 884, 1171,
            822, 786, 913, 573, 910, 1196, 1265, 1275, 882, 573, 913, 884, 823, 616, 936, 1163,
            920, 673, 1163, 936, 786, 673, 639, 934, 925, 868, 882, 824, 573, 882, 919, 448, 653,
            673, 936, 350, 673, 653, 639, 673, 933, 824, 882, 847, 510, 919, 646, 847, 882, 448,
            933, 673, 350, 943, 545, 639, 1252, 547, 494, 985, 250, 826, 1168, 868, 925, 110, 250,
            985, 919, 545, 646, 985, 826, 1168, 1243, 1273, 985, 870, 1130, 940, 951, 940, 1130,
            1060, 1243, 1249, 1060, 951, 1243, 1061, 951, 1060, 1061, 940, 951, 868, 1168, 826,
            1060, 1249, 1027, 897, 940, 1061, 722, 248, 868, 162, 633, 869, 1243, 913, 1273, 913,
            1243, 1130, 919, 1171, 545, 1130, 870, 822, 545, 1171, 639, 786, 1123, 1163, 882, 1171,
            919, 395, 156, 189, 1135, 1243, 985, 1171, 1130, 822, 1242, 1135, 1168, 1171, 913,
            1130, 1247, 580, 1240, 822, 870, 1123, 1171, 786, 639, 1168, 925, 1137, 1137, 1242,
            1168, 448, 919, 510, 1168, 1135, 985, 1243, 951, 1130, 1247, 863, 1010, 822, 1123, 786,
            870, 940, 1123, 1254, 1134, 975, 1254, 1257, 1134, 975, 1134, 892, 613, 577, 557, 100,
            173, 776, 1225, 577, 613, 1195, 577, 1225, 586, 1195, 757, 791, 1195, 1225, 793, 791,
            1225, 757, 1195, 791, 488, 1273, 1211, 757, 791, 1227, 1174, 757, 1227, 179, 1174,
            1227, 791, 299, 1227, 414, 179, 1227, 414, 1227, 299, 414, 299, 415, 826, 722, 868,
            416, 281, 409,
        ];
        let mut mapper = HashMap::new();
        for x in indices.iter() {
            if !mapper.contains_key(x) {
                mapper.insert(*x, mapper.len());
            }
        }
        let mapped_indices = indices
            .iter()
            .map(|i| mapper.get(i).unwrap().clone())
            .collect::<Vec<_>>();
        println!("{:?}", mapped_indices);
    }
}
