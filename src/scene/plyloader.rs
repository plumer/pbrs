use std::collections::HashSet;

use crate::shape::{TriangleMeshRaw, Vertex};
use ply_rs::ply::*;

pub fn load_ply(path: &str) -> TriangleMeshRaw {
    let mut f = std::fs::File::open(path).unwrap();
    let vertex_name_set = ["x", "y", "z", "nx", "ny", "nz", "u", "v"]
        .iter()
        .copied()
        .collect::<HashSet<&'static str>>();

    // create a parser
    let p = ply_rs::parser::Parser::<ply_rs::ply::DefaultElement>::new();

    // use the parser: read the entire file
    let ply_mesh = p.read_ply(&mut f).unwrap();

    // for (name, element_def) in ply_mesh.header.elements {
    //     println!(
    //         "{} elements '{}' with the following properties:",
    //         element_def.count, name
    //     );
    //     for (name, prop) in element_def.properties {
    //         println!("  {}: {:?}", name, prop);
    //         if name == "vertex" && prop.data_type != PropertyType::Scalar(ScalarType::Float) {
    //             panic!("doesn't support data type {:?} yet", prop.data_type);
    //         }
    //         if name == "vertex" && vertex_name_set.contains(name.as_str()) == false {
    //             panic!("not supported vertex property name {}", name);
    //         }
    //     }
    // }

    let ref vertices_raw = ply_mesh.payload["vertex"];
    let mut vertex_data = vec![];
    for vertex_raw in vertices_raw {
        let mut vertex = Vertex::zero();
        if let Property::Float(f) = vertex_raw["x"] {
            vertex.pos.x = f;
        } else {
            panic!("property x not formed correctly: {:?}", vertex_raw["x"]);
        }
        if let Property::Float(y) = vertex_raw["y"] {
            vertex.pos.y = y;
        }
        if let Property::Float(z) = vertex_raw["z"] {
            vertex.pos.z = z;
        }
        if let Property::Float(nx) = vertex_raw["nx"] {
            vertex.normal.x = nx;
        }
        if let Property::Float(ny) = vertex_raw["ny"] {
            vertex.normal.y = ny;
        }
        if let Property::Float(nz) = vertex_raw["nz"] {
            vertex.normal.z = nz;
        }

        if vertex_raw.contains_key("u") {
            if let Property::Float(u) = vertex_raw["u"] {
                vertex.uv.0 = u;
            }
        }
        if vertex_raw.contains_key("v") {
            if let Property::Float(v) = vertex_raw["v"] {
                vertex.uv.1 = v;
            }
        }

        if vertex_raw
            .keys()
            .any(|k| vertex_name_set.contains(k.as_str()) == false)
        {
            eprintln!("unhandled vertex attribute {:?}", vertex_raw);
        }

        vertex_data.push(vertex);
    }

    let mut triangle_indices = vec![];
    let ref faces_raw = ply_mesh.payload["face"];
    for face_raw in faces_raw {
        if let Property::ListInt(indices) = &face_raw["vertex_indices"] {
            triangle_indices.append(&mut indices.clone());
        }
    }

    TriangleMeshRaw {
        vertices: vertex_data,
        indices: triangle_indices,
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
