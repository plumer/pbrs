use std::{
    collections::HashSet,
    io::{BufRead, Read},
};

use log::{error, info, warn};
use math::hcm;
use shape::{TriangleMeshRaw, Vertex};

pub fn load_ply(path: &str) -> TriangleMeshRaw {
    return load_ply_self_housed(path).unwrap();
}

fn get_type_size(name: &str) -> Option<usize> {
    match name {
        "uchar" | "uint8" => Some(1),
        "short" => Some(2),
        "int" | "uint" => Some(4),
        _ => None,
    }
}

#[derive(Clone, Copy)]
enum PlyFormat {
    Ascii,
    BinaryLE,
    BinaryBE,
}

fn bytes_to_f32(bytes: &[u8], ply_format: PlyFormat) -> f32 {
    if bytes.len() != std::mem::size_of::<f32>() {
        error!("Length of a u8 for f32 isn't 4: {}", bytes.len());
    }
    let b = [bytes[0], bytes[1], bytes[2], bytes[3]];
    match ply_format {
        PlyFormat::BinaryBE => f32::from_be_bytes(b),
        PlyFormat::BinaryLE => f32::from_le_bytes(b),
        PlyFormat::Ascii => panic!("Not supported ascii"),
    }
}
fn bytes_to_uint(bytes: &[u8], ply_format: PlyFormat) -> u32 {
    if bytes.len() == std::mem::size_of::<u8>() {
        bytes[0] as u32
    } else if bytes.len() == std::mem::size_of::<u16>() {
        let b = [bytes[0], bytes[1]];
        (match ply_format {
            PlyFormat::Ascii => panic!("Not supported"),
            PlyFormat::BinaryBE => u16::from_be_bytes(b),
            PlyFormat::BinaryLE => u16::from_le_bytes(b),
        }) as u32
    } else if bytes.len() == std::mem::size_of::<u32>() {
        let b = [bytes[0], bytes[1], bytes[2], bytes[3]];
        match ply_format {
            PlyFormat::Ascii => panic!("Not supported"),
            PlyFormat::BinaryBE => u32::from_be_bytes(b),
            PlyFormat::BinaryLE => u32::from_le_bytes(b),
        }
    } else {
        panic!("unsupported byte length {}", bytes.len());
    }
}

macro_rules! wrong_data_error {
    ($($arg:tt)*) => {
        std::io::Result::Err(std::io::Error::new(std::io::ErrorKind::InvalidData, format!($($arg)*)))
    }
}

fn load_ply_self_housed(path: &str) -> std::io::Result<TriangleMeshRaw> {
    let file = std::fs::File::open(path).unwrap();
    let mut reader = std::io::BufReader::new(file);

    // Reads the header.
    let mut header = String::new();
    reader.read_line(&mut header).unwrap();
    if header.trim() != "ply" {
        return wrong_data_error!("Header isn't ply: '{}'", header);
    }
    let mut format_line = String::new();
    reader.read_line(&mut format_line).unwrap();

    let ply_format = {
        let mut words = format_line.split(' ');
        if words.next() != Some("format") {
            return wrong_data_error!("Format line is bad: {}", format_line);
        }
        match words.next() {
            Some("ascii") => PlyFormat::Ascii,
            Some("binary_little_endian") => PlyFormat::BinaryLE,
            Some("binary_big_endian") => PlyFormat::BinaryBE,
            Some(s) => return wrong_data_error!("Unrecognized format string: {}", s),
            None => return wrong_data_error!("No format string"),
        }
    };

    // Reads the lines from the header.
    let mut header_lines = Vec::new();
    loop {
        let mut buffer = String::new();
        reader.read_line(&mut buffer).unwrap();
        let buffer = buffer.trim().to_owned();
        if buffer == "end_header" {
            break;
        } else if !buffer.starts_with("comment") {
            header_lines.push(buffer);
        }
    }

    let mut property_names = Vec::new();
    let mut num_vertices = None;
    let mut num_faces = None;
    let mut list_length_number_size = None;
    let mut list_element_size = None;
    for line in header_lines.iter() {
        let words: Vec<&str> = line.split(' ').collect();
        if words.is_empty() {
            continue;
        } else if words.len() < 3 {
            return wrong_data_error!("Can't handle the line {}", line);
        }
        if let ["element", "vertex", nverts] = words.as_slice() {
            num_vertices = nverts.parse::<usize>().ok();
        } else if let ["element", "face", nfaces] = words.as_slice() {
            num_faces = nfaces.parse::<usize>().ok();
        } else if let ["property", "float", property_name] = words.as_slice() {
            property_names.push(property_name.to_string());
        } else if let ["property", "list", list_length_number_size_str, list_element_size_str, "vertex_indices"] =
            words.as_slice()
        {
            list_length_number_size = get_type_size(list_length_number_size_str);
            list_element_size = get_type_size(list_element_size_str);
        } else {
            warn!("Unprocessed header line: {}", line);
        }
    }

    if let (
        Some(num_vertices),
        Some(num_faces),
        Some(list_length_number_size),
        Some(list_element_size),
    ) = (
        num_vertices,
        num_faces,
        list_length_number_size,
        list_element_size,
    ) {
        log::trace!("num_vertices = {}, num_faces = {}, list_length_number_size = {}, list_element_size = {}",
                num_vertices, num_faces, list_length_number_size, list_element_size);
        let mut vertex_buffer_u8 = Vec::new();
        vertex_buffer_u8.resize(
            num_vertices * property_names.len() * std::mem::size_of::<f32>(),
            0u8,
        );
        reader.read_exact(&mut vertex_buffer_u8).unwrap();

        let vertex_buffer = vertex_buffer_u8
            .chunks_exact(4)
            .map(|bytes| bytes_to_f32(bytes, ply_format))
            .collect::<Vec<_>>();
        // info!("Vertex buffer = {:?}", vertex_buffer);

        let mut index_triples = Vec::new();
        for _ in 0..num_faces {
            let mut list_length_u8 = Vec::new();
            list_length_u8.resize(list_length_number_size * std::mem::size_of::<u8>(), 0u8);
            reader.read_exact(&mut list_length_u8).unwrap();
            let list_length = bytes_to_uint(&list_length_u8, ply_format);
            assert!(matches!(list_element_size, 1 | 2 | 4));

            let mut face_indices_bytes = Vec::new();
            face_indices_bytes.resize(list_element_size * list_length as usize, 0u8);
            reader.read_exact(&mut face_indices_bytes).unwrap();

            let face_indices: Vec<_> = face_indices_bytes
                .chunks_exact(list_element_size)
                .map(|bytes| bytes_to_uint(bytes, ply_format) as i32)
                .collect();
            if let [i, j, k] = face_indices.as_slice() {
                index_triples.push((*i as usize, *j as usize, *k as usize));
            } else {
                for i in 1..(list_length - 1) as usize {
                    index_triples.push((
                        face_indices[0] as usize,
                        face_indices[i] as usize,
                        face_indices[i + 1] as usize,
                    ));
                }
            }
        }

        let stride = property_names.len();
        #[derive(Default)]
        struct Offset {
            px: Option<usize>,
            py: Option<usize>,
            pz: Option<usize>,
            nx: Option<usize>,
            ny: Option<usize>,
            nz: Option<usize>,
            u: Option<usize>,
            v: Option<usize>,
        }
        let mut offset = Offset::default();
        for i in 0..stride {
            match property_names[i].as_str() {
                "x" => offset.px = Some(i),
                "y" => offset.py = Some(i),
                "z" => offset.pz = Some(i),
                "nx" => offset.nx = Some(i),
                "ny" => offset.ny = Some(i),
                "nz" => offset.nz = Some(i),
                "u" => offset.u = Some(i),
                "v" => offset.v = Some(i),
                name => warn!("Unrecognized vertex property named {}", name),
            }
        }
        match (offset.px, offset.py, offset.pz) {
            (Some(_), Some(_), Some(_)) => (),
            _ => {
                return wrong_data_error!(
                    "position xyz: some missing: {:?},{:?},{:?}",
                    offset.px,
                    offset.py,
                    offset.pz,
                )
            }
        }
        let mut positions = Vec::new();
        let mut normals = Vec::new();
        let mut uvs = Vec::new();
        for base in (0..vertex_buffer.len()).step_by(stride) {
            positions.push(hcm::Point3::new(
                vertex_buffer[base + offset.px.unwrap()],
                vertex_buffer[base + offset.py.unwrap()],
                vertex_buffer[base + offset.pz.unwrap()],
            ));
            if let (Some(offset_nx), Some(offset_ny), Some(offset_nz)) =
                (offset.nx, offset.ny, offset.nz)
            {
                normals.push(hcm::Vec3::new(
                    vertex_buffer[base + offset_nx],
                    vertex_buffer[base + offset_ny],
                    vertex_buffer[base + offset_nz],
                ));
            }
            if let (Some(offset_u), Some(offset_v)) = (offset.u, offset.v) {
                uvs.push((
                    vertex_buffer[base + offset_u],
                    vertex_buffer[base + offset_v],
                ));
            }
        }
        if normals.is_empty() {
            normals = geometry::compute_normals(&positions, &index_triples);
        }


// #[allow(dead_code)]
// pub fn load_ply_external_lib(path: &str) -> TriangleMeshRaw {
//     let mut f = std::fs::File::open(path).unwrap();
//     let vertex_name_set = ["x", "y", "z", "nx", "ny", "nz", "u", "v"]
//         .iter()
//         .copied()
//         .collect::<HashSet<&'static str>>();
//     This piece of code is commented out in favor of removing crate `ply-rs`
//     since it introduces a lot of crates.  However the source code is still
//     retained in case the in-house implementation is found to be buggy.
//     // create a parser
//     let p = ply_rs::parser::Parser::<ply_rs::ply::DefaultElement>::new();
//
//     // use the parser: read the entire file
//     let ply_mesh = p.read_ply(&mut f).unwrap();
//
//     let ref vertices_raw = ply_mesh.payload["vertex"];
//     let mut vertex_data = vec![];
//     for vertex_raw in vertices_raw {
//         let mut vertex = Vertex::zero();
//         if let Property::Float(f) = vertex_raw["x"] {
//             vertex.pos.x = f;
//         } else {
//             panic!("property x not formed correctly: {:?}", vertex_raw["x"]);
//         }
//         if let Property::Float(y) = vertex_raw["y"] {
//             vertex.pos.y = y;
//         }
//         if let Property::Float(z) = vertex_raw["z"] {
//             vertex.pos.z = z;
//         }
//         if let Property::Float(nx) = vertex_raw["nx"] {
//             vertex.normal.x = nx;
//         }
//         if let Property::Float(ny) = vertex_raw["ny"] {
//             vertex.normal.y = ny;
//         }
//         if let Property::Float(nz) = vertex_raw["nz"] {
//             vertex.normal.z = nz;
//         }
//         if vertex_raw.contains_key("u") {
//             if let Property::Float(u) = vertex_raw["u"] {
//                 vertex.uv.0 = u;
//             }
//         }
//         if vertex_raw.contains_key("v") {
//             if let Property::Float(v) = vertex_raw["v"] {
//                 vertex.uv.1 = v;
//             }
//         }
//
//         if vertex_raw
//             .keys()
//             .any(|k| vertex_name_set.contains(k.as_str()) == false)
//         {
//             eprintln!("unhandled vertex attribute {:?}", vertex_raw);
//         }
//
//         vertex_data.push(vertex);
//     }
//
//     let mut index_triples = vec![];
//     let ref faces_raw = ply_mesh.payload["face"];
//     for face_raw in faces_raw {
//         if let Property::ListInt(indices) = &face_raw["vertex_indices"] {
//             if let [i, j, k] = indices.as_slice() {
//                 index_triples.push((*i as usize, *j as usize, *k as usize));
//             } else {
//                 eprintln!("face index list not of length 3");
//             }
//         }
//     }
//
//     let points_are_close = |p0: hcm::Point3, p1: hcm::Point3| (p0 - p1).norm_squared() < 1e-6;
//     let vecs_are_close = |v0: hcm::Vec3, v1: hcm::Vec3| (v0 - v1).norm_squared() < 1e-6;
//
//     let self_housed = load_ply_self_housed(path);
//     if let Ok(x) = self_housed {
//         info!("_load_ply ok");
//         assert_eq!(x.index_triples, index_triples, "indices are the same ?");
//         assert_eq!(
//             x.vertices.len(),
//             vertex_data.len(),
//             "vertices are the same length?"
//         );
//         for (id, (vnew, vold)) in x.vertices.iter().zip(vertex_data.iter()).enumerate() {
//             if !points_are_close(vnew.pos, vold.pos) {
//                 error!("differ on #{} by position", id);
//             }
//             if !vecs_are_close(vnew.normal, vold.normal) {
//                 error!("differ on #{} by normal", id);
//             }
//             if vnew.uv.0 != vold.uv.0 || vnew.uv.1 != vold.uv.1 {
//                 error!("differ on #{} by uv", id);
//             }
//         }
//     } else {
//         let e = self_housed.unwrap_err();
//         error!("_load_ply error: {}", e);
//     }
//
//     TriangleMeshRaw {
//         vertices: vertex_data,
//         index_triples,
//     }
// }
