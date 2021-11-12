use math::hcm::{Point3, Vec3};
use math::Angle;
use std::collections::HashMap;
use std::fmt;

#[derive(Debug, Clone)]
pub enum ArgValue {
    String(String),
    Numbers(Vec<f32>),
    Number(f32),
}

#[derive(Debug, Clone)]
pub struct ParameterSet(pub HashMap<String, ArgValue>);

impl ParameterSet {
    pub fn lookup_f32(&self, key: &str) -> Option<f32> {
        let num = self.0.get(key)?;
        match num {
            ArgValue::Numbers(x) => x.first(),
            ArgValue::Number(x) => Some(x),
            _ => None,
        }
        .map(|x| *x)
    }

    pub fn lookup_string(&self, key: &str) -> Option<String> {
        let string_parameter = self.0.get(key)?;
        match string_parameter {
            ArgValue::String(s) => Some(s.clone()),
            _ => None,
        }
    }

    pub fn extract_string(&mut self, key: &str) -> Option<String> {
        let string = self.0.remove(key)?;
        match string {
            ArgValue::String(s) => Some(s),
            _ => None,
        }
    }

    pub fn extract(&mut self, key: &str) -> Option<ArgValue> {
        self.0.remove(key)
    }

    /// Finds a key-value pair in the parameter set where the key contains `pattern` as a substring,
    /// delimited by spaces.
    /// Returns the fully matched key and its value.
    pub fn extract_substr(&mut self, pattern: &str) -> Option<(String, ArgValue)> {
        let full_key = self
            .0
            .keys()
            .find(|k| k.split(' ').any(|part| part == pattern))?
            .clone();
        let value = self
            .0
            .remove(&full_key)
            .expect("the value should exist after a key is found");
        Some((full_key, value))
    }
}
#[derive(Debug, Clone)]
pub enum Transform {
    Identity,
    Translate(Vec3),
    Scale(Vec3),
    Rotate(Vec3, Angle),
    LookAt(Point3, Point3, Vec3),
    // CoordSys,
    // Matrix4x4(Mat4),
    // CoordSysTransform,
    // ConcatMatrix4x4(Mat4),
}

#[derive(Debug)]
pub enum SceneWideOption {
    Camera(String, ParameterSet),
    Film(String, ParameterSet),
    Filter(String, ParameterSet),
    Integrator(String, ParameterSet),
    Accel(String, ParameterSet),
    Transform(Transform),
    Sampler(String, ParameterSet),
}

#[allow(dead_code)]
#[derive(Clone)]
pub enum WorldItem {
    Transform(Transform),
    Shape(String, ParameterSet),
    Material(String, ParameterSet),
    Light(String, ParameterSet),
    AreaLight(String, ParameterSet),
    Media(String, ParameterSet),
    /// implmentation, type, name
    Texture(String, String, String, ParameterSet),

    AttributeBlock(Vec<WorldItem>),
    ObjectBlock(String, Vec<WorldItem>),
    TransformBlock(Vec<WorldItem>),
    MakeMedium(String, ParameterSet),
    MakeMaterial(String, ParameterSet),

    MediumInstance(String),
    MaterialInstance(String),
    ObjectInstance(String),

    ReverseOrientation,
}

pub struct Scene {
    pub options: Vec<SceneWideOption>,
    pub items: Vec<WorldItem>,
}

// Formatters
// --------------------------------------------------------------------
impl fmt::Display for ArgValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Numbers(numbers) => write!(f, "{:?}", numbers),
            Self::String(s) => write!(f, "{}", s),
            Self::Number(x) => write!(f, "{}", x),
        }
    }
}

impl fmt::Display for ParameterSet {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (key, parameter) in self.0.iter() {
            match parameter {
                ArgValue::Numbers(numbers) => write!(f, "    {} = {:?}\n", key, numbers)?,
                ArgValue::String(s) => write!(f, "    {} = {}\n", key, s)?,
                ArgValue::Number(n) => write!(f, "    {} = {}\n", key, n)?,
            }
        }
        write!(f, "")
    }
}

impl fmt::Display for Transform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Transform::Identity => write!(f, "Identity"),
            Transform::Translate(t) => write!(f, " Translate({})", t),
            Transform::Scale(s) => write!(f, " Scale({})", s),
            Transform::Rotate(axis, angle) => write!(f, "Rotate({}, {})", axis, angle),
            Transform::LookAt(e, t, u) => write!(f, "LookAt({} -> {} ^ {})", e, t, u),
        }
    }
}

impl fmt::Display for Scene {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Scene-wide options:\n")?;
        for option in self.options.iter() {
            write!(f, "  {}\n", option)?;
        }
        write!(f, "World items:\n")?;
        for item in self.items.iter() {
            write!(f, "  {}\n", item)?;
        }
        write!(f, "\n")
    }
}

impl fmt::Display for WorldItem {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Transform(t) => write!(f, "Trans: {}", t),
            Self::Shape(r#impl, param_set) => {
                write!(f, "Shape of {}, parameters: \n{}", r#impl, param_set)
            }
            Self::Material(r#impl, param_set) => {
                write!(f, "Material of {}, parameters: \n{}", r#impl, param_set)
            }
            Self::Light(r#impl, param_set) => {
                write!(f, "Light of {}, parameters: \n{}", r#impl, param_set)
            }
            Self::AreaLight(r#impl, param_set) => {
                write!(f, "AreaLight of {}, parameters: \n{}", r#impl, param_set)
            }
            Self::Media(r#impl, param_set) => {
                write!(f, "Media of {}, parameters: \n{}", r#impl, param_set)
            }
            Self::Texture(r#impl, tex_type, tex_name, param_set) => {
                write!(
                    f,
                    "Texture of {}, type {}, named {}, parameters: \n{}",
                    r#impl, tex_type, tex_name, param_set
                )
            }
            Self::MakeMedium(r#impl, param_set) => {
                write!(f, "MakeMedium of {}, parameters: \n{}", r#impl, param_set)
            }
            Self::MakeMaterial(r#impl, param_set) => {
                write!(f, "MakeMaterial of {}, parameters: \n{}", r#impl, param_set)
            }
            Self::MediumInstance(name) => write!(f, "medium instance named {}", name),
            Self::ObjectInstance(name) => write!(f, "object instance named {}", name),
            Self::MaterialInstance(name) => write!(f, "material instance named {}", name),
            Self::ReverseOrientation => write!(f, "reverse_orientation"),
            Self::AttributeBlock(items) | Self::TransformBlock(items) => {
                write!(f, "attribute block \n")?;
                for item in items.iter() {
                    write!(f, "    {}\n", item)?;
                }
                write!(f, "")
            }
            Self::ObjectBlock(name, items) => {
                write!(f, "object (name = {}) block \n", name)?;
                for item in items.iter() {
                    write!(f, "    {}\n", item)?;
                }
                write!(f, "")
            }
        }
    }
}

impl fmt::Display for SceneWideOption {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Camera(implementation, param_set) => {
                write!(
                    f,
                    "Camera of {}, parameters:\n{}",
                    implementation, param_set
                )
            }
            Self::Film(implementation, param_set) => {
                write!(f, "Film of {}, parameters:\n{}", implementation, param_set)
            }
            Self::Filter(implementation, param_set) => {
                write!(
                    f,
                    "Filter of {}, parameters:\n{}",
                    implementation, param_set
                )
            }
            Self::Integrator(implementation, param_set) => {
                write!(
                    f,
                    "Integrator of {}, parameters:\n{}",
                    implementation, param_set
                )
            }
            Self::Accel(implementation, param_set) => {
                write!(f, "Accel of {}, parameters:\n{}", implementation, param_set)
            }
            Self::Transform(t) => write!(f, "Transform({})", t),
            Self::Sampler(implementation, param_set) => {
                write!(
                    f,
                    "Sampler of {}, parameters:\n{}",
                    implementation, param_set
                )
            }
        }
    }
}
