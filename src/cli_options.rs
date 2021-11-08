use std::collections::HashMap;

pub enum Integrator {
    Direct,
    Path,
}

impl Integrator {
    pub fn from_str(s: &str) -> Option<Self> {
        match s {
            "direct" => Some(Self::Direct),
            "path" => Some(Self::Path),
            _ => None,
        }
    }

    pub fn to_str(&self) -> &'static str {
        match self {
            &Self::Direct => "direct",
            &Self::Path => "path",
        }
    }
}

pub struct CliOptions {
    pub use_multi_thread: bool,
    pub scene_name: Option<String>,
    pub pbrt_file: Option<String>,
    pub integrator: Integrator,
}

impl Default for CliOptions {
    fn default() -> Self {
        Self {
            use_multi_thread: true,
            scene_name: None,
            pbrt_file: None,
            integrator: Integrator::Path,
        }
    }
}

impl CliOptions {
    pub fn message() -> &'static str {
        r#"
        --use_multi_thread | --use_single_thread
        --scene_name <scene_name>
        --pbrt_file <file.pbrt>
        --integrator <direct|path>
        "#
    }
}

pub fn parse_args(args: Vec<String>) -> Result<CliOptions, String> {
    let mut pairs: HashMap<String, Option<String>> = HashMap::new();
    let mut args = args.into_iter().rev().collect::<Vec<_>>();
    args.pop(); // Removes args[0]

    while let Some(key) = args.pop() {
        if !key.starts_with('-') {
            return Err(format!("Unrecognized key {}", key));
        }
        match args.last() {
            None => {
                pairs.insert(key, None);
            }
            Some(value) => {
                if value.starts_with('-') {
                    pairs.insert(key, None);
                } else {
                    let value = args.pop();
                    pairs.insert(key, value);
                }
            }
        }
    }
    let mut options = CliOptions::default();
    for (k, v) in pairs.into_iter() {
        match k.as_str() {
            "--use_multi_thread" => options.use_multi_thread = true,
            "--use_single_thread" => options.use_multi_thread = false,
            "--scene_name" => options.scene_name = v,
            "--pbrt_file" => options.pbrt_file = v,
            "--help" => {
                println!("usage: {}", CliOptions::message());
                std::process::exit(0);
            }
            "--integrator" => {
                options.integrator = v
                    .map(|s| Integrator::from_str(&s))
                    .flatten()
                    .expect("unsupported")
            }
            _ => return Err(format!("Unrecognized key {}", k)),
        }
    }
    Ok(options)
}
