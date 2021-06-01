use logos::Logos;
#[derive(Logos, Debug, Clone, PartialEq)]
pub enum Token {
    #[error]
    Error,
    #[regex(r"[ \t\n\f]+", logos::skip)]
    Whitespace,

    #[token("$")]
    End,

    #[regex(r"#[^\n]+\n", logos::skip)]
    Comment,

    // Directive of scene-wide options
    // ----------------------------------------------------------------
    #[token("LookAt")]
    KwLookAt,
    #[token("Camera")]
    KwCamera,
    #[token("Integrator")]
    KwIntegrator,
    #[token("Accelerator")]
    KwAccelerator,
    #[token("Sampler")]
    KwSampler,
    #[token("Film")]
    KwFilm,
    #[token("PixelFilter")]
    KwPixelFilter,
    #[token("Filter")]
    KwFilter,

    // Pairs
    // ----------------------------------------------------------------
    #[token("WorldBegin")]
    KwWorldBegin,
    #[token("WorldEnd")]
    KwWorldEnd,

    #[token("AttributeBegin")]
    KwAttributeBegin,
    #[token("AttributeEnd")]
    KwAttributeEnd,

    #[token("TransformBegin")]
    KwTransformBegin,
    #[token("TransformEnd")]
    KwTransformEnd,

    // World item directives
    // ----------------------------------------------------------------
    #[token("LightSource")]
    KwLightSource,
    #[token("AreaLightSource")]
    KwAreaLightSource,
    #[token("Material")]
    KwMaterial,
    #[token("Shape")]
    KwShape,
    #[token("Texture")]
    KwTexture,

    // Transform directives
    // ----------------------------------------------------------------
    #[token("Identity")]
    KwIdentity,
    #[token("Translate")]
    KwTranslate,
    #[token("Scale")]
    KwScale,
    #[token("Rotate")]
    KwRotate,
    #[token("CoordinateSystem")]
    KwCoordinateSystem,
    #[token("CoordSysTransform")]
    KwCoordSysTransform,
    #[token("Transform")]
    KwTransform,
    #[token("ConcatTransform")]
    KwConcatTransform,

    #[token("ReverseOrientation")]
    KwReverseOrientation,

    #[token("MediumInterface")]
    KwMediumInterface,
    #[token("NamedMedium")]
    KwNamedMedium,
    #[token("MakeNamedMedium")]
    KwMakeNamedMedium,
    #[token("NamedMaterial")]
    KwNamedMaterial,
    #[token("MakeNamedMaterial")]
    KwMakeNamedMaterial,

    #[token("ObjectBegin")]
    KwObjectBegin,
    #[token("ObjectEnd")]
    KwObjectEnd,
    #[token("ObjectInstance")]
    KwObjectInstance,

    #[token("[")]
    LBracket,
    #[token("]")]
    RBracket,

    #[regex(r"[\-\+]?\d+(\.\d*)?", |str| str.slice().parse())]
    #[regex(r"[\-\+]?\.\d+", |str| str.slice().parse())]
    Float(f32),
    #[regex("\"[^\"\n]+\"", |str| str.slice().trim_matches('\"').to_owned())]
    QuotedString(String),
}

impl Token {
    /// Returns `true` if the token is in the FIRST-set of a Transform non-terminal
    /// and `false` otherwise.
    pub fn starts_transform(&self) -> bool {
        matches!(
            self,
            Token::KwIdentity
                | Token::KwTranslate
                | Token::KwScale
                | Token::KwRotate
                | Token::KwLookAt
                | Token::KwTransform
                | Token::KwConcatTransform
                | Token::KwCoordSysTransform
                | Token::KwCoordinateSystem
        )
    }

    pub fn starts_scene_option(&self) -> bool {
        matches!(
            self,
            Self::KwCamera
                | Self::KwSampler
                | Self::KwFilm
                | Self::KwFilter
                | Self::KwIntegrator
                | Self::KwAccelerator
                | Self::KwLookAt
        ) || self.starts_transform()
    }

    pub fn starts_world_item(&self) -> bool {
        matches!(
            self,
            Self::KwAttributeBegin
                | Self::KwObjectBegin
                | Self::KwTransformBegin
                | Self::KwShape
                | Self::KwLightSource
                | Self::KwAreaLightSource
                | Self::KwMaterial
                | Self::KwTexture
                | Self::KwMakeNamedMaterial
                | Self::KwNamedMaterial
                | Self::KwObjectInstance
                | Self::KwNamedMedium
                | Self::KwMakeNamedMedium
        ) || self.starts_transform()
    }

    pub fn take(&mut self) -> Self {
        let mut t = Self::Error;
        std::mem::swap(&mut t, self);
        t
    }
}
