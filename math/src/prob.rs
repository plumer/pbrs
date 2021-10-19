/// Represents probability mass (discrete pr) or probability density (continuous pr).
/// - `density()` and `mass()` returns the corresponding variant value or 0 if the `Prob` value is
///   not the variant.
#[derive(Debug, Clone, Copy)]
pub enum Prob {
    Density(f32),
    Mass(f32),
}

pub struct Density(f32);
pub struct Mass(f32);

impl Prob {
    pub fn is_density(&self) -> bool {
        matches!(self, Self::Density(_))
    }
    pub fn is_positive(&self) -> bool {
        match self {
            Self::Density(x) => *x > 0.0,
            Self::Mass(x) => *x > 0.0,
        }
    }
    pub fn density(&self) -> f32 {
        if let Self::Density(pdf) = &self {
            *pdf
        } else {
            0.0
        }
    }
    pub fn mass(&self) -> f32 {
        if let Self::Mass(pmf) = self {
            *pmf
        } else {
            0.0
        }
    }
}