/// Defines useful functions for common math operations, tools and constants:
/// - 1D interval,
/// - Simple interpolation and barycentric interpolation on not only primitive types,
/// - Macros to check if two math quantities are close, less than / greater than (or equal to) each other.
pub mod float;

/// Homogeneous-coordinate maths module. 
/// - Types: 3D points and vectors, 4D vector, 3x3 and 4x4 matrices.
/// - Types: `Degree` and `Radian` to represent angles unambiguously.
/// - Function `normalize()` to build a normalized `Vec3`.
/// - Function `make_coord_system()` to build an orthogonal base from a `Vec3`.
/// - Functions `reflect()` and `refract()` to compute surface interactions.
pub mod hcm;

/// Provides `Prob` struct representing a probability mass or probability density.
pub mod prob;

pub use float::Angle;
pub fn new_rad(rad: f32) -> float::Angle {
    float::Angle::new_rad(rad)
}
pub fn new_deg(deg: f32) -> float::Angle {
    float::Angle::new_deg(deg)
}