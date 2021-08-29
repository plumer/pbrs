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