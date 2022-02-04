use std::ops::Range;

use crate::float::{Fallback, Float, Inside, Interval};
use crate::prob::Prob;
use itertools::Itertools;

/// Cubic spline modeling 1D function.
///
/// - Build a `CubicSpline` from samples using `from_samples()`,
/// - `evaluate()` the spline at any x-value.
pub struct CubicSpline {
    m: Vec<f32>,
    xs: Vec<f32>,
    ys: Vec<f32>,
}

impl CubicSpline {
    /// Builds a cubic spline from a set of data points (pairs of (x, y) values). The x-values from
    /// the input should be already sorted. The spline is built with the boundary condition of
    /// having a zero 2nd-order derivative at both ends.
    pub fn from_samples(xs_and_ys: &[(f32, f32)]) -> Self {
        let mut m = vec![0.0f32];
        m.append(&mut cubic_spline_zero_hess(xs_and_ys));
        m.push(0.0);
        let mut xs = Vec::new();
        let mut ys = Vec::new();

        xs_and_ys.iter().cloned().for_each(|(x, y)| {
            xs.push(x);
            ys.push(y);
        });

        assert_eq!(m.len(), xs.len());
        assert_eq!(m.len(), ys.len());
        Self { m, xs, ys }
    }

    /// Evaluates the spline at any x-value. If x is outside the domain of the spline, then the
    /// y-values at the corresponding extermal is returned.
    pub fn evaluate(&self, at: f32) -> f32 {
        let i1 = self.xs.partition_point(|&x| x < at);
        if i1 <= 0 {
            self.ys[0]
        } else if i1 >= self.ys.len() {
            *self.ys.last().unwrap()
        } else {
            let x0 = self.xs[i1 - 1];
            let x1 = self.xs[i1];
            let (y0, y1) = (self.ys[i1 - 1], self.ys[i1]);
            let (m0, m1) = (self.m[i1 - 1], self.m[i1]);
            assert!(at.inside((x0, x1)));

            let h = x1 - x0;
            let frac_1_6h = 1.0 / (6.0 * h);
            0.0 + m0 * (x1 - at).powi(3) * frac_1_6h
                + m1 * (at - x0).powi(3) * frac_1_6h
                + (y0 - m0 * h * h / 6.0) * (x1 - at) / h
                + (y1 - m1 * h * h / 6.0) * (at - x0) / h
        }
    }
}

/// Computes the coefficient of a cubic spline given the set of x-y data points.
///
/// For data points x0, x1, x2 ... xn, an corresponding values of y, computes the coefficient `M`
/// values to construct a piece-wise function that is continuous up to the 2nd derivative, and goes
/// through the given data points. The n+1 `M` coefficients are used to specify each one of n pieces
/// with the formula:
/// ``` ignore
///                    3                       3   
/// M_i   (x_{i+1} - x)    M_{i+1}    (x - x_i)    
/// --- * -------------- + ------- * ----------- +
///  6    x_{i+1} - x_i       6      x_{i+1}-x_i   
///
///               2                                        2
///              h     x_{i+1} - x                        h      x - x_i
/// (y_i - M_i * --) * ----------- + (y_{i+1} - M_{i+1} * --) * ---------
///              6     x_{i+1}-x_i                        6    x_{i+1}-x_i
/// ```
/// The resulting cubic spline has zero 2nd derivatives at x=x0 and x=xn. As a result, M0 = 0 and
/// Mn = 0. These values are implicit, and the returned M vector would be of length 2 less than the
/// input vector `xs_and_ys`.
pub fn cubic_spline_zero_hess(xs_and_ys: &[(f32, f32)]) -> Vec<f32> {
    let dx_and_dydx = xs_and_ys
        .iter()
        .tuple_windows::<(_, _)>()
        .map(|((x0, y0), (x1, y1))| (x1 - x0, (y1 - y0) / (x1 - x0)))
        .collect::<Vec<_>>();
    let x_windows3 = xs_and_ys
        .iter()
        .map(|(x, _y)| x)
        .tuple_windows::<(_, _, _)>();

    let mut mus_1n_1 = Vec::new();
    let mut lambdas_1n_1 = Vec::new();
    let mut ds_1n_1 = Vec::new();
    dx_and_dydx.iter().tuple_windows().zip(x_windows3).for_each(
        |(((dx0, dy0), (dx1, dy1)), (x_1, _x0, x1))| {
            mus_1n_1.push(dx0 / (dx0 + dx1));
            lambdas_1n_1.push(1.0 - dx0 / (dx0 + dx1));
            ds_1n_1.push(6.0 * (dy1 - dy0) / (x1 - x_1));
        },
    );
    let diag = vec![2.0; ds_1n_1.len()];
    // Sets boundary conditions as: M0 = Mn = 0, so d1 and d_{n-1} are not changed.
    tridiagonal(&mus_1n_1, &diag, &lambdas_1n_1, &ds_1n_1)
}

/// Solves a linear system of size N, where the matrix on the LHS is tri-diagonal.
/// ``` ignore
/// a1 | b1   c1                             |
///    | a2   b2   c2                        |
///    |      a3   b3   c3                   |
///    |        ..      ..     ..            |
///    |             a_{n-1} b_{n-1} c_{n-1} |
///    |                     an      bn      | cn
/// ```
/// Input `b` should contain values on the main diagonal, and `a` for the diagonal below the main
/// diagonal, and `c` above the main diagonal.
///
/// For the convenience of indexing, the input vectors `a`, `b` and `c` all have the same length
/// (which is the side-length of the matrix), but a_1 and c_n will be ignored.
fn tridiagonal(a: &[f32], b: &[f32], c: &[f32], rhs: &[f32]) -> Vec<f32> {
    let mut betas = Vec::new();
    betas.push(c[0] / b[0]);
    for i in 1..a.len() - 1 {
        let beta = c[i] / (b[i] - betas[i - 1] * a[i]);
        betas.push(beta);
    }

    let mut ys = Vec::new();
    ys.push(rhs[0] / b[0]);
    for i in 1..a.len() {
        let y = (rhs[i] - a[i] * ys[i - 1]) / (b[i] - a[i] * betas[i - 1]);
        ys.push(y);
    }

    let mut xs = ys.clone(); // implicitly setting x.last to y.last.
    assert_eq!((0..3).rev().collect::<Vec<i32>>(), vec![2, 1, 0]);
    for i in (0..a.len() - 1).rev() {
        xs[i] = ys[i] - betas[i] * xs[i + 1];
    }
    assert_eq!(betas.len(), b.len() - 1);
    xs
}

/// Computes a interval of length 1 such that predicate(start) holds but predicate(end) doesn't.
/// The computed range always falls in the range `[0, size)`, or end <= size.
///
/// It is assumed such that `predicate()` evaluates to `true` for all x <= start and `false` for
/// all x >= end. If predicate is true for all or false for all, the computed interval is clamped
/// to `[0, size)`.
pub fn find_interval<Predicate>(size: usize, predicate: Predicate) -> Range<usize>
where
    Predicate: Fn(usize) -> bool,
{
    let mut first = 0;
    let mut len = size;
    while len > 0 {
        let half = len >> 1;
        let middle = first + half;
        if predicate(middle) {
            first = middle + 1;
            len -= half + 1;
        } else {
            len = half;
        }
    }
    let left = (first.max(1) - 1).min(size - 2);
    if left > 0 {
        assert!(predicate(left));
    }
    if left < size - 2 {
        assert!(!predicate(left + 1));
    }
    left..left + 1
}

/// Computes weights for Catmull-Rom-interpolating function values on the given nodes.
///
/// Given (x, y) value pairs `x1, x2, ... xn` and `y1, y2, ... yn`, and x-values being sorted, there
/// exists a piece-wise cubic function that interpolates all y-values for any given x-value in
/// `(x1, xn)`:
///
/// `f(x) = w_{-1}y_{-1} + w0y0 + w1y1 + w2y2`
///
/// where weights (w-values) can be computed from this function.
///
/// - `nodes`: a slice of *sorted* x-values.
/// - Returns weights `w_{-1} .. w_2` together with the index to the -1-th element in the array.
/// `x` value is guaranteed to be in `nodes[i+0]` and `nodes[i+1]`. `i-1` and `i+2` may be out of
/// bounds, and in those case, `w_{-1}` and `w_2` will be zero, correspondingly.
pub fn catmull_rom_weights(nodes: &[f32], x: f32) -> Option<(isize, [f32; 4])> {
    assert!(nodes.len() >= 3);
    if x < nodes[0] || x > *nodes.last().unwrap() {
        return None;
    }
    // let index = nodes.partition_point(|&v| v < x);
    let range = find_interval(nodes.len(), |i| nodes[i] <= x);
    // let (x0, x1) = (nodes[index - 1], nodes[index]);
    // let (il, i0, i1, ir) = (index as isize - 2, index - 1, index, index + 1);
    let (i0, i1) = (range.start, range.end);
    let (il, ir) = (range.start as isize - 1, range.end + 1);
    let (x0, x1) = (nodes[i0], nodes[i1]);
    assert!(x.inside((x0, x1)), "{} should be inside [{x0}, {x1}]", x);

    let t = (x - x0) / (x1 - x0);
    let (t2, t3) = (t * t, t * t * t);
    let mut weights = [
        0.0,
        2.0 * t3 - 3.0 * t2 + 1.0,
        -2.0 * t3 + 3.0 * t2, // w2 - w0; w0 to be determined.
        0.0,
    ];
    // Computes first node weight.
    if il >= 0 {
        let w0 = (t3 - 2.0 * t2 + t) * (x1 - x0) / (x1 - nodes[il as usize]);
        weights[0] = -w0;
        weights[2] += w0;
    } else {
        let w0 = t3 - 2.0 * t2 + t;
        weights[0] = 0.0;
        weights[1] -= w0;
        weights[2] += w0;
    }

    if ir < nodes.len() {
        let w3 = (t3 - t2) * (x1 - x0) / (nodes[ir] - x0);
        weights[1] -= w3;
        weights[3] = w3;
    } else {
        let w3 = t3 - t2;
        weights[1] -= w3;
        weights[2] += w3;
        weights[3] = 0.0;
    }
    Some((il, weights))
}

pub fn sample_catmull_rom_2d(
    nodes_v: &[f32], nodes_h: &[f32], values: &[f32], cdf: &[f32], alpha: f32, u: f32,
) -> Option<(f32, f32, Prob)> {
    assert_eq!(nodes_v.len() * nodes_h.len(), cdf.len());
    assert_eq!(nodes_v.len() * nodes_h.len(), values.len());
    let (offset, weights) = catmull_rom_weights(nodes_v, alpha)?;

    // Defines a closure to interpolate table entries.
    let interpolate = |array2d: &[f32], col: usize| {
        (0..4isize)
            .map(|i| match weights[i as usize] == 0.0 {
                false => array2d[(offset + i) as usize * nodes_h.len() + col] * weights[i as usize],
                true => 0.0,
            })
            .sum::<f32>()
    };

    let maximum = interpolate(cdf, nodes_h.len() - 1);
    let u = u * maximum;

    let index = find_interval(nodes_h.len(), |i| interpolate(cdf, i) <= u).start;
    let f0 = interpolate(values, index);
    let f1 = interpolate(values, index + 1);
    let x0 = nodes_h[index];
    let x1 = nodes_h[index + 1];
    let width = x1 - x0;
    // Re-scale _u_ using the interpolated cdf.
    let u = (u - interpolate(cdf, index)) / width;

    let d0 = match index > 0 {
        true => width * (f1 - interpolate(values, index - 1)) / (x1 - nodes_h[index - 1]),
        false => f1 - f0,
    };
    let d1 = match index + 2 < nodes_h.len() {
        true => width * (interpolate(values, index + 2) - f0) / (nodes_h[index + 2] - x0),
        false => f1 - f0,
    };

    // Inverts definite integral over spline segment and returns solution.
    let mut t = match f0 - f1 {
        zero if zero == 0.0 => u / f0,
        diff => (f0 - (f0 * f0 + 2.0 * u * -diff).max(0.0).sqrt()) / diff,
    };
    let mut interval = Interval::new(0.0, 1.0);
    let f = loop {
        t = t.filter_or(|t| interval.contains(t), interval.midpoint());
        let integral_hat = t.polynomial([
            0.0,
            f0,
            0.5 * d0,
            1.0 / 3.0 * (-2.0 * d0 - d1) + f1 - f0,
            0.25 * (d0 + d1) + 0.5 * (f0 - f1),
        ]);
        let fhat = t.polynomial([
            f0,
            d0,
            -2.0 * d0 - d1 + 3.0 * (f1 - f0),
            d0 + d1 + 2.0 * (f0 - f1),
        ]);

        if (integral_hat - u).abs() < 1e-6 || interval.length() < 1e-6 {
            break fhat;
        }
        interval = match integral_hat - u < 0.0 {
            true => Interval::new(t, interval.max()),
            false => Interval::new(interval.min(), t),
        };
        t -= (integral_hat - u) / fhat;
    };
    Some((f, x0 + width * t, Prob::Density(f / maximum)))
}

mod test {
    #[test]
    fn tridiagonal_test() {
        let a = [1.0; 7];
        let b = [1.5; 7];
        let c = [1.0; 7];
        let mut rhs = [7.0; 7];
        rhs[0] = 5.0;
        rhs[6] = 5.0;

        let expected_x = [2.0; 7];
        let actual_x = super::tridiagonal(&a, &b, &c, &rhs);

        let total_square_error: f32 = actual_x
            .iter()
            .zip(expected_x.iter())
            .map(|(x0, x1)| (x0 - x1).powi(2))
            .sum();
        crate::assert_le!(total_square_error, 1e-6);
    }

    #[test]
    fn cubic_spline_solve_test() {
        let x = [0.2, 0.4, 0.6, 0.8, 1.0f32];
        let y = [0.97986, 0.91777, 0.80803, 0.63860, 0.38437f32];
        let pairs = x.iter().cloned().zip(y.iter().cloned()).collect::<Vec<_>>();

        let actual_m = super::cubic_spline_zero_hess(&pairs);
        let expected_m = vec![-1.5021, -1.1390, -2.8952];
        let total_error_squared: f32 = (0..expected_m.len())
            .map(|i| (expected_m[i] - actual_m[i]).powi(2))
            .sum();
        crate::assert_le!(total_error_squared, 1e-6);
    }

    #[test]
    fn cubic_spline_eval_test() {
        use crate::assert_le;
        use crate::float::Float;
        let x = [0.2, 0.4, 0.6, 0.8, 1.0f32];
        let y = [0.97986, 0.91777, 0.80803, 0.63860, 0.38437f32];
        let pairs = x.iter().cloned().zip(y.iter().cloned()).collect::<Vec<_>>();

        let spline = super::CubicSpline::from_samples(&pairs);
        assert_le!(spline.evaluate(0.3).dist_to(0.9526), 3e-5);
        assert_le!(spline.evaluate(0.5).dist_to(0.8695), 3e-5);
        assert_le!(spline.evaluate(0.7).dist_to(0.7334), 3e-5);
        assert_le!(spline.evaluate(0.9).dist_to(0.5187), 3e-5);
    }

    #[test]
    fn test_find_interval() {
        use crate::{assert_ge, assert_gt, assert_le, assert_lt};
        let array = [16, 21, 32, 43, 55, 62, 73, 82];
        for pivot in (0..10).map(|x| x * 10 + 5) {
            let std::ops::Range { start, end } =
                super::find_interval(array.len(), |x| array[x] < pivot);
            if pivot < array[0] {
                assert_eq!(start, 0);
            } else if pivot > *array.last().unwrap() {
                assert_eq!(end, array.len() - 1);
            } else {
                assert_lt!(array[start], pivot);
                assert_ge!(array[end], pivot);
                assert_eq!(start + 1, end);
            }

            let std::ops::Range { start, end } =
                super::find_interval(array.len(), |x| array[x] <= pivot);
            if pivot < array[0] {
                assert_eq!(start, 0);
            } else if pivot > *array.last().unwrap() {
                assert_eq!(end, array.len() - 1);
            } else {
                assert_le!(array[start], pivot);
                assert_gt!(array[end], pivot);
                assert_eq!(start + 1, end);
            }
        }
    }

    #[test]
    fn catmull_test() {
        use crate::float::Float;
        #[rustfmt::skip]
        let values = [
            -1.0, -0.9992586, -0.99751526, -0.9947773, -0.9910477, -0.98633015,
            -0.98062944, -0.97395116, -0.9663021, -0.95768976, -0.94812274, -0.9376106,
            -0.9261639, -0.913794, -0.9005131, -0.88633466, -0.8712726, -0.85534215,
            -0.838559, -0.82093996, -0.80250263, -0.7832653, -0.7632472, -0.7424683,
            -0.7209493, -0.69871163, -0.67577744, -0.65216964, -0.62791175, -0.60302794,
            -0.577543, -0.5514824, -0.52487206, -0.49773848, -0.47010878, -0.44201043,
            -0.4134715, -0.3845204, -0.35518602, -0.32549757, -0.29548463, -0.2651772,
            -0.23460539, -0.20379974, -0.17279093, -0.14160988, -0.110287674, -0.07885553,
            -0.04734478, -0.01578684, 0.0, 0.0, 0.01578684, 0.04734478,
            0.07885553, 0.110287674, 0.14160988, 0.17279093, 0.20379974, 0.23460539,
            0.2651772, 0.29548463, 0.32549757, 0.35518602, 0.3845204, 0.4134715,
            0.44201043, 0.47010878, 0.49773848, 0.52487206, 0.5514824, 0.577543,
            0.60302794, 0.62791175, 0.65216964, 0.67577744, 0.69871163, 0.7209493,
            0.7424683, 0.7632472, 0.7832653, 0.80250263, 0.82093996, 0.838559,
            0.85534215, 0.8712726, 0.88633466, 0.9005131, 0.913794, 0.9261639,
            0.9376106, 0.94812274, 0.95768976, 0.9663021, 0.97395116, 0.98062944,
            0.98633015, 0.9910477, 0.9947773, 0.99751526, 0.9992586, 1.0,
        ];

        let mut inputs = crate::float::linspace((-1.1f32, 1.1), 30).0;
        inputs.push((values[0] + values[1]) / 2.0);
        for &x in inputs.iter() {
            if let Some((il, weights)) = super::catmull_rom_weights(&values, x) {
                let weight_sum = weights.iter().sum::<f32>();
                assert!(
                    weight_sum.dist_to(1.0) < 1e-6,
                    "weights = {:?}, summing to {weight_sum}",
                    weights,
                );
                let (i0, i1) = ((il + 1) as usize, (il + 2) as usize);
                assert!(x >= values[i0]);
                assert!(x < values[i1]);
            } else {
                println!("x = {}", x);
            }
        }
    }
}
