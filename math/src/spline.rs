use crate::float::Inside;
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
}
