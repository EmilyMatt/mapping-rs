// SPDX-License-Identifier: MIT
/*
 * Copyright (c) [2023 - Present] Emily Matheys <emilymatt96@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

use nalgebra::{Const, DimMin, Point, RealField, SMatrix, Scalar};
use num_traits::NumOps;

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Distance Squared", skip_all, level = "trace")
)]
pub(crate) fn distance_squared<T, const N: usize>(point_a: &Point<T, N>, point_b: &Point<T, N>) -> T
where
    T: Copy + Default + NumOps + Scalar,
{
    point_a
        .iter()
        .zip(point_b.iter())
        .map(|(&x, &y)| {
            let diff = x - y;
            diff * diff
        })
        .fold(T::default(), |acc, x| acc + x)
}

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Verify Matrix Determinant", skip_all, level = "info")
)]
pub(crate) fn verify_rotation_matrix_determinant<T, const N: usize>(
    mut u: SMatrix<T, N, N>,
    v_t: SMatrix<T, N, N>,
) -> SMatrix<T, N, N>
where
    T: Copy + RealField,
    Const<N>: DimMin<Const<N>, Output = Const<N>>,
{
    if (u * v_t).determinant() < T::zero() {
        u.column_mut(N - 1)
            .iter_mut()
            .for_each(|element| *element *= T::one().neg()); // Reverse the last column
    }
    u * v_t
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use nalgebra::{Matrix2, Point3};

    #[test]
    fn test_distance_squared() {
        let point_a = Point3::new(1.0, 2.0, 3.0);
        let point_b = Point3::new(4.0, 5.0, 6.0);
        assert_eq!(distance_squared(&point_a, &point_b), 27.0)
    }

    #[test]
    fn test_verify_rotation_matrix_determinant() {
        let mat_a = Matrix2::new(2.0, 3.0, 2.0, 1.0);
        let mat_b = Matrix2::new(-1.0, 0.0, 0.0, -1.0);

        let regular_dot = mat_a * mat_b;
        assert!(regular_dot.determinant() < 0.0);

        let func_dot = verify_rotation_matrix_determinant(mat_a, mat_b);

        // Verify second column is actually reversed by this function
        assert!(func_dot.determinant() >= 0.0);
        assert_eq!(func_dot.m12, -regular_dot.m12);
        assert_eq!(func_dot.m22, -regular_dot.m22);
    }
}
