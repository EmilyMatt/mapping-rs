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

use crate::{types::IsNan, VecDeque};
use nalgebra::{ComplexField, Dyn, Point, Scalar, SquareMatrix, VecStorage};
use num_traits::{AsPrimitive, NumAssignOps, NumOps};

// Hopefully the compiler does all of this in compile time TBH
fn calculate_determinant<
    O: ComplexField + Copy,
    T: Copy + Scalar + PartialOrd + IsNan + NumOps + NumAssignOps + AsPrimitive<O>,
    const N: usize,
>(
    points: [&Point<T, N>; 3],
) -> O {
    assert!(N >= 2);
    SquareMatrix::from_vec_storage(VecStorage::new(
        Dyn(N + 1),
        Dyn(N + 1),
        (0..=N).fold(vec![O::one(); (N + 1).pow(2)], |mut acc, idx| {
            let row = idx / (N + 1);
            let col = idx % (N + 1);
            acc[row * (N + 1) + col] = match (row, col) {
                (row, col) if row < 3 && col < N => points[row].coords[col].as_(),
                _ => O::one(),
            };

            acc
        }),
    ))
    .determinant()
}

pub fn graham_scan<
    O: ComplexField + Copy + PartialOrd,
    T: Copy + Scalar + PartialOrd + IsNan + NumOps + NumAssignOps + AsPrimitive<O>,
    const N: usize,
>(
    points: &[Point<T, N>],
) -> Option<Vec<Point<T, N>>> {
    let sorted = crate::point_clouds::lex_sort(points)?;

    let upper_hull = sorted.iter().fold(VecDeque::new(), |mut acc, it| {
        while acc.len() >= 2
            && calculate_determinant([acc[acc.len() - 2], acc[acc.len() - 1], it]) > O::zero()
        {
            acc.pop_back();
        }

        // Push the new point
        acc.push_back(it);
        acc
    });

    let lower_hull = sorted.iter().rev().fold(VecDeque::new(), |mut acc, it| {
        while acc.len() >= 2
            && calculate_determinant([acc[acc.len() - 2], acc[acc.len() - 1], it]) > O::zero()
        {
            acc.pop_back();
        }

        // Push the new point
        acc.push_back(it);
        acc
    });

    Some(
        upper_hull
            .into_iter()
            .skip(1)
            .chain(lower_hull.into_iter().skip(1))
            .map(ToOwned::to_owned)
            .collect::<Vec<_>>(),
    )
}

#[cfg(test)]
mod tests {
    use crate::convex_hulls::graham_scan::graham_scan;
    use nalgebra::Point2;

    #[test]
    fn test_graham_scan() {
        let a = graham_scan::<f32, f32, 2>(&[
            Point2::new(1.0, 2.0),
            Point2::new(-1.0, 3.0),
            Point2::new(-0.5, -4.0),
            Point2::new(0.0, 0.0),
            Point2::new(-5.0, 1.5),
        ]);

        panic!("{a:?}");
    }
}
