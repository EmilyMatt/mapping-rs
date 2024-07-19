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

use nalgebra::{Point, Scalar};
use num_traits::Zero;

use crate::{types::IsNan, Ordering, Vec};

fn validate_input<T: Scalar + PartialOrd + IsNan, const N: usize>(input: &[Point<T, N>]) -> bool {
    !(N.is_zero() || input.iter().any(|a| a.coords.iter().any(|b| b.is_nan())))
}

fn lex_sort_func<T: Scalar + PartialOrd + IsNan, const N: usize>(
    a: &Point<T, N>,
    b: &Point<T, N>,
) -> Ordering {
    (0..N).fold(Ordering::Equal, |ord, i| {
        ord.then_with(|| a.coords[i].partial_cmp(&b.coords[i]).unwrap())
    })
}

/// Sorts a point cloud in lexicographical order.
/// Modifies the inserted slice in place.
/// # Arguments
/// * `input`: a mutable slice of [`Point`], representing the point cloud.
///
/// # Returns
/// a [`bool`], indicating if the operation was successful.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Lexicographical Sort In Place", skip_all)
)]
pub fn lex_sort_in_place<T: Scalar + PartialOrd + IsNan, const N: usize>(
    input: &mut [Point<T, N>],
) -> bool {
    if !validate_input(input) {
        return false;
    }

    input.sort_by(lex_sort_func);
    true
}

/// Sorts a copy of the point cloud in lexicographical order.
/// # Arguments
/// * `input`: a slice of [`Point`], representing the point cloud.
///
/// # Returns
/// [`Some`] containing a vector of [`Point`]s, if the operation was successful.
/// Otherwise, returns [`None`].
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Lexicographical Sort With Copy", skip_all)
)]
pub fn lex_sort<T: Scalar + PartialOrd + IsNan, const N: usize>(
    input: &[Point<T, N>],
) -> Option<Vec<Point<T, N>>> {
    if !validate_input(input) {
        return None;
    }

    let mut input = input.to_vec();
    input.sort_by(lex_sort_func);
    Some(input)
}

/// Sorts the point cloud in lexicographical order, returning a [`Vec`] of references to the original points, in order..
/// # Arguments
/// * `input`: a slice of [`Point`], representing the point cloud.
///
/// # Returns
/// [`Some`] containing a vector of &[`Point`]s, if the operation was successful.
/// Otherwise, returns [`None`].
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Lexicographical Sort With References", skip_all)
)]
pub fn lex_sort_ref<T: Scalar + PartialOrd + IsNan, const N: usize>(
    input: &[Point<T, N>],
) -> Option<Vec<&Point<T, N>>> {
    if !validate_input(input) {
        return None;
    }

    let mut refs = input.iter().collect::<Vec<_>>();
    refs.sort_by(|&a, &b| lex_sort_func(a, b));
    Some(refs)
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use crate::point_clouds::generate_point_cloud;

    use super::*;

    #[test]
    fn test_lex_sort_in_place() {
        let mut input = Vec::from([
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(1.0, 2.0, 4.0),
            Point3::new(1.0, 2.0, 2.0),
        ]);
        assert!(lex_sort_in_place(&mut input));
        assert_eq!(
            input,
            Vec::from([
                Point3::new(1.0, 2.0, 2.0),
                Point3::new(1.0, 2.0, 3.0),
                Point3::new(1.0, 2.0, 4.0)
            ])
        );
    }

    #[test]
    fn test_lex_sort_in_place_nan() {
        let mut input = Vec::from([
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(1.0, 2.0, 4.0),
            Point3::new(1.0, 2.0, f32::NAN),
        ]);
        assert!(!lex_sort_in_place(&mut input));
    }

    #[test]
    fn test_lex_sort() {
        let input = Vec::from([
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(1.0, 2.0, 4.0),
            Point3::new(1.0, 2.0, 2.0),
        ]);
        assert_eq!(
            lex_sort(&input),
            Some(Vec::from([
                Point3::new(1.0, 2.0, 2.0),
                Point3::new(1.0, 2.0, 3.0),
                Point3::new(1.0, 2.0, 4.0)
            ]))
        );
    }

    #[test]
    fn ensure_starting_point() {
        let input = generate_point_cloud(100, [-10.0..=10.0, -10.0..=10.0]);
        let sorted = lex_sort(&input).unwrap();

        assert_eq!(
            sorted
                .iter()
                .fold(f64::INFINITY, |acc, it| { acc.min(it.x) }),
            sorted[0].x
        );
    }

    #[test]
    fn test_lex_sort_nan() {
        let input = Vec::from([
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(1.0, 2.0, 4.0),
            Point3::new(1.0, 2.0, f32::NAN),
        ]);
        assert_eq!(lex_sort(&input), None);
    }

    // Test coplanar points
    #[test]
    fn test_lex_sort_coplanar() {
        let input = Vec::from([
            Point3::new(1.0, 4.0, 3.0),
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(1.0, 2.0, 1.0),
        ]);
        assert_eq!(
            lex_sort(&input),
            Some(Vec::from([
                Point3::new(1.0, 2.0, 1.0),
                Point3::new(1.0, 2.0, 3.0),
                Point3::new(1.0, 4.0, 3.0)
            ]))
        );
    }
}
