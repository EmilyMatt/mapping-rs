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
use num_traits::{Bounded, NumOps};

use crate::utils::distance_squared;

/// Finds the closest matching target point to the passed source point.
///
/// # Arguments
/// * `point`: A [`Point`], for which to find the closest point.
/// * `all_points`: A slice of [`Point`], representing the target point cloud.
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, representing the number of dimensions in the points.
///
/// # Returns
/// An [`Option`] of [`Point`], representing said closest point, empty if no points were provided.
#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Find Closest Points", skip_all)
)]
pub fn find_nearest_neighbour_naive<T, const N: usize>(
    point: &Point<T, N>,
    all_points: &[Point<T, N>],
) -> Option<Point<T, N>>
where
    T: Bounded + Copy + Default + NumOps + PartialOrd + Scalar,
{
    if all_points.is_empty() {
        return None;
    }

    let mut current_distance = T::max_value();
    let mut current_point = all_points[0]; // Guaranteed to exist

    for target_point in all_points.iter() {
        let distance = distance_squared(point, target_point);
        if distance < current_distance {
            current_distance = distance;
            current_point = *target_point;
        }
    }

    Some(current_point)
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point, Point2};

    use crate::Vec;

    use super::*;

    #[test]
    fn test_find_closest_point() {
        // Given:
        // A set of target points
        let target_points = Vec::from([
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 2.0),
            Point2::new(5.0, 5.0),
            Point2::new(8.0, 8.0),
        ]);

        // A transformed point
        let transformed_point = Point2::new(4.0, 4.0);

        // When:
        // Finding the closest point
        let closest_point = find_nearest_neighbour_naive(&transformed_point, &target_points);

        // Expect the closest point to be (5.0, 5.0)
        assert_eq!(closest_point, Some(Point2::new(5.0, 5.0)));
    }

    #[test]
    fn test_find_closest_point_with_empty_target() {
        // Given:
        // An empty set of target points
        let target_points: Vec<Point<f64, 2>> = Vec::new();

        // A transformed point
        let transformed_point = Point2::new(4.0, 4.0);

        // This should panic as the target_points array is empty
        assert_eq!(
            find_nearest_neighbour_naive(&transformed_point, &target_points),
            None
        );
    }
}
