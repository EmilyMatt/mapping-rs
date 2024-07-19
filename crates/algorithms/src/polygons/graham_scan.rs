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

use nalgebra::{ComplexField, Point2, Scalar};
use num_traits::{AsPrimitive, Float, NumOps};

use crate::{point_clouds::lex_sort_ref, types::IsNan, ToOwned, Vec, VecDeque};

fn calculate_determinant<O: ComplexField + Copy, T: Scalar + NumOps + AsPrimitive<O>>(
    point_a: &Point2<T>,
    point_b: &Point2<T>,
    point_c: &Point2<T>,
) -> O {
    T::as_(
        ((point_b.y - point_a.y) * (point_c.x - point_b.x))
            - ((point_b.x - point_a.x) * (point_c.y - point_b.y)),
    )
}

fn check_hull_segment<
    'a,
    O: ComplexField + Copy + PartialOrd,
    T: AsPrimitive<O> + Default + NumOps + Scalar,
>(
    mut accumulator: VecDeque<&'a Point2<T>>,
    current_point: &&'a Point2<T>,
) -> VecDeque<&'a Point2<T>> {
    while accumulator.len() > 1
        && calculate_determinant(
            accumulator[accumulator.len() - 2],
            accumulator[accumulator.len() - 1],
            current_point,
        ) <= O::zero()
    {
        accumulator.pop_back();
    }

    accumulator.push_back(current_point);
    accumulator
}

/// Computes the convex hull of a set of points using the Graham scan algorithm.
/// Specifically the Monotone Chain variant
///
/// # Arguments
/// * `points` - A slice of points to compute the convex hull of
///
/// # Generics
/// * `O` - The output type of the trigonometric functions, essentially the precision of the calculations
/// * `T` - The type of the points, can be of any scalar type
///
/// # Returns
/// An [`Option`] of [`Vec<Point2<T>>`] representing the convex hull, or [`None`] if there were not enough points to compute a convex hull, or if all points are collinear
pub fn graham_scan<
    O: ComplexField + Float,
    T: AsPrimitive<O> + Default + IsNan + NumOps + PartialOrd + Scalar,
>(
    points: &[Point2<T>],
) -> Option<Vec<Point2<T>>> {
    if points.len() < 3 {
        return None;
    }

    let points_sorted = lex_sort_ref(points)?;

    let upper_hull = points_sorted
        .iter()
        .fold(VecDeque::new(), check_hull_segment);
    let upper_hull_len = upper_hull.len();

    let lower_hull = points_sorted
        .iter()
        .rev()
        .fold(VecDeque::new(), check_hull_segment);
    let lower_hull_len = lower_hull.len();

    ((upper_hull_len + lower_hull_len - 2) > 2).then(|| {
        upper_hull
            .into_iter()
            .take(upper_hull_len - 1)
            .chain(lower_hull.into_iter().take(lower_hull_len - 1))
            .map(ToOwned::to_owned)
            .collect::<Vec<_>>()
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_graham_scan() {
        let point_cloud = Vec::from([
            Point2::new(-9.792094, 6.501087),
            Point2::new(-9.297297, 1.4398155),
            Point2::new(-3.0243487, 1.593832),
            Point2::new(-8.812742, 4.631695),
            Point2::new(-1.0105534, 6.7442665),
            Point2::new(5.054867, 5.4078026),
            Point2::new(-6.2726927, -6.1140366),
            Point2::new(3.9031277, 1.8494358),
            Point2::new(9.026024, 4.158765),
            Point2::new(3.3359728, -5.591101),
            Point2::new(-1.2357492, 6.9803257),
            Point2::new(0.83374596, 4.5202436),
            Point2::new(-9.786989, -2.6681538),
            Point2::new(8.778954, -0.79308414),
            Point2::new(-8.8654585, -0.54637814),
            Point2::new(-8.697586, -5.999858),
            Point2::new(6.7509384, 3.09805),
            Point2::new(3.8945904, -2.1427813),
            Point2::new(-6.2516804, -0.11557007),
            Point2::new(-0.11794281, -7.1581955),
            Point2::new(-8.771274, 7.540737),
            Point2::new(-7.273285, -2.2619143),
            Point2::new(-8.424244, 2.289342),
            Point2::new(3.288208, -7.215433),
            Point2::new(-8.128621, -9.368102),
            Point2::new(4.178275, 5.9939947),
            Point2::new(-0.8436794, 5.534689),
            Point2::new(6.414747, 2.2250452),
            Point2::new(2.5352774, 5.041601),
            Point2::new(-1.8946352, 3.5172358),
            Point2::new(-0.6175842, -1.1188431),
            Point2::new(-1.4161129, 9.456829),
            Point2::new(-5.09273, -9.197719),
            Point2::new(1.5842638, 4.6569405),
            Point2::new(8.992567, -1.571001),
            Point2::new(8.999609, -8.009958),
            Point2::new(-6.1459584, -2.6052542),
            Point2::new(-4.50453, 9.498695),
            Point2::new(-1.3855286, 1.1417828),
            Point2::new(-4.4411488, -3.7954373),
            Point2::new(0.8548746, 8.935608),
            Point2::new(2.503477, -6.5350723),
            Point2::new(-3.7824507, -9.775844),
            Point2::new(7.757971, 9.683109),
            Point2::new(6.6446037, 3.5783281),
            Point2::new(-3.5842485, 6.8722763),
            Point2::new(3.4604778, 8.430944),
            Point2::new(-5.876791, -6.3074894),
            Point2::new(-2.1599998, 7.8314323),
            Point2::new(6.1809216, -6.801429),
        ]);
        let hull = graham_scan::<f32, f32>(&point_cloud);

        assert!(hull.is_some());
        assert_eq!(
            hull.unwrap(),
            Vec::from([
                Point2::new(-9.792094, 6.501087),
                Point2::new(-8.771274, 7.540737),
                Point2::new(-4.50453, 9.498695),
                Point2::new(7.757971, 9.683109),
                Point2::new(9.026024, 4.158765),
                Point2::new(8.999609, -8.009958),
                Point2::new(-3.7824507, -9.775844),
                Point2::new(-8.128621, -9.368102),
                Point2::new(-9.786989, -2.6681538),
            ])
        );
    }

    #[test]
    fn test_not_enough_points() {
        assert_eq!(graham_scan::<f32, f32>(&[]), None);
        assert_eq!(graham_scan::<f32, f32>(&[Point2::new(1.0, 1.0)]), None);
        assert_eq!(
            graham_scan::<f32, f32>(&[Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)]),
            None
        );
    }

    #[test]
    fn test_collinear_points() {
        let points = Vec::from([
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 2.0),
        ]);
        assert_eq!(graham_scan::<f32, f32>(&points), None);
    }

    #[test]
    fn test_square_points() {
        let points = Vec::from([
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 0.0),
            Point2::new(0.5, 0.5),
        ]);
        let expected = Vec::from([
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 0.0),
        ]);
        assert_eq!(graham_scan::<f32, f32>(&points), Some(expected));
    }
}
