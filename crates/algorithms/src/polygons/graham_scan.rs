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

use core::fmt::Debug;
use nalgebra::{ComplexField, Point2};
use num_traits::{AsPrimitive, NumAssign};

use crate::{
    point_clouds::{downsample_point_cloud_voxel, lex_sort},
    types::IsNan,
    ToOwned, Vec, VecDeque,
};

use super::calculate_determinant;

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Build Hull Segment", skip_all)
)]
fn build_hull_segment<
    'a,
    O: ComplexField + Copy + PartialOrd,
    T: AsPrimitive<O> + Debug + IsNan + NumAssign + PartialOrd,
>(
    mut accumulator: VecDeque<&'a Point2<T>>,
    current_point: &'a Point2<T>,
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

/// Computes the convex hull of a set of points using the Graham Scan algorithm.
/// Specifically the Monotone Chain variant
/// This version sorts the points lexicographically before computing the convex hull.
/// A downside of using this algorithm is that it does not handle collinear points well.
///
/// # Arguments
/// * `points` - A slice of points to compute the convex hull of
/// * `assume_sorted` - A boolean indicating whether the points are already sorted lexicographically, this is critical for correctness so make sure you don't set this to `true` unless you are sure the points are sorted
/// * `voxel_size` - An optional parameter specifying the voxel size by which to downsample the point cloud before computing the convex hull, useful in reducing errors due to close or identical vertices.
///
/// # Genericsoc -
/// * `O` - The output type of the trigonometric functions, essentially the precision of the calculations
/// * `T` - The type of the points, can be of any scalar type
///
/// # Returns
/// An [`Option`] of [`Vec<Point2<T>>`] representing the convex hull, or [`None`] if there were not enough points to compute a convex hull, or if all points are collinear
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Construct Convex Hull Using Graham Scan", skip_all)
)]
pub fn graham_scan<O, T>(
    points: &[Point2<T>],
    assume_sorted: bool,
    voxel_size: Option<O>,
) -> Option<Vec<Point2<T>>>
where
    O: AsPrimitive<isize> + ComplexField + Copy + PartialOrd,
    T: AsPrimitive<O> + Debug + IsNan + NumAssign + PartialOrd,
    usize: AsPrimitive<T>,
{
    if points.len() < 3 {
        return None;
    }

    let points_sorted;
    let points_sorted_slice = match (assume_sorted, voxel_size) {
        (true, None) => points,
        (true, Some(voxel_size)) => {
            points_sorted = downsample_point_cloud_voxel(points, voxel_size);
            points_sorted.as_slice()
        }
        (false, None) => {
            points_sorted = lex_sort(points)?;
            points_sorted.as_slice()
        }
        (false, Some(voxel_size)) => {
            let downsampled_points = downsample_point_cloud_voxel(points, voxel_size);
            points_sorted = lex_sort(downsampled_points.as_slice())?;
            points_sorted.as_slice()
        }
    };

    let upper_hull = points_sorted_slice
        .iter()
        .fold(VecDeque::new(), |accumulator, point| {
            build_hull_segment(accumulator, point)
        });
    let upper_hull_len = upper_hull.len();

    let lower_hull = points_sorted_slice
        .iter()
        .rev()
        .fold(VecDeque::new(), |accumulator, point| {
            build_hull_segment(accumulator, point)
        });
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

#[cfg(feature = "pregenerated")]
macro_rules! impl_graham_scan {
    ($t:expr, $prec:expr, doc $doc:tt) => {
        ::paste::paste! {

            #[doc = "A premade variant of the Graham Scan algorithm function, made for " $doc " precision floating-point arithmetic, using " $t " as the point type."]
            pub fn [<graham_scan_ $t>](input: &[Point2<$t>], assume_sorted: bool, voxel_size: Option<$prec>) -> Option<Vec<Point2<$t>>> {
                super::graham_scan::<$prec, $t>(input, assume_sorted, voxel_size)
            }
        }
    };
    ($prec:expr, doc $doc:tt) => {
        ::paste::paste! {
            pub(super) mod [<$doc _precision>] {
                use nalgebra::Point2;
                use crate::Vec;

                impl_graham_scan!(u8, $prec, doc $doc);
                impl_graham_scan!(u16, $prec, doc $doc);
                impl_graham_scan!(u32, $prec, doc $doc);
                impl_graham_scan!(u64, $prec, doc $doc);
                impl_graham_scan!(usize, $prec, doc $doc);

                impl_graham_scan!(i8, $prec, doc $doc);
                impl_graham_scan!(i16, $prec, doc $doc);
                impl_graham_scan!(i32, $prec, doc $doc);
                impl_graham_scan!(i64, $prec, doc $doc);
                impl_graham_scan!(isize, $prec, doc $doc);

                impl_graham_scan!(f32, $prec, doc $doc);
                impl_graham_scan!(f64, $prec, doc $doc);
            }
        }
    };
}

impl_graham_scan!(f32, doc single);
impl_graham_scan!(f64, doc double);

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
        let hull = graham_scan::<f32, f32>(&point_cloud, false, None);

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
        assert_eq!(graham_scan::<f32, f32>(&[], false, None), None);
        assert_eq!(
            graham_scan::<f32, f32>(&[Point2::new(1.0, 1.0)], false, None),
            None
        );
        assert_eq!(
            graham_scan::<f32, f32>(&[Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)], false, None),
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
        assert_eq!(graham_scan::<f32, f32>(&points, false, None), None);
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
        assert_eq!(
            graham_scan::<f32, f32>(&points, false, None),
            Some(expected)
        );
    }

    #[test]
    fn test_concave_shape() {
        let points = Vec::from([
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 2.0),
            Point2::new(0.0, 2.0),
        ]);
        let expected = Vec::from([
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 2.0),
            Point2::new(2.0, 2.0),
            Point2::new(2.0, 0.0),
        ]);
        assert_eq!(
            graham_scan::<f32, f32>(&points, false, None),
            Some(expected)
        );
    }

    #[test]
    fn test_duplicate_points() {
        let points = Vec::from([
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 0.0),
        ]);
        let expected = Vec::from([
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 0.0),
        ]);
        assert_eq!(
            graham_scan::<f32, f32>(&points, false, None),
            Some(expected)
        );
    }

    #[test]
    fn test_negative_coordinates() {
        let points = Vec::from([
            Point2::new(-1.0, -1.0),
            Point2::new(-1.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, -1.0),
            Point2::new(0.0, 0.0),
        ]);
        let expected = Vec::from([
            Point2::new(-1.0, -1.0),
            Point2::new(-1.0, 1.0),
            Point2::new(1.0, 1.0),
            Point2::new(1.0, -1.0),
        ]);
        assert_eq!(
            graham_scan::<f32, f32>(&points, false, None),
            Some(expected)
        );
    }

    #[test]
    fn test_complex_shape() {
        let points = Vec::from([
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 2.0),
            Point2::new(0.0, 2.0),
            Point2::new(1.0, 3.0),
            Point2::new(3.0, 1.0),
        ]);

        // point 2, 2 is not included in the hull, due to it being on the same line as 1, 3
        // This is the downside of using a Graham Scan algorithm
        let expected = Vec::from([
            Point2::new(0.0, 0.0),
            Point2::new(0.0, 2.0),
            Point2::new(1.0, 3.0),
            Point2::new(3.0, 1.0),
            Point2::new(2.0, 0.0),
        ]);
        assert_eq!(
            graham_scan::<f32, f32>(&points, false, None),
            Some(expected)
        );
    }
}
