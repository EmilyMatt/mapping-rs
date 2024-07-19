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

use nalgebra::{ComplexField, Point, RealField, Scalar};
use num_traits::AsPrimitive;

use crate::{array, Vec};

/// This is a free-form version of the bresenham line-drawing algorithm,
/// allowing for any input, any output, and N dimensions, under the constraints of the function.
///
/// # Arguments
/// * `start_point`: A [`Point`] of floating type `F` and `N` dimensions, representing the starting point of the line.
/// * `end_point`: A [`Point`] of floating type `F` and `N` dimensions, representing the ending point of the line.
///
/// # Generics
/// * F: either [`prim@f32`] or [`prim@f64`]
/// * N: a usize, representing the dimension to use
///
/// # Returns
/// A [`Vec`] of [`Point`]s with inner type `T`, representing the drawn line, including the starting point and ending point.
///
/// NOTE: The returned [`Vec`] will always go from the starting point to the ending point, regardless of direction in axis.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Plot Bresenham Line", skip_all)
)]
pub fn plot_bresenham_line<F, T, const N: usize>(
    start_point: Point<F, N>,
    end_point: Point<F, N>,
) -> Vec<Point<T, N>>
where
    F: RealField + AsPrimitive<usize> + AsPrimitive<T>,
    usize: AsPrimitive<F>,
    T: Scalar + Copy,
{
    let deltas: [F; N] =
        array::from_fn(|idx| <F as ComplexField>::abs(end_point[idx] - start_point[idx]));
    let steps: [F; N] = array::from_fn(|idx| {
        if end_point[idx] > start_point[idx] {
            F::one()
        } else {
            -F::one()
        }
    });
    let primary_axis = deltas
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .unwrap()
        .0;

    let mut current_point = start_point;
    let mut errors = Vec::from([F::zero(); N]);
    let mut points = Vec::with_capacity(<F as AsPrimitive<usize>>::as_(
        deltas[primary_axis] + F::one(),
    ));
    while <F as ComplexField>::abs(current_point[primary_axis] - end_point[primary_axis])
        >= F::one()
    {
        points.push(current_point.map(|element| element.as_()));

        for axis in 0..N {
            if axis == primary_axis {
                continue;
            }

            errors[axis] += deltas[axis] / deltas[primary_axis];

            if errors[axis] >= F::one() - (F::one() / (N + 1).as_()) {
                current_point[axis] += steps[axis];
                errors[axis] -= F::one();
            }
        }

        current_point[primary_axis] += steps[primary_axis];
    }

    points.push(end_point.map(|element| element.as_()));
    points
}

#[cfg(feature = "pregenerated")]
macro_rules! impl_bresenham_algorithm {
    ($precision:expr, doc $doc:tt, $nd:expr, $out:expr) => {
        ::paste::paste! {
            #[doc = "A premade variant of the bresenham line function for " $doc "-precision floating-point arithmetic, returns a [`Vec`] of [`Point`]s with inner type " $out "."]
            pub fn [<plot_$nd d_$out _bresenham_line>](start_point: Point<$precision, $nd>, end_point: Point<$precision, $nd>) -> Vec<Point<$out, $nd>> {
                    super::plot_bresenham_line::<$precision, $out, $nd>(start_point, end_point)
            }
        }
    };

    ($prec:expr, doc $doc:tt, $nd:expr) => {
        impl_bresenham_algorithm!($prec, doc $doc, $nd, i32);
        impl_bresenham_algorithm!($prec, doc $doc, $nd, i64);
        impl_bresenham_algorithm!($prec, doc $doc, $nd, isize);

        impl_bresenham_algorithm!($prec, doc $doc, $nd, u32);
        impl_bresenham_algorithm!($prec, doc $doc, $nd, u64);
        impl_bresenham_algorithm!($prec, doc $doc, $nd, usize);

        impl_bresenham_algorithm!($prec, doc $doc, $nd, f32);
        impl_bresenham_algorithm!($prec, doc $doc, $nd, f64);
    };

    ($prec:expr, doc $doc:tt) => {
        ::paste::paste! {
            pub(super) mod [<$doc _precision>] {
                use nalgebra::Point;
                use crate::Vec;

                impl_bresenham_algorithm!($prec, doc $doc, 2);
                impl_bresenham_algorithm!($prec, doc $doc, 3);
            }
        }
    }
}

#[cfg(feature = "pregenerated")]
impl_bresenham_algorithm!(f32, doc single);
#[cfg(feature = "pregenerated")]
impl_bresenham_algorithm!(f64, doc double);

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point2, Point3};

    fn calculate_expected_vec_size<const N: usize>(
        start: Point<f32, N>,
        end: Point<f32, N>,
    ) -> usize {
        let arr: [f32; N] = array::from_fn(|idx| f32::abs(end[idx] - start[idx]));
        arr.into_iter().max_by(|a, b| a.total_cmp(b)).unwrap() as usize + 1
    }

    #[test]
    fn test_plot_bresenham_line_2d_nonsteep_pos() {
        let start = Point2::new(0.0f32, 0.0f32);
        let end = Point2::new(10.0f32, 3.0f32);
        let res = single_precision::plot_2d_isize_bresenham_line(start, end);
        assert_eq!(
            res,
            Vec::<Point2<isize>>::from([
                Point2::new(0, 0),
                Point2::new(1, 0),
                Point2::new(2, 0),
                Point2::new(3, 1),
                Point2::new(4, 1),
                Point2::new(5, 1),
                Point2::new(6, 2),
                Point2::new(7, 2),
                Point2::new(8, 2),
                Point2::new(9, 3),
                Point2::new(10, 3),
            ])
        );
    }

    #[test]
    fn test_plot_bresenham_line_2d_steep_pos() {
        let start = Point2::new(0.0f32, 0.0f32);
        let end = Point2::new(3.0f32, 10.0f32);
        let res = single_precision::plot_2d_isize_bresenham_line(start, end);
        assert_eq!(res.len(), calculate_expected_vec_size(start, end));
        assert_eq!(
            res,
            Vec::<Point2<isize>>::from([
                Point2::new(0, 0),
                Point2::new(0, 1),
                Point2::new(0, 2),
                Point2::new(1, 3),
                Point2::new(1, 4),
                Point2::new(1, 5),
                Point2::new(2, 6),
                Point2::new(2, 7),
                Point2::new(2, 8),
                Point2::new(3, 9),
                Point2::new(3, 10),
            ])
        );
    }

    #[test]
    fn test_plot_bresenham_line_2d_nonsteep_neg() {
        let start = Point2::new(0.0f32, 0.0f32);
        let end = Point2::new(-10.0f32, -3.0f32);
        let res = single_precision::plot_2d_isize_bresenham_line(start, end);
        assert_eq!(res.len(), calculate_expected_vec_size(start, end));
        assert_eq!(
            res,
            Vec::<Point2<isize>>::from([
                Point2::new(0, 0),
                Point2::new(-1, 0),
                Point2::new(-2, 0),
                Point2::new(-3, -1),
                Point2::new(-4, -1),
                Point2::new(-5, -1),
                Point2::new(-6, -2),
                Point2::new(-7, -2),
                Point2::new(-8, -2),
                Point2::new(-9, -3),
                Point2::new(-10, -3),
            ])
        );
    }

    #[test]
    fn test_plot_bresenham_line_2d_steep_neg() {
        let start = Point2::new(0.0f32, 0.0f32);
        let end = Point2::new(-3.0f32, -10.0f32);
        let res = single_precision::plot_2d_isize_bresenham_line(start, end);
        assert_eq!(res.len(), calculate_expected_vec_size(start, end));
        assert_eq!(
            res,
            Vec::<Point2<isize>>::from([
                Point2::new(0, 0),
                Point2::new(0, -1),
                Point2::new(0, -2),
                Point2::new(-1, -3),
                Point2::new(-1, -4),
                Point2::new(-1, -5),
                Point2::new(-2, -6),
                Point2::new(-2, -7),
                Point2::new(-2, -8),
                Point2::new(-3, -9),
                Point2::new(-3, -10),
            ])
        );
    }

    #[test]
    fn test_plot_bresenham_line_3d_x() {
        let start = Point3::new(0.0f32, 0.0f32, 0.0f32);
        let end = Point3::new(-3.0f32, -10.0f32, 7.0f32);
        let res = single_precision::plot_3d_isize_bresenham_line(start, end);
        assert_eq!(res.len(), calculate_expected_vec_size(start, end));
        assert_eq!(
            res,
            Vec::<Point3<isize>>::from([
                Point3::new(0, 0, 0),
                Point3::new(0, -1, 0),
                Point3::new(0, -2, 1),
                Point3::new(-1, -3, 2),
                Point3::new(-1, -4, 3),
                Point3::new(-1, -5, 3),
                Point3::new(-2, -6, 4),
                Point3::new(-2, -7, 5),
                Point3::new(-2, -8, 5),
                Point3::new(-2, -9, 6),
                Point3::new(-3, -10, 7),
            ])
        );
    }

    #[test]
    fn test_small_deltas() {
        let start = Point3::new(512.0, 512.0, 512.0);
        let end = Point3::new(512.5, 511.294, 512.1);

        let res: Vec<Point3<usize>> = plot_bresenham_line(start, end);
        assert_eq!(res.len(), 1)
    }
}
