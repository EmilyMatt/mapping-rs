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
use num_traits::{AsPrimitive, ConstOne};

use crate::{FusedIterator, Vec, array, fmt, marker::PhantomData};

/// An error type containing the various errors that might arise when plotting a bresenham line,
/// when compiling with the `std` feature, it will also derive [`thiserror::Error`].
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "std", derive(thiserror::Error))]
pub enum BresenhamError {
    /// The line was requested in zero dimensions, meaning no primary axis can be selected.
    ZeroDimensions,
    /// At least one of the points' coordinates is NaN or infinite,
    /// meaning the axes cannot be ordered, and the amount of steps cannot be determined.
    NonFiniteCoordinate,
}

impl fmt::Display for BresenhamError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroDimensions => {
                write!(f, "a bresenham line requires at least one dimension")
            }
            Self::NonFiniteCoordinate => {
                write!(f, "the points' coordinates must all be finite")
            }
        }
    }
}

/// A lazy iterator over the [`Point`]s of a bresenham line in `N` dimensions.
///
/// Each [`Iterator::next`] call advances a single step along the line's primary axis,
/// meaning no allocation is performed, and the line may be consumed as it is plotted.
/// See [`plot_bresenham_line`] for a convenience wrapper collecting this into a [`Vec`].
///
/// Construct one using [`BresenhamLine::plotter`].
///
/// # Generics
/// * `F`: the floating type of the input points, either [`prim@f32`] or [`prim@f64`]
/// * `T`: the [`Scalar`] type of the yielded points, typically an integer type
/// * `N`: a usize, representing the dimension to use
#[derive(Clone, Debug)]
pub struct BresenhamLine<F: RealField, T, const N: usize> {
    current: Point<F, N>,
    end: Point<F, N>,
    increments: [F; N],
    steps: [F; N],
    errors: [F; N],
    primary_axis: usize,
    threshold: F,
    remaining: usize,
    _output: PhantomData<fn() -> T>,
}

impl<F: ConstOne + RealField + Copy + AsPrimitive<usize>, T, const N: usize> BresenhamLine<F, T, N>
where
    usize: AsPrimitive<F>,
{
    /// Creates an iterator plotting a bresenham line between the two given points.
    ///
    /// # Arguments
    /// * `start_point`: A [`Point`] of floating type `F` and `N` dimensions, representing the starting point of the line.
    /// * `end_point`: A [`Point`] of floating type `F` and `N` dimensions, representing the ending point of the line.
    ///
    /// # Returns
    /// A [`BresenhamLine`] yielding [`Point`]s with inner type `T`, including the starting point and ending point.
    ///
    /// NOTE: The iterator will always go from the starting point to the ending point, regardless of direction in axis,
    /// and always yields at least one point; should both points fall within the same step,
    /// that single point is the ending point.
    ///
    /// # Errors
    /// * [`BresenhamError::ZeroDimensions`]: if `N` is 0, as no primary axis can be selected.
    /// * [`BresenhamError::NonFiniteCoordinate`]: if any of the points' coordinates is NaN or infinite.
    pub fn plotter(
        start_point: Point<F, N>,
        end_point: Point<F, N>,
    ) -> Result<Self, BresenhamError> {
        if N == 0 {
            return Err(BresenhamError::ZeroDimensions);
        }

        let deltas: [F; N] =
            array::from_fn(|idx| <F as ComplexField>::abs(end_point[idx] - start_point[idx]));

        // A non-finite delta means a coordinate was NaN or infinite; the former cannot be ordered
        // against the other axes, and the latter cannot be expressed as an amount of steps.
        if !deltas.iter().all(<F as ComplexField>::is_finite) {
            return Err(BresenhamError::NonFiniteCoordinate);
        }

        let steps: [F; N] = array::from_fn(|idx| {
            if end_point[idx] > start_point[idx] {
                F::ONE
            } else {
                -F::ONE
            }
        });

        // Deltas are absolute, so zero is a valid starting maximum.
        // comparing with `>=` lets the last of several equal axes win.
        let (primary_axis, primary_delta) = deltas.iter().enumerate().fold(
            (0, F::zero()),
            |(primary_axis, primary_delta), (idx, &delta)| {
                if delta >= primary_delta {
                    (idx, delta)
                } else {
                    (primary_axis, primary_delta)
                }
            },
        );

        let increments: [F; N] = if primary_delta.is_zero() {
            [F::zero(); N]
        } else {
            array::from_fn(|idx| deltas[idx] / primary_delta)
        };

        Ok(Self {
            current: start_point,
            end: end_point,
            increments,
            steps,
            errors: [F::zero(); N],
            primary_axis,
            threshold: F::ONE - (F::ONE / <usize as AsPrimitive<F>>::as_(N + 1)),
            remaining: <F as AsPrimitive<usize>>::as_(primary_delta + F::ONE),
            _output: PhantomData,
        })
    }
}

impl<F: ConstOne + RealField + AsPrimitive<T>, T: Scalar + Copy, const N: usize> Iterator
    for BresenhamLine<F, T, N>
{
    type Item = Point<T, N>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.remaining = self.remaining.checked_sub(1)?;

        if self.remaining == 0 {
            return Some(self.end.map(|e| e.as_()));
        }

        let output = self.current.map(|e| e.as_());
        for axis in 0..N {
            if axis == self.primary_axis {
                continue;
            }

            self.errors[axis] += self.increments[axis];
            if self.errors[axis] >= self.threshold {
                self.current[axis] += self.steps[axis];
                self.errors[axis] -= F::ONE;
            }
        }

        self.current[self.primary_axis] += self.steps[self.primary_axis];

        Some(output)
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl<F: ConstOne + RealField + AsPrimitive<T>, T: Scalar + Copy, const N: usize> ExactSizeIterator
    for BresenhamLine<F, T, N>
{
}

impl<F: ConstOne + RealField + AsPrimitive<T>, T: Scalar + Copy, const N: usize> FusedIterator
    for BresenhamLine<F, T, N>
{
}

/// This is a free-form version of the bresenham line-drawing algorithm,
/// allowing for any input, any output, and N dimensions, under the constraints of the function.
///
/// This collects a [`BresenhamLine`] into a [`Vec`];
/// use [`BresenhamLine::plotter`] directly to plot the line lazily, without allocating.
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
///
/// # Errors
/// * [`BresenhamError::ZeroDimensions`]: if `N` is 0, as no primary axis can be selected.
/// * [`BresenhamError::NonFiniteCoordinate`]: if any of the points' coordinates is NaN or infinite.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Plot Bresenham Line", skip_all)
)]
pub fn plot_bresenham_line<F, T, const N: usize>(
    start_point: Point<F, N>,
    end_point: Point<F, N>,
) -> Result<Vec<Point<T, N>>, BresenhamError>
where
    F: ConstOne + RealField + AsPrimitive<usize> + AsPrimitive<T>,
    usize: AsPrimitive<F>,
    T: Scalar + Copy,
{
    Ok(BresenhamLine::plotter(start_point, end_point)?.collect())
}

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
        let res = plot_bresenham_line(start, end).unwrap();
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
        let res = plot_bresenham_line(start, end).unwrap();
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
        let res = plot_bresenham_line(start, end).unwrap();
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
        let res = plot_bresenham_line(start, end).unwrap();
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
        let res = plot_bresenham_line(start, end).unwrap();
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

        let res: Vec<Point3<usize>> = plot_bresenham_line(start, end).unwrap();
        assert_eq!(res.len(), 1)
    }

    #[test]
    fn test_plotter_size_hint_is_exact() {
        let start = Point3::new(0.0f32, 0.0f32, 0.0f32);
        let end = Point3::new(-3.0f32, -10.0f32, 7.0f32);

        let mut plotter = BresenhamLine::<f32, isize, 3>::plotter(start, end).unwrap();
        let mut expected_remaining = calculate_expected_vec_size(start, end);
        assert_eq!(plotter.len(), expected_remaining);

        while plotter.next().is_some() {
            expected_remaining -= 1;
            assert_eq!(
                plotter.size_hint(),
                (expected_remaining, Some(expected_remaining))
            );
            assert_eq!(plotter.len(), expected_remaining);
        }

        assert_eq!(expected_remaining, 0);
    }

    #[test]
    fn test_plotter_is_fused() {
        let start = Point2::new(0.0f32, 0.0f32);
        let end = Point2::new(2.0f32, 2.0f32);

        let mut plotter = BresenhamLine::<f32, isize, 2>::plotter(start, end).unwrap();
        assert_eq!(plotter.by_ref().count(), 3);

        // Exhausted iterators must keep returning None, rather than wrapping around.
        assert!(plotter.next().is_none());
        assert!(plotter.next().is_none());
        assert_eq!(plotter.len(), 0);
    }

    #[test]
    fn test_zero_dimensions_is_an_error() {
        let point = Point::<f32, 0>::from([]);

        let res = plot_bresenham_line::<f32, isize, 0>(point, point);
        assert_eq!(res.unwrap_err(), BresenhamError::ZeroDimensions);

        let res = BresenhamLine::<f32, isize, 0>::plotter(point, point);
        assert_eq!(res.unwrap_err(), BresenhamError::ZeroDimensions);
    }

    #[test]
    fn test_non_finite_coordinates_are_an_error() {
        let start = Point2::new(0.0f32, 0.0f32);

        for end in [
            Point2::new(f32::NAN, 0.0f32),
            Point2::new(0.0f32, f32::NAN),
            Point2::new(f32::INFINITY, 0.0f32),
            Point2::new(f32::NEG_INFINITY, 0.0f32),
            // Both coordinates non-finite, making the delta itself NaN rather than infinite.
            Point2::new(f32::INFINITY, f32::INFINITY),
        ] {
            let res = plot_bresenham_line::<f32, isize, 2>(start, end);
            assert_eq!(res.unwrap_err(), BresenhamError::NonFiniteCoordinate);

            let res = BresenhamLine::<f32, isize, 2>::plotter(start, end);
            assert_eq!(res.unwrap_err(), BresenhamError::NonFiniteCoordinate);

            // The starting point is checked just the same, as the deltas span both points.
            let res = plot_bresenham_line::<f32, isize, 2>(end, start);
            assert_eq!(res.unwrap_err(), BresenhamError::NonFiniteCoordinate);

            let res = BresenhamLine::<f32, isize, 2>::plotter(end, start);
            assert_eq!(res.unwrap_err(), BresenhamError::NonFiniteCoordinate);
        }
    }

    #[test]
    fn test_zero_delta_line() {
        let point = Point3::new(512.0f32, 512.0f32, 512.0f32);

        let plotter = BresenhamLine::<f32, isize, 3>::plotter(point, point).unwrap();

        // Identical points make every delta zero; without the `is_zero` guard the increments
        // would each be computed as `0.0 / 0.0`, leaving the iterator full of NaNs.
        assert_eq!(plotter.increments, [0.0f32; 3]);
        assert_eq!(plotter.len(), 1);

        // A zero-length line still yields exactly its single point.
        let res: Vec<Point3<isize>> = plot_bresenham_line(point, point).unwrap();
        assert_eq!(res, Vec::from([Point3::new(512, 512, 512)]));
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_error_is_a_std_error() {
        fn assert_error<E: std::error::Error>(_: E) {}

        assert_error(BresenhamError::ZeroDimensions);
        assert_eq!(
            BresenhamError::ZeroDimensions.to_string(),
            "a bresenham line requires at least one dimension"
        );
    }
}
