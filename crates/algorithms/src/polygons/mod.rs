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

pub use graham_scan::graham_scan;
pub use point_in_polygon::{are_multiple_points_in_polygon, is_single_point_in_polygon};

use nalgebra::{Point, RealField};
use num_traits::Bounded;

use crate::{array, ops::RangeInclusive, types::PolygonExtents};

mod graham_scan;
mod point_in_polygon;

#[cfg(feature = "pregenerated")]
#[doc = "This module contains polygon algorithms that are pregenerated for single precision floating points."]
pub mod single_precision {
    pub use super::point_in_polygon::single_precision::{
        are_multiple_points_in_polygon, is_single_point_in_polygon,
    };
}

#[cfg(feature = "pregenerated")]
#[doc = "This module contains polygon algorithms that are pregenerated for double precision floating points."]
pub mod double_precision {
    pub use super::point_in_polygon::double_precision::{
        are_multiple_points_in_polygon, is_single_point_in_polygon,
    };
}

/// This function calculates the extents of the polygon, i.e., the minimum and maximum values for each coordinate dimension.
///
/// # Generics
/// * `T`: one of [`f32`] or [`f64`].
/// * `N`: a constant generic of type [`usize`].
///
/// # Arguments
/// * `polygon`: a slice of [`Point`].
///
/// # Returns
/// See [`PolygonExtents`]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Polygon Extents", skip_all, level = "info")
)]
pub fn calculate_polygon_extents<T, const N: usize>(polygon: &[Point<T, N>]) -> PolygonExtents<T, N>
where
    T: Bounded + Copy + RealField,
{
    let mut extents_accumulator: [RangeInclusive<T>; N] =
        array::from_fn(|_| <T as Bounded>::max_value()..=<T as Bounded>::min_value());

    for vertex in polygon.iter() {
        for (extent_for_dimension, vertex_coord) in
            extents_accumulator.iter_mut().zip(vertex.coords.iter())
        {
            *extent_for_dimension = extent_for_dimension.start().min(*vertex_coord)
                ..=extent_for_dimension.end().max(*vertex_coord);
        }
    }

    extents_accumulator
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point, Point2};

    use crate::Vec;

    use super::*;

    #[test]
    fn test_calculate_polygon_extents() {
        // Given:
        // A set of polygon vertices
        let polygon = Vec::from([
            Point2::new(1.0, 1.0),
            Point2::new(1.0, 4.0),
            Point2::new(5.0, 4.0),
            Point2::new(5.0, 1.0),
        ]);

        // When:
        // Calculating the extents
        let extents = calculate_polygon_extents(&polygon);
        assert_eq!(
            extents,
            [RangeInclusive::new(1.0, 5.0), RangeInclusive::new(1.0, 4.0)]
        );
    }

    #[test]
    fn test_calculate_polygon_extents_empty_polygon() {
        // An empty polygon
        let polygon: Vec<Point<f64, 2>> = Vec::new();

        // Calculating the extents
        let extents = calculate_polygon_extents(&polygon);

        // Expect the extents to be [max_value..=min_value] for x and y respectively
        assert_eq!(
            extents,
            [
                RangeInclusive::new(f64::MAX, f64::MIN),
                RangeInclusive::new(f64::MAX, f64::MIN)
            ]
        );
    }
}
