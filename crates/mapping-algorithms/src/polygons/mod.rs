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
pub use jarvis_march::jarvis_march;
pub use point_in_polygon::{are_multiple_points_in_polygon, is_single_point_in_polygon};

use nalgebra::{ComplexField, Point, Point2, RealField, Scalar};
use num_traits::{AsPrimitive, Bounded, NumOps};

use crate::{array, ops::RangeInclusive, types::PolygonExtents};

mod graham_scan;
mod jarvis_march;
mod point_in_polygon;

#[cfg(feature = "pregenerated")]
#[doc = "This module contains polygon mapping-algorithms that are pregenerated for single precision floating points."]
pub mod single_precision {
    pub use super::graham_scan::single_precision::*;
    pub use super::jarvis_march::single_precision::*;
    pub use super::point_in_polygon::single_precision::*;
}

#[cfg(feature = "pregenerated")]
#[doc = "This module contains polygon mapping-algorithms that are pregenerated for double precision floating points."]
pub mod double_precision {
    pub use super::graham_scan::double_precision::*;
    pub use super::jarvis_march::double_precision::*;
    pub use super::point_in_polygon::double_precision::*;
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
pub fn calculate_polygon_extents<T, const N: usize>(
    polygon: &[Point<T, N>],
) -> Option<PolygonExtents<T, N>>
where
    T: Bounded + Copy + RealField,
{
    let mut extents_accumulator: [RangeInclusive<T>; N] =
        array::from_fn(|_| <T as Bounded>::max_value()..=<T as Bounded>::min_value());

    if polygon.len() < 3 {
        return None;
    }

    for vertex in polygon {
        for (extent_for_dimension, vertex_coord) in
            extents_accumulator.iter_mut().zip(vertex.coords.iter())
        {
            *extent_for_dimension = extent_for_dimension.start().min(*vertex_coord)
                ..=extent_for_dimension.end().max(*vertex_coord);
        }
    }

    Some(extents_accumulator)
}

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
            Some([RangeInclusive::new(1.0, 5.0), RangeInclusive::new(1.0, 4.0)])
        );
    }

    #[test]
    fn test_calculate_polygon_extents_empty_polygon() {
        // An empty polygon
        let polygon: Vec<Point<f64, 2>> = Vec::new();

        // Calculating the extents
        let extents = calculate_polygon_extents(&polygon);

        // Expect the extents to be [max_value..=min_value] for x and y respectively
        assert_eq!(extents, None);
    }
}
