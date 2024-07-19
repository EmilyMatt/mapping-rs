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

use nalgebra::{Point2, RealField, Vector2};
use num_traits::{AsPrimitive, Bounded};

use crate::Vec;

use super::calculate_polygon_extents;

#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Does Ray Intersect Polygon", skip_all, level = "trace")
)]
fn does_ray_intersect_polygon_segment<T>(
    point: &Vector2<T>,
    vertex1: Point2<T>,
    vertex2: Point2<T>,
) -> bool
where
    T: Copy + RealField,
    f32: AsPrimitive<T>,
{
    if point.y > vertex1.y.min(vertex2.y)
        && point.y <= vertex1.y.max(vertex2.y)
        && point.x <= vertex1.x.max(vertex2.x)
    {
        let origin_x = (vertex1.y != vertex2.y)
            .then(|| {
                (point.y - vertex1.y) * (vertex2.x - vertex1.x) / (vertex2.y - vertex1.y)
                    + vertex1.x
            })
            .unwrap_or(point.x);

        if vertex1.x == vertex2.x || point.x <= origin_x {
            return true;
        }
    }

    false
}

/// Check if the provided point is within the provided polygon.
///
/// # Arguments
/// * `point`: A reference to a [`Point2`].
/// * `polygon`: A slice of [`Point2`]s representing the vertices.
///
/// # Generics:
/// * `T`: Either an [`prim@f32`] or [`prim@f64`]
///
/// # Returns
/// A boolean value, specifying if the point is within the polygon.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Is Point In Polygon", skip_all, level = "debug")
)]
pub fn is_single_point_in_polygon<T>(point: &Point2<T>, polygon: &[Point2<T>]) -> bool
where
    T: Copy + RealField,
    f32: AsPrimitive<T>,
{
    let polygon_len = polygon.len();
    (0..polygon_len)
        .filter_map(|current_vertex_idx| {
            let current_vertex = polygon[current_vertex_idx];
            let next_vertex = polygon[(current_vertex_idx + 1) % polygon_len];

            does_ray_intersect_polygon_segment(&point.coords, current_vertex, next_vertex)
                .then_some(1)
        })
        .sum::<usize>()
        % 2
        == 1 // If the number of intersections is odd - we didn't exit the polygon, and are therefor in it.
}

/// This function will run the [`is_single_point_in_polygon`] for each on of the points given, and the provided polygon,
/// But pre-calculates the polygon extents to reduce workloads for larger datasets, please profile this for you specific use-case.
///
/// # Arguments
/// * `points`: A slice of [`Point2`].
/// * `polygon`: A slice of [`Point2`]s, representing the vertices.
///
/// # Generics:
/// * `T`: Either an [`prim@f32`] or [`prim@f64`]
///
/// # Returns
/// A [`Vec`] of booleans, with the same size as `points`, containing the result for each point.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Are Points In Polygon", skip_all, level = "info")
)]
pub fn are_multiple_points_in_polygon<T>(points: &[Point2<T>], polygon: &[Point2<T>]) -> Vec<bool>
where
    T: Bounded + Copy + RealField,
    f32: AsPrimitive<T>,
{
    let polygon_extents = calculate_polygon_extents(polygon);

    points
        .iter()
        .map(|current_point| {
            // Verify that each coordinate is within the bounds of the polygon, will save a lot of computational load for large polygons
            polygon_extents
                .iter()
                .zip(current_point.coords.iter())
                .fold(
                    true,
                    |is_in_extents, (extent_for_dimension, vertex_coord)| {
                        is_in_extents && extent_for_dimension.contains(vertex_coord)
                    },
                )
                && is_single_point_in_polygon(current_point, polygon)
        })
        .collect()
}

#[cfg(feature = "pregenerated")]
macro_rules! impl_p_i_p_algorithm {
    ($prec:expr, doc $doc:tt) => {
        ::paste::paste! {
            pub(super) mod [<$doc _precision>] {
                use nalgebra::{Point2};
                use crate::Vec;

                #[doc = "Check if the provided point is within the provided polygon."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `point`: A reference to a [`Point2`]."]
                #[doc = "* `polygon`: A slice of [`Point2`]s representing the vertices."]
                #[doc = "# Returns"]
                #[doc = "A boolean value, specifying if the point is within the polygon."]
                pub fn is_single_point_in_polygon(point: &Point2<$prec>, polygon: &[Point2<$prec>]) -> bool {
                    super::is_single_point_in_polygon(point, polygon)
                }

                #[doc = "This function will run the [`is_single_point_in_polygon`] for each on of the points given, and the provided polygon,"]
                #[doc = "But pre-calculates the polygon extents to reduce workloads for larger datasets, please profile this for you specific use-case."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `points`: A slice of [`Point2`]."]
                #[doc = "* `polygon`: A slice of [`Point2`]s, representing the vertices."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "A [`Vec`](crate::Vec) of booleans, with the same size as `points`, containing the result for each point."]
                pub fn are_multiple_points_in_polygon(
                    points: &[Point2<$prec>],
                    polygon: &[Point2<$prec>],
                ) -> Vec<bool> {
                    super::are_multiple_points_in_polygon(points, polygon)
                }
            }
        }
    };
}

#[cfg(feature = "pregenerated")]
impl_p_i_p_algorithm!(f32, doc single);
#[cfg(feature = "pregenerated")]
impl_p_i_p_algorithm!(f64, doc double);

#[cfg(test)]
mod tests {
    use nalgebra::{Point2, Vector2};

    use crate::Vec;

    use super::*;

    fn get_polygon_for_tests() -> Vec<Point2<f32>> {
        Vec::from([
            Point2::from([0.0, 0.0]),
            Point2::from([1.0, 1.2]),
            Point2::from([1.4, 1.2]),
            Point2::from([1.4, 2.0]),
            Point2::from([0.5, 1.8]),
            Point2::from([-2.0, 0.2]),
            Point2::from([-1.2, -0.4]),
            Point2::from([-0.3, -0.4]),
        ])
    }

    #[test]
    fn test_does_ray_intersect() {
        let point_a = Vector2::new(0.5, -1.5);
        let vertex_a1 = Point2::new(5.0, 0.0);
        let vertex_a2 = Point2::new(1.0, -4.0);
        assert!(does_ray_intersect_polygon_segment(
            &point_a, vertex_a1, vertex_a2
        ));

        let point_b = Vector2::new(-0.5, -1.5);
        let vertex_b1 = Point2::new(0.0, 0.0);
        let vertex_b2 = Point2::new(1.0, 5.0);
        assert!(!does_ray_intersect_polygon_segment(
            &point_b, vertex_b1, vertex_b2
        ));
    }

    #[test]
    fn test_is_single_points_in_polygon() {
        let polygon = get_polygon_for_tests();

        let point = Point2::from([0.5, 1.5]);

        assert!(single_precision::is_single_point_in_polygon(
            &point, &polygon
        ));
    }

    #[test]
    fn test_multiple_points_in_polygon_clockwise() {
        let polygon = get_polygon_for_tests();

        // One point inside the polygon and one outside.
        let points = &[
            Point2::from([0.5, 1.5]), // Inside
            Point2::from([1.5, 1.5]), // Outside
        ];

        let result = single_precision::are_multiple_points_in_polygon(points, &polygon);

        // Expecting [true, false] since the first point is inside and the second is outside.
        assert_eq!(result, Vec::from([true, false]));
    }

    #[test]
    fn test_multiple_points_in_polygon_counter_clockwise() {
        let polygon = get_polygon_for_tests();

        // One point inside the polygon and one outside.
        let points = &[
            Point2::from([0.5, 1.5]), // Inside
            Point2::from([1.5, 1.5]), // Outside
        ];

        let result = single_precision::are_multiple_points_in_polygon(points, &polygon);

        // Expecting [true, false] since the first point is inside and the second is outside.
        assert_eq!(result, Vec::from([true, false]));
    }
}
