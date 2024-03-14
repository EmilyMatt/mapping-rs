use nalgebra::{Point2, RealField, Vector2};
use num_traits::{AsPrimitive, Bounded};

use crate::{utils::calculate_polygon_extents, Vec};

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

/// Get all intersections of this point, with this polygon.
///
/// # Arguments
/// * `point`: A reference to a [`Point2`]
/// * `polygon`: A slice of [`Point2`]s representing the vertices.
///
/// # Generics:
/// * `T`: Either an [`prim@f32`] or [`prim@f64`]
///
/// # Returns
/// A usize, representing the number of intersections.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument(
        "Get Point's Number Of Intersections With Polygon",
        skip_all,
        level = "debug"
    )
)]
pub fn get_point_intersections_with_polygon<T>(
    point: &Point2<T>,
    polygon: &[Point2<T>],
) -> Vec<Point2<T>>
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
                .then_some(*point)
        })
        .collect() // Only returns the intersections as Some()
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
    let len: usize = get_point_intersections_with_polygon(point, polygon).len();
    len % 2 == 1 // If the number of intersections is odd - we didn't exit the polygon, and are therefor in it.
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
            #[doc = "A " $doc "-precision implementation of a point-in-polygon algorithm."]
            pub mod [<$doc _precision>] {
                use nalgebra::{Point2};
                use crate::Vec;

                #[doc = "Get all intersections of this point, with this polygon."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `point`: A reference to a [`Point2`]"]
                #[doc = "* `polygon`: A slice of [`Point2`]s representing the vertices."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "A usize, representing the number of intersections."]
                pub fn get_point_intersections_with_polygon(
                        point: &Point2<$prec>,
                        polygon: &[Point2<$prec>],
                    ) -> Vec<Point2<$prec>> {
                    super::get_point_intersections_with_polygon(point, polygon)
                }

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

    #[test]
    fn test_does_ray_intersect_on_vertex() {
        let point_a = Vector2::new(3.0, -1.5);
        let vertex_a1 = Point2::new(5.0, 0.0);
        let vertex_a2 = Point2::new(1.0, -3.0);
        assert!(does_ray_intersect_polygon_segment(
            &point_a, vertex_a1, vertex_a2
        ));
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

    // These following tests pretty much test all the functions:
    #[test]
    fn test_multiple_points_in_polygon_clockwise() {
        // A simple square polygon with vertices (0,0), (0,1), (1,1), (1,0)
        let polygon = &[
            Point2::from([0.0, 0.0]),
            Point2::from([0.0, 1.0]),
            Point2::from([1.0, 1.0]),
            Point2::from([1.0, 0.0]),
        ];

        // One point inside the polygon and one outside.
        let points = &[
            Point2::from([0.5, 0.5]), // Inside
            Point2::from([1.5, 1.5]), // Outside
        ];

        let result = single_precision::are_multiple_points_in_polygon(points, polygon);

        // Expecting [true, false] since the first point is inside and the second is outside.
        assert_eq!(result, Vec::from([true, false]));
    }

    #[test]
    fn test_multiple_points_in_polygon_counter_clockwise() {
        // A simple square polygon with vertices (0,0), (0,1), (1,1), (1,0)
        let polygon = &[
            Point2::from([0.0, 0.0]),
            Point2::from([0.0, 1.0]),
            Point2::from([1.0, 1.0]),
            Point2::from([1.0, 0.0]),
        ];

        // One point inside the polygon and one outside.
        let points = &[
            Point2::from([0.5, 0.5]), // Inside
            Point2::from([1.5, 1.5]), // Outside
        ];

        let result = single_precision::are_multiple_points_in_polygon(points, polygon);

        // Expecting [true, false] since the first point is inside and the second is outside.
        assert_eq!(result, Vec::from([true, false]));
    }
}
