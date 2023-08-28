use crate::types::PolygonExtents;
use nalgebra::{ComplexField, Const, Point, Point2, RealField, SimdComplexField, SimdRealField};
use std::ops::RangeInclusive;

/// This function calculates the extents of the polygon, i.e., the minimum and maximum values for each coordinate dimension.
///
/// # Generics
/// * `T`: one of [`f32`] or [`f64`].
/// * `N`: a constant generic of type [`usize`].
///
/// # Arguments
/// * `polygon`: a slice of [`Point<T, N>`].
///
/// # Returns
/// See [`PolygonExtents`]
pub fn calculate_polygon_extents<T, const N: usize>(polygon: &[Point<T, N>]) -> PolygonExtents<T, N>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    let mut extents_accumulator: [RangeInclusive<T>; N] = std::array::from_fn(|_| {
        T::max_value().expect("Must have a maximum value")
            ..=T::min_value().expect("Must have a minimum value")
    });

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

#[inline]
fn does_ray_intersect<T>(
    mut point: Point2<T>,
    mut vertex1: Point2<T>,
    mut vertex2: Point2<T>,
) -> bool
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    // Reverse direction
    if vertex1.y > vertex2.y {
        std::mem::swap(&mut vertex1, &mut vertex2);
    }

    // Handle case where difference is too small and will cause issues
    if point.y == vertex1.y || point.y == vertex2.y {
        point.y += T::default_epsilon();
    }

    // Check if out of extents
    if point.y > vertex2.y || point.y < vertex1.y || point.x > vertex1.x.max(vertex2.x) {
        return false;
    }

    if point.x < vertex1.x.min(vertex2.x) {
        return true;
    }

    (((vertex2.x - vertex1.x) * (point.y - vertex1.y))
        - ((vertex2.y - vertex1.y) * (point.x - vertex1.x)))
        < T::zero()
}

/// A trait blanketing all dimensions for which a point-in-polygon algorithm is implemented.
///
/// # Generics
/// * `T`: One of [`f32`] or [`f64`].
/// * `N`: The number of dimensions.
pub trait HasPointInPolygonAlgorithm<T, const N: usize>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    /// Get the number of intersections of this point as a vector, with this polygon.
    ///
    /// # Arguments
    /// * `point`: A reference to a [`Point`].
    /// * `polygon`: A slice of [`Point`]s representing the vertices.
    ///
    /// # Returns
    /// A usize, representing the number of intersections.
    fn get_point_polygon_intersections_num(point: &Point<T, N>, polygon: &[Point<T, N>]) -> usize;

    /// Check if the provided point is within the provided polygon.
    ///
    /// # Arguments
    /// * `point`: A reference to a [`Point`].
    /// * `polygon`: A slice of [`Point`]s representing the vertices.
    ///
    /// # Returns
    /// A boolean value, specifying if the point is within the polygon.
    fn is_single_point_in_polygon(point: &Point<T, N>, polygon: &[Point<T, N>]) -> bool;

    /// This function will run the [`Self::is_single_point_in_polygon`] for each on of the points given, and the provided polygon,
    /// But containing some further optimizations to reduce workloads for larger datasets.
    /// # Arguments
    /// * `points`: A slice of [`Point`].
    /// * `polygon`: A slice of [`Point`]s, representing the vertices.
    ///
    /// # Returns
    /// A [`Vec`] of [`bool`], with the same size as `points`, containing the result for each point.
    fn are_multiple_points_in_polygon(points: &[Point<T, N>], polygon: &[Point<T, N>])
        -> Vec<bool>;
}

impl<T> HasPointInPolygonAlgorithm<T, 2> for Const<2>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    fn get_point_polygon_intersections_num(point: &Point<T, 2>, polygon: &[Point<T, 2>]) -> usize {
        let polygon_len = polygon.len();
        (0..polygon_len).fold(0usize, |intersections, current_vertex_idx| {
            let current_vertex = polygon[current_vertex_idx];
            let next_vertex = polygon[(current_vertex_idx + 1) % polygon_len];

            if does_ray_intersect(*point, current_vertex, next_vertex) {
                return intersections + 1;
            }

            intersections
        })
    }

    fn is_single_point_in_polygon(point: &Point<T, 2>, polygon: &[Point<T, 2>]) -> bool {
        Self::get_point_polygon_intersections_num(point, polygon) % 2 == 1
    }

    fn are_multiple_points_in_polygon(
        points: &[Point<T, 2>],
        polygon: &[Point<T, 2>],
    ) -> Vec<bool> {
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
                    && Self::is_single_point_in_polygon(current_point, polygon)
            })
            .collect()
    }
}

/// See [`HasPointInPolygonAlgorithm::get_point_polygon_intersections_num`]
pub fn get_point_polygon_intersections_num_2d<T>(
    point: &Point<T, 2>,
    polygon: &[Point<T, 2>],
) -> usize
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    <Const<2> as HasPointInPolygonAlgorithm<T, 2>>::get_point_polygon_intersections_num(
        point, polygon,
    )
}

/// See [`HasPointInPolygonAlgorithm::is_single_point_in_polygon`]
pub fn is_single_point_in_polygon_2d<T>(point: &Point<T, 2>, polygon: &[Point<T, 2>]) -> bool
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    <Const<2> as HasPointInPolygonAlgorithm<T, 2>>::is_single_point_in_polygon(point, polygon)
}

/// See [`HasPointInPolygonAlgorithm::are_multiple_points_in_polygon`]
pub fn are_multiple_points_in_polygon_2d<T>(
    points: &[Point<T, 2>],
    polygon: &[Point<T, 2>],
) -> Vec<bool>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    <Const<2> as HasPointInPolygonAlgorithm<T, 2>>::are_multiple_points_in_polygon(points, polygon)
}

#[cfg(test)]
mod tests {
    use super::*;

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

        let result = are_multiple_points_in_polygon_2d(points, polygon);

        // Expecting [true, false] since the first point is inside and the second is outside.
        assert_eq!(result, vec![true, false]);
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

        let result = are_multiple_points_in_polygon_2d(points, polygon);

        // Expecting [true, false] since the first point is inside and the second is outside.
        assert_eq!(result, vec![true, false]);
    }
}
