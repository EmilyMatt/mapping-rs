use crate::types::PolygonExtents;
use nalgebra::{ComplexField, Const, Point, RealField, SimdComplexField, SimdRealField};
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
    fn get_intersections_num(point: &Point<T, N>, polygon: &[Point<T, N>]) -> usize;

    /// Check if the provided point is within the provided polygon.
    ///
    /// # Arguments
    /// * `point`: A reference to a [`Point`].
    /// * `polygon`: A slice of [`Point`]s representing the vertices.
    ///
    /// # Returns
    /// A boolean value, specifying if the point is within the polygon.
    fn single_point_in_polygon(point: &Point<T, N>, polygon: &[Point<T, N>]) -> bool;

    /// This function will run the [`single_point_in_polygon`] for each on of the points given, and the provided polygon,
    /// But containing some further optimizations to reduce workloads for larger datasets.
    /// # Arguments
    /// `points`: A slice of [`Point`].
    /// `polygon`: A slice of [`Point`]s, representing the vertices.
    ///
    /// # Returns
    /// A [`Vec`] of [`bool`], with the same size as `points`, containing the result for each point.
    fn multiple_points_in_polygon(points: &[Point<T, N>], polygon: &[Point<T, N>]) -> Vec<bool>;
}

impl<T> HasPointInPolygonAlgorithm<T, 2> for Const<2>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    fn get_intersections_num(point: &Point<T, 2>, polygon: &[Point<T, 2>]) -> usize {
        let polygon_len = polygon.len();
        (0..polygon_len).fold(0usize, |intersections, current_vertex_idx| {
            let current_vertex = polygon[current_vertex_idx];
            let next_vertex = polygon[(current_vertex_idx + 1) % polygon_len];

            if (current_vertex.y < point.y && next_vertex.y >= point.y
                || next_vertex.y < point.y && current_vertex.y >= point.y)
                && current_vertex.x
                    + (point.y - current_vertex.y) / (next_vertex.y - current_vertex.y)
                        * (next_vertex.x - current_vertex.x)
                    < point.x
            {
                return intersections + 1;
            }

            intersections
        })
    }

    fn single_point_in_polygon(point: &Point<T, 2>, polygon: &[Point<T, 2>]) -> bool {
        Self::get_intersections_num(point, polygon) % 2 == 0
    }

    fn multiple_points_in_polygon(points: &[Point<T, 2>], polygon: &[Point<T, 2>]) -> Vec<bool> {
        let polygon_extents = calculate_polygon_extents(polygon);

        points
            .iter()
            .map(|current_point| {
                // Verify that each coordinate is within the bounds of the polygon, will save a lot of computational load for large polygons
                polygon_extents
                    .iter()
                    .zip(current_point.coords.iter())
                    .fold(
                        false,
                        |is_in_extents, (extent_for_dimension, vertex_coord)| {
                            is_in_extents && extent_for_dimension.contains(vertex_coord)
                        },
                    )
                    && Self::single_point_in_polygon(current_point, polygon)
            })
            .collect()
    }
}

/// See [`HasPointInPolygonAlgorithm::get_intersections_num`]
pub fn get_intersections_num<T, const N: usize, O>(
    point: &Point<T, N>,
    polygon: &[Point<T, N>],
) -> usize
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
    O: HasPointInPolygonAlgorithm<T, N>,
{
    O::get_intersections_num(point, polygon)
}

/// See [`HasPointInPolygonAlgorithm::single_point_in_polygon`]
pub fn single_point_in_polygon<T, const N: usize, O>(
    point: &Point<T, N>,
    polygon: &[Point<T, N>],
) -> bool
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
    O: HasPointInPolygonAlgorithm<T, N>,
{
    O::single_point_in_polygon(point, polygon)
}

/// See [`HasPointInPolygonAlgorithm::multiple_points_in_polygon`]
pub fn multiple_points_in_polygon<T, const N: usize, O>(
    points: &[Point<T, N>],
    polygon: &[Point<T, N>],
) -> Vec<bool>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
    O: HasPointInPolygonAlgorithm<T, N>,
{
    O::multiple_points_in_polygon(points, polygon)
}
