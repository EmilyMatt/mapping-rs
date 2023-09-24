use crate::types::PolygonExtents;
use nalgebra::{ComplexField, Point, RealField, SimdComplexField, SimdRealField};

#[cfg(feature = "std")]
use std::{array, ops::RangeInclusive};
#[cfg(not(feature = "std"))]
use {
    core::{
        array,
        fmt::Debug,
        ops::{RangeInclusive, SubAssign},
    },
    num_traits::float::FloatCore as Float,
};

#[cfg(feature = "tracing")]
use tracing::instrument;

#[cfg(not(feature = "std"))]
pub(crate) fn distance_squared<T, const N: usize>(point_a: &Point<T, N>, point_b: &Point<T, N>) -> T
where
    T: 'static + Float + Debug + SubAssign + ComplexField<RealField = T>,
{
    let distance = (point_a - point_b).norm();
    distance * distance
}

/// Finds the closest matching target point to the passed source point.
///
/// # Arguments
/// * `transformed_point`: a [`Point`], for which to find the closest point.
/// * `target_points`: a slice of [`Point`], representing the target point cloud.
///
/// # Returns
/// A [`Point`], representing said closest point.
///
/// # Panics
/// In debug builds, this function will panic if the `target_points` is an empty slice.
#[inline]
#[cfg_attr(feature = "tracing", instrument("Find Closest Points", skip_all))]
pub(crate) fn find_closest_point<'a, T, const N: usize>(
    transformed_point: &'a Point<T, N>,
    target_points: &'a [Point<T, N>],
) -> Point<T, N>
where
    T: ComplexField + RealField + Copy,
{
    debug_assert!(!target_points.is_empty());

    let mut current_distance = T::max_value().expect("Number Must Have a MAX value");
    let mut current_idx = 0;
    for (idx, target_point) in target_points.iter().enumerate() {
        let distance = (transformed_point - target_point).norm();

        if distance < current_distance {
            current_distance = distance;
            current_idx = idx;
        }
    }

    // Guaranteed to exist, so direct access is valid
    target_points[current_idx]
}

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
    let mut extents_accumulator: [RangeInclusive<T>; N] = array::from_fn(|_| {
        T::max_value().expect("System floating number must have a maximum value")
            ..=T::min_value().expect("System floating number must have a minimum value")
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

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use nalgebra::{Isometry, Point, Point2};

    #[cfg(feature = "std")]
    use std::vec::Vec;
    #[cfg(not(feature = "std"))]
    use {alloc::vec::Vec, core::array};

    /// Generates a points cloud, and a corresponding points cloud, transformed by `isometry_matrix`
    /// # Arguments
    /// * `isometry_matrix`: an [`Isometry`], with [`R`] being [`UnitQuaternion`](nalgebra::UnitQuaternion) for [`N = 3`] and [`UnitComplex`](nalgebra::UnitComplex) for [`N = 2`]
    ///
    /// # Returns
    /// A tuple of two [`Vec<Point<f32, N>>`]'s, the first one being the source set of points, and the second the transformed set of points.
    pub fn generate_points<const N: usize, R>(
        num_points: usize,
        isometry_matrix: Isometry<f32, R, N>,
    ) -> (Vec<Point<f32, N>>, Vec<Point<f32, N>>)
    where
        R: nalgebra::AbstractRotation<f32, N>,
    {
        use rand::Rng;
        let mut rng = rand::thread_rng();

        (0..num_points)
            .map(|_| {
                let storage: [f32; N] = array::from_fn(|_| rng.gen_range(-15.0f32..=15.0f32));
                let orig = nalgebra::Point::from(storage);
                let transformed = isometry_matrix.transform_point(&orig);

                (orig, transformed)
            })
            .unzip()
    }

    #[test]
    fn test_find_closest_point() {
        // Given:
        // A set of target points
        let target_points = Vec::from([
            Point2::new(1.0, 1.0),
            Point2::new(2.0, 2.0),
            Point2::new(5.0, 5.0),
            Point2::new(8.0, 8.0),
        ]);

        // A transformed point
        let transformed_point = Point2::new(4.0, 4.0);

        // When:
        // Finding the closest point
        let closest_point = find_closest_point(&transformed_point, &target_points);

        // Expect the closest point to be (5.0, 5.0)
        assert_eq!(closest_point, Point2::new(5.0, 5.0));
    }

    #[test]
    #[should_panic]
    fn test_find_closest_point_with_empty_target() {
        // Given:
        // An empty set of target points
        let target_points: Vec<Point<f64, 2>> = Vec::new();

        // A transformed point
        let transformed_point = Point2::new(4.0, 4.0);

        // This should panic as the target_points array is empty
        let _ = find_closest_point(&transformed_point, &target_points);
    }

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
