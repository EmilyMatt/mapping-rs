use crate::{
    array,
    ops::RangeInclusive,
    types::{PolygonExtents, SameSizeMat},
};
use nalgebra::{Const, DimMin, Point, RealField, SimdRealField};

/// Various utility functions regarding point clouds of 2 or 3 dimensions.
pub mod point_cloud;

#[cfg(any(not(feature = "std"), test))]
pub(crate) fn distance_squared<T, const N: usize>(point_a: &Point<T, N>, point_b: &Point<T, N>) -> T
where
    T: RealField + SimdRealField + Copy + Default,
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
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Polygon Extents", skip_all)
)]
pub fn calculate_polygon_extents<T, const N: usize>(polygon: &[Point<T, N>]) -> PolygonExtents<T, N>
where
    T: RealField + SimdRealField + Copy,
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

pub(crate) fn verify_rotation_matrix_determinant<T, const N: usize>(
    mut u: SameSizeMat<T, N>,
    v_t: SameSizeMat<T, N>,
) -> SameSizeMat<T, N>
where
    T: Copy + RealField + SimdRealField,
    Const<N>: DimMin<Const<N>, Output = Const<N>>,
{
    if (u * v_t).determinant() < T::zero() {
        u.column_mut(N - 1)
            .iter_mut()
            .for_each(|element| *element *= T::one().neg()); // Reverse the last column
    }
    u * v_t
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::Vec;
    use nalgebra::{Matrix2, Point, Point2, Point3};

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

    #[test]
    fn test_distance_squared() {
        let point_a = Point3::new(1.0, 2.0, 3.0);
        let point_b = Point3::new(4.0, 5.0, 6.0);
        assert_eq!(distance_squared(&point_a, &point_b), 27.0)
    }

    #[test]
    fn test_verify_rotation_matrix_determinant() {
        let mat_a = Matrix2::new(2.0, 3.0, 2.0, 1.0);
        let mat_b = Matrix2::new(-1.0, 0.0, 0.0, -1.0);

        let regular_dot = mat_a * mat_b;
        assert!(regular_dot.determinant() < 0.0);

        let func_dot = verify_rotation_matrix_determinant(mat_a, mat_b);

        // Verify second column is actually reversed by this function
        assert!(func_dot.determinant() >= 0.0);
        assert_eq!(func_dot.m12, -regular_dot.m12);
        assert_eq!(func_dot.m22, -regular_dot.m22);
    }
}
