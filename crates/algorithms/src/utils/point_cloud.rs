use nalgebra::{AbstractRotation, Isometry, Point, RealField, SimdRealField};
use num_traits::AsPrimitive;

/// Calculates the mean(centeroid) of the point cloud.
///
/// # Arguments
/// * points: a slice of [`Point`], representing the point cloud.
///
/// # Returns
/// A [`Point`], representing the point cloud centeroid.
///
/// # Panics
/// In debug builds, this function will panic if `points` is an empty slice, to avoid dividing by 0.
#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Mean Point", skip_all)
)]
pub fn calculate_point_cloud_center<T, const N: usize>(points: &[Point<T, N>]) -> Point<T, N>
where
    T: RealField + SimdRealField + Copy,
    usize: AsPrimitive<T>,
{
    debug_assert!(!points.is_empty());

    points
        .iter()
        .fold(Point::<T, N>::from([T::zero(); N]), |acc, it| {
            Point::from(acc.coords + it.coords)
        })
        / points.len().as_()
}

#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Find Closest Points", skip_all)
)]
pub(crate) fn find_closest_point<'a, T, const N: usize>(
    transformed_point: &'a Point<T, N>,
    target_points: &'a [Point<T, N>],
) -> Point<T, N>
where
    T: RealField + SimdRealField + Copy,
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

/// Downsample a points cloud, returning a new point cloud, with minimum intervals between each point.
/// # Arguments
/// * `points`: a slice of [`Point<T, N>`], representing the point cloud.
/// * `min_distance`: a floating point number, specifying the minimum interval between points.
///
/// # Returns
/// A [`Vec`] of [`Point<f32, N>`] representing the downsampled point cloud.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Downsample Point Cloud", skip_all)
)]
pub fn downsample_point_cloud<T, const N: usize>(
    points: &[Point<T, N>],
    min_distance: T,
) -> Vec<Point<T, N>>
where
    T: RealField + SimdRealField + Copy,
{
    if points.is_empty() {
        return Vec::new();
    }

    let mut latest_point = points[0];
    points
        .iter()
        .skip(1)
        .filter_map(|element| {
            if nalgebra::distance(element, &latest_point) >= min_distance {
                latest_point = *element;
                Some(*element)
            } else {
                None
            }
        })
        .collect()
}

/// Generates a points cloud, and a corresponding points cloud, transformed by `isometry_matrix`
/// # Arguments
/// * `num_points`: a [`usize`], specifying the amount of points to generate
/// * `range`: a [`RangeInclusive<T>`] specifying the normal distribution of points.
///
/// # Returns
/// A [`Vec`] of [`Point<f32, N>`] representing the point cloud.
#[cfg(any(all(feature = "std", feature = "rand"), test))]
pub fn generate_point_cloud<T, const N: usize>(
    num_points: usize,
    range: std::ops::RangeInclusive<T>,
) -> Vec<Point<T, N>>
where
    T: RealField + SimdRealField + rand::distributions::uniform::SampleUniform,
{
    use rand::Rng;
    let mut rng = rand::thread_rng();

    (0..num_points)
        .map(|_| nalgebra::Point::from(std::array::from_fn(|_| rng.gen_range(range.clone()))))
        .collect()
}

/// Transform a point cloud using an [`AbstractRotation`], returning a transformed point cloud.
/// This function does not mutate the original point cloud.
/// # Arguments
/// * `source_points`: a slice of [`Point<T, N>`], representing the point cloud
/// * `isometry_matrix`: a transform that implements [`AbstractRotation<T, N>`], to use for the transformation.
///
/// # Returns
/// A [`Vec`] of [`Point<f32, N>`] containing the transformed point cloud.
#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Transform Point Cloud", skip_all)
)]
pub fn transform_point_cloud<T, const N: usize, R>(
    source_points: &[Point<T, N>],
    isometry_matrix: Isometry<T, R, N>,
) -> Vec<Point<T, N>>
where
    T: RealField + SimdRealField,
    R: AbstractRotation<T, N>,
{
    source_points
        .iter()
        .map(|point| isometry_matrix.transform_point(point))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::find_closest_point;
    use nalgebra::{Point, Point2};

    #[cfg(not(feature = "std"))]
    use alloc::vec::Vec;
    #[cfg(feature = "std")]
    use std::vec::Vec;

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
}
