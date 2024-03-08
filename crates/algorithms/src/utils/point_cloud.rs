use crate::{array, utils::distance_squared, HashMap, Vec};
use nalgebra::{
    AbstractRotation, ClosedAdd, ClosedDiv, ComplexField, Isometry, Point, RealField, Scalar,
};
use num_traits::{AsPrimitive, Bounded, NumOps, Zero};

/// Calculates the mean(centeroid) of the point cloud.
///
/// # Arguments
/// * points: a slice of [`Point`], representing the point cloud.
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, representing the number of dimensions in the points.
///
/// # Returns
/// A [`Point`], representing the point cloud centeroid.
/// Returns Point::default() if point cloud is empty.
#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Mean Point", skip_all)
)]
pub fn calculate_point_cloud_center<T, const N: usize>(points: &[Point<T, N>]) -> Point<T, N>
where
    T: ClosedAdd + ClosedDiv + Copy + Scalar + Zero,
    usize: AsPrimitive<T>,
{
    if points.is_empty() {
        return Point::default();
    }

    points
        .iter()
        .fold(Point::<T, N>::from([T::zero(); N]), |acc, it| {
            Point::from(acc.coords + it.coords)
        })
        / points.len().as_()
}

/// Finds the closest matching target point to the passed source point.
///
/// # Arguments
/// * `point`: A [`Point`], for which to find the closest point.
/// * `all_points`: A slice of [`Point`], representing the target point cloud.
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, representing the number of dimensions in the points.
///
/// # Returns
/// A [`Point`], representing said closest point.
///
/// # Panics
/// this function will panic if the `target_points` is an empty slice.
#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Find Closest Points", skip_all)
)]
pub fn find_closest_point<T, const N: usize>(
    point: &Point<T, N>,
    all_points: &[Point<T, N>],
) -> Point<T, N>
where
    T: Bounded + Copy + Default + NumOps + PartialOrd + Scalar,
{
    assert!(!all_points.is_empty(), "Point cloud must not be empty");

    let mut current_distance = T::max_value();
    let mut current_point = all_points[0]; // Guaranteed to exist

    for target_point in all_points.iter() {
        let distance = distance_squared(point, target_point);
        if distance < current_distance {
            current_distance = distance;
            current_point = *target_point;
        }
    }

    current_point
}

/// Generates a randomized points cloud within a specified spherical range.
///
/// # Arguments
/// * `num_points`: a [`usize`], specifying the amount of points to generate
/// * `range`: a [`crate::ops::RangeInclusive<T>`] specifying the normal distribution of points.
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, representing the number of dimensions to use.
///
/// # Returns
/// A [`Vec`] of [`Point<f32, N>`] representing the point cloud.
pub fn generate_point_cloud<T, const N: usize>(
    num_points: usize,
    range: crate::ops::RangeInclusive<T>,
) -> Vec<Point<T, N>>
where
    T: PartialOrd + rand::distributions::uniform::SampleUniform + Scalar,
{
    use rand::{Rng, SeedableRng};
    let mut rng = rand::rngs::SmallRng::seed_from_u64(3765665954583626552);

    (0..num_points)
        .map(|_| nalgebra::Point::from(array::from_fn(|_| rng.gen_range(range.clone()))))
        .collect()
} // Just calls a different function a number of times, no specific test needed

/// Transform a point cloud, returning a transformed copy.
/// This function does not mutate the original point cloud.
///
/// # Arguments
/// * `source_points`: a slice of [`Point<T, N>`], representing the point cloud
/// * `isometry_matrix`: a transform that implements [`AbstractRotation<T, N>`], to use for the transformation.
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, either `2` or `3`.
/// * `R`: An [`AbstractRotation`] for `T` and `N`.
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
    T: RealField,
    R: AbstractRotation<T, N>,
{
    source_points
        .iter()
        .map(|point| isometry_matrix.transform_point(point))
        .collect()
} // Just calls a different function a number of times, no specific test needed

/// Downsample a points cloud, returning a new point cloud, with all points within each voxel combined into their mean.
///
/// # Arguments
/// * `points`: a slice of [`Point<T, N>`], representing the point cloud.
/// * `voxel_size`: a floating point number, specifying the size for each voxel, all points inside that voxel will be downsampled to their centeroid..
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, representing the number of dimensions in the points.
///
/// # Returns
/// A [`Vec`] of [`Point<f32, N>`] representing the downsampled point cloud.
///
/// # Warnings
/// * Point cloud order is *never* guaranteed.
/// * When compiling for no_std, a `BTreeMap` from the `alloc` crate is used in place of a [`HashMap`].
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Downsample Point Cloud Using Voxels", skip_all)
)]
pub fn voxel_downsample_point_cloud<T, const N: usize>(
    points: &[Point<T, N>],
    voxel_size: T,
) -> Vec<Point<T, N>>
where
    T: ComplexField + Copy + AsPrimitive<isize>,
    usize: AsPrimitive<T>,
{
    let mut voxel_map: HashMap<[isize; N], Vec<Point<T, N>>> = HashMap::new();

    // Assign points to voxels
    for point in points {
        let voxel_coords: [isize; N] =
            array::from_fn(|idx| (point[idx] / voxel_size).floor().as_());
        voxel_map.entry(voxel_coords).or_default().push(*point);
    }

    // Compute centroid for each voxel and collect them as the downsampled points
    voxel_map
        .into_values()
        .map(|points_in_voxel| {
            let num_points = points_in_voxel.len().as_();
            let sum = points_in_voxel
                .into_iter()
                .fold(Point::default(), |acc, p| acc + p.coords);
            sum / num_points
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vec;
    use nalgebra::{Point, Point2, Point3};

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
    fn test_calculate_point_cloud_center() {
        let point_cloud = [
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(2.0, 3.0, 4.0),
            Point3::new(3.0, 4.0, 5.0),
            Point3::new(-2.0, -1.0, 0.0),
            Point3::new(-5.0, -2.0, -3.0),
            Point3::new(1.0, 0.0, 0.0),
        ];

        assert_eq!(
            calculate_point_cloud_center(point_cloud.as_slice()),
            Point3::new(0.0, 1.0, 1.5)
        );
    }

    #[test]
    fn test_downsample_point_cloud() {
        let point_cloud = [
            Point3::new(-5.9, -5.0, -3.9), // These two are very close now
            Point3::new(-6.0, -5.0, -4.0), // Will end up in the same voxel
            Point3::new(-1.0, -2.0, -3.0),
            Point3::new(0.0, 0.0, 0.0),    // These two are also very close
            Point3::new(0.05, 0.08, 0.01), // Will end up in the same voxel
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(6.0, 5.0, 4.0),
        ];

        // We should be left with 5 voxels
        let res = voxel_downsample_point_cloud(point_cloud.as_slice(), 0.5);
        assert_eq!(res.len(), 5);

        // Moreover, the most negative voxel had two points, (-5.9, -5.0, -4.0) and (-6.0, -5.0, -4.0)
        // Meaning there should be a voxel resulting in the elements' centeroid
        assert!(res
            .iter()
            .any(|element| *element == Point3::new(-5.95, -5.0, -3.95)));
    }
}
