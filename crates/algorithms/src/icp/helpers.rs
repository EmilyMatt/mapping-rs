use crate::types::SameSizeMat;
use nalgebra::{ArrayStorage, ComplexField, Const, Matrix, OPoint, Point, RealField, Vector};
use num_traits::AsPrimitive;
use std::{iter::Sum, ops::Add};

#[cfg(feature = "tracing")]
use tracing::instrument;

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
#[cfg_attr(feature = "tracing", instrument("Calculate Mean Point", skip_all))]
pub(crate) fn calculate_mean<T, const N: usize>(points: &[Point<T, N>]) -> Point<T, N>
where
    T: Copy + Default + ComplexField,
    usize: AsPrimitive<T>,
{
    debug_assert!(!points.is_empty());

    let zeros: [T; N] = std::array::from_fn(|_| T::default());
    points.iter().fold(Point::<T, N>::from(zeros), |acc, it| {
        Point::from(acc.coords + it.coords)
    }) / points.len().as_()
}

/// Calculates the Mean Squared Error between two point clouds.
///
/// # Arguments
/// * `transformed_points_a`: a slice of [`Point`], representing the source point cloud, transformed by the current [`Isometry`] matrix.
/// * `points_b`: a slice of [`Point`], representing the point cloud to match against.
///
/// # Returns
/// An [`f32`], representing the sum of all squared distances between each point in `transformed_points_a` and its corresponding point in `points_b`.
#[inline]
#[cfg_attr(feature = "tracing", instrument("Calculate MSE", skip_all))]
pub(crate) fn calculate_mse<T, const N: usize>(
    transformed_points_a: &[Point<T, N>],
    points_b: &[Point<T, N>],
) -> T
where
    T: ComplexField + Copy + Sum,
{
    transformed_points_a
        .iter()
        .zip(points_b.iter())
        .map(|(transformed_a, point_b)| {
            // Again, there has GOT to be a distance squred function in nalgebra, just didn't dig enough to find it
            // Also, we are doing duplicate transforming of the points, perhaps merge the two
            (0..N)
                .map(|access_idx| {
                    (transformed_a.coords.data.0[0][access_idx]
                        - point_b.coords.data.0[0][access_idx])
                        .powi(2)
                })
                .sum::<T>()
        })
        .sum::<T>()
}

/// Calculates the outer product of two `N` length [`Vector`]s.
///
/// # Arguments
/// * `point_a`: A reference to the first [`Vector`].
/// * `point_b`: A reference to the second [`Vector`].
///
/// # Returns
/// A [`SameSizeMat`] of size `N` by `N`, containing the outer product.
#[inline]
#[cfg_attr(feature = "tracing", instrument("Calculate Outer Product", skip_all))]
pub(crate) fn outer_product<T, const N: usize>(
    point_a: &Vector<T, Const<N>, ArrayStorage<T, N, 1>>,
    point_b: &Vector<T, Const<N>, ArrayStorage<T, N, 1>>,
) -> SameSizeMat<T, N>
where
    T: ComplexField + Copy,
{
    Matrix::from_data(ArrayStorage(std::array::from_fn(|b_idx| {
        std::array::from_fn(|a_idx| point_a.data.0[0][a_idx] * point_b.data.0[0][b_idx])
    })))
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
    transformed_point: &'a OPoint<T, Const<N>>,
    target_points: &'a [OPoint<T, Const<N>>],
) -> Point<T, N>
where
    T: ComplexField + RealField + Copy,
{
    debug_assert!(!target_points.is_empty());

    let mut current_distance = T::max_value().expect("Number Must Have a MAX value");
    let mut current_idx = 0;
    for (idx, target_point) in target_points.iter().enumerate() {
        let distance = (transformed_point - target_point).norm();
        log::info!("{distance}");

        if distance < current_distance {
            current_distance = distance;
            current_idx = idx;
        }
    }

    // Guaranteed to exist, so direct access is valid
    target_points[current_idx]
}

/// Calculates the estimated transformation matrix between the two point clouds.
///
/// # Arguments
/// * `points_a`: a slice of [`Point`], representing the source point cloud.
/// * `points_b`: a slice of [`Point`], representing the target point cloud.
///
/// # Returns
/// A tuple of
/// * [`SameSizeMat<T, N>`], representing the covariance matrix of the outer products of the centered point clouds.
/// * [`Point`], representing the `points_a` centeroid.
/// * [`Point`], representing the `points_b` centeroid.
///
/// # Panics
/// See [`calculate_mean`]
#[inline]
#[cfg_attr(feature = "tracing", instrument("Estimate Transform", skip_all))]
pub(crate) fn transform_using_centeroids<T, const N: usize>(
    points_a: &[Point<T, N>],
    points_b: &[Point<T, N>],
) -> (SameSizeMat<T, N>, Point<T, N>, Point<T, N>)
where
    T: Copy + ComplexField + RealField + Default,
    usize: AsPrimitive<T>,
    SameSizeMat<T, N>: Add<Output = SameSizeMat<T, N>>,
{
    let (mean_a, mean_b) = (calculate_mean(points_a), calculate_mean(points_b));
    let rot_mat = points_a.iter().zip(points_b.iter()).fold(
        Matrix::from_array_storage(ArrayStorage([[T::default(); N]; N])),
        |rot_mat, (point_a, point_b)| {
            let a_distance_from_c = point_a - mean_a;
            let b_distance_from_c = point_b - mean_b;
            rot_mat + outer_product(&a_distance_from_c, &b_distance_from_c)
        },
    );

    (rot_mat, mean_a, mean_b)
}
