#[cfg(feature = "tracing")]
use tracing::instrument;

use crate::types::{ICPSuccess, IsometryAbstration};
use helpers::{calculate_mse, find_closest_point, transform_using_centeroids};
use nalgebra::{ComplexField, Const, Point, RealField};
use num_traits::AsPrimitive;
use std::iter::Sum;

mod helpers;

#[cfg_attr(feature = "tracing", instrument("Full ICP ALgorithm", skip_all))]
fn icp<T, const N: usize, O>(
    points_a: &[Point<T, N>],
    points_b: &[Point<T, N>],
    max_iterations: usize,
    mse_threshold: T,
) -> Result<ICPSuccess<T, N, O>, String>
where
    T: ComplexField + Copy + Default + RealField + Sum,
    usize: AsPrimitive<T>,
    O: IsometryAbstration<T, N>,
{
    let mut current_transform: O::Isom = O::identity();
    let mut current_mse = T::max_value().expect("Must have MAX");

    let mut transformed_points = points_a
        .iter()
        .map(|point_a| O::transform_point(&current_transform, point_a))
        .collect::<Vec<_>>();

    for iteration_num in 0..max_iterations {
        let closest_points = transformed_points
            .iter()
            .map(|transformed_point_a| find_closest_point(transformed_point_a, points_b))
            .collect::<Vec<_>>();

        let (rot_mat, mean_a, mean_b) =
            transform_using_centeroids(transformed_points.as_slice(), closest_points.as_slice());

        current_transform = O::update_transform(&current_transform, mean_b, mean_a, &rot_mat);

        for (idx, point_a) in points_a.iter().enumerate() {
            transformed_points[idx] = O::transform_point(&current_transform, point_a)
        }
        let new_mse = calculate_mse(transformed_points.as_slice(), closest_points.as_slice());

        if (current_mse - new_mse).abs() < mse_threshold {
            return Ok(ICPSuccess {
                transform: current_transform,
                mse: new_mse,
                iteration_num,
            });
        }

        current_mse = new_mse;
    }

    Err("Could not converge".to_string())
}

/// An ICP algorithm in 2D space.
/// # Generics
/// * `T`: either an [`f32`] or [`f64`]
///
/// # Arguments
/// * `points_a`: A slice of [`Point<T, 2>`], representing the source point cloud.
/// * `points_b`: A slice of [`Point<T, 2>`], representing the target point cloud.
/// * `max_iterations`: a [`usize`], specifying for how many iterations to try converging before returning an error.
/// * `mse_threshold`: a `T`, if the MSE __differential__ is smaller than this number, we are considered converged[^convergence_note].
///
/// # Returns
/// An [`ICPSuccess`] struct with an [`Isometry`](nalgebra::Isometry) of [`UnitComplex`](nalgebra::UnitComplex) with `T` precision.
///
/// [^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration.
///
#[cfg_attr(feature = "tracing", instrument("2D ICP ALgorithm", skip_all))]
pub fn icp_2d<T>(
    points_a: &[Point<T, 2>],
    points_b: &[Point<T, 2>],
    max_iterations: usize,
    mse_threshold: T,
) -> Result<ICPSuccess<T, 2, Const<2>>, String>
where
    T: ComplexField + Copy + Default + RealField + Sum,
    usize: AsPrimitive<T>,
{
    icp(points_a, points_b, max_iterations, mse_threshold)
}

/// An ICP algorithm in 3D space.
/// # Generics
/// * `T`: either an [`f32`] or [`f64`]
///
/// # Arguments
/// * `points_a`: A slice of [`Point<T, 3>`], representing the source point cloud.
/// * `points_b`: A slice of [`Point<T, 3>`], representing the target point cloud.
/// * `max_iterations`: a [`usize`], specifying for how many iterations to try converging before returning an error.
/// * `mse_threshold`: a `T`, if the MSE __differential__ is smaller than this number, we are considered converged[^convergence_note].
///
/// # Returns
/// An [`ICPSuccess`] struct with an [`Isometry`](nalgebra::Isometry) of [`UnitQuaternion`](nalgebra::UnitQuaternion) with `T` precision.
///
/// [^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration.
///
#[cfg_attr(feature = "tracing", instrument("3D ICP ALgorithm", skip_all))]
pub fn icp_3d<T>(
    points_a: &[Point<T, 3>],
    points_b: &[Point<T, 3>],
    max_iterations: usize,
    mse_threshold: T,
) -> Result<ICPSuccess<T, 3, Const<3>>, String>
where
    T: ComplexField + Copy + Default + RealField + Sum,
    usize: AsPrimitive<T>,
{
    icp(points_a, points_b, max_iterations, mse_threshold)
}

#[cfg(test)]
mod tests {
    use crate::utils;

    #[test]
    fn test_csm_2d() {
        let translation = nalgebra::Vector2::new(-0.8, 1.3);
        let isom = nalgebra::Isometry2::new(translation, 0.1);
        let (points, points_transformed) = utils::tests::generate_points(isom);

        let icp_res = super::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            100,
            0.0001,
        );
        assert!(icp_res.is_ok());
    }

    #[test]
    fn test_sm_3d() {
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.5, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let (points, points_transformed) = utils::tests::generate_points(isom);

        let icp_res = super::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            100,
            0.0001,
        );
        assert!(icp_res.is_ok());
    }
}
