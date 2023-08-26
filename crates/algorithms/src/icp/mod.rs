#[cfg(any(feature = "2d", feature = "3d"))]
use {
    crate::types::ICPSuccess,
    helpers::{calculate_mse, find_closest_point, transform_using_centeroids},
    nalgebra::{Isometry, Point},
};

mod helpers;

// TODO: implement using trait, that will only be implemented for these two combos:
// 1. <2usize, UnitComplex>
// 2. <3usize, UnitQuaternion>

// TODO: see how much of this code can be reused, I think only the SVD part actually requires a specific struct
// Maybe move mse calculation upwards? IDK, requires more design

/// An ICP function in 2D space.
/// # Arguments
/// * `points_a`: A slice of [`Point<f32, 2>`], representing the source point cloud.
/// * `points_b`: A slice of [`Point<f32, 2>`], representing the target point cloud.
/// * `max_iterations`: a [`usize`], specifying for how many iterations to try converging before returning an error.
/// * `mse_threshold`: an [`f32`], if the MSE __differential__ is smaller than this number, we are considered converged[^convergence_note].
///
/// # Returns
/// An [`ICPSuccess`] struct with a 2D isometric matrix.
///
/// [^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration.
///
#[cfg(feature = "2d")]
pub fn icp_2d(
    points_a: &[Point<f32, 2>],
    points_b: &[Point<f32, 2>],
    max_iterations: usize,
    mse_threshold: f32,
) -> Result<ICPSuccess<2, nalgebra::UnitComplex<f32>>, String> {
    let mut current_transform = Isometry::identity();
    let mut current_mse = f32::MAX;

    let mut transformed_points = points_a
        .iter()
        .map(|point_a| current_transform.transform_point(point_a))
        .collect::<Vec<_>>();
    for iteration_num in 0..max_iterations {
        let closest_points = transformed_points
            .iter()
            .map(|transformed_point_a| find_closest_point(transformed_point_a, points_b))
            .collect::<Vec<_>>();

        let (rot_mat, mean_a, mean_b) =
            transform_using_centeroids(transformed_points.as_slice(), closest_points.as_slice());

        let svd = rot_mat.svd(true, true);
        let rotation = svd.v_t.unwrap().transpose() * svd.u.unwrap().transpose();
        let translation = mean_b.coords - (rotation * mean_a.coords);

        let estimated_transform = Isometry::from_parts(
            translation.into(),
            nalgebra::UnitComplex::from_matrix(&rotation),
        );

        current_transform = estimated_transform * current_transform;

        for (idx, point_a) in points_a.iter().enumerate() {
            transformed_points[idx] = current_transform.transform_point(point_a)
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

/// An ICP function in 3D space.
/// # Arguments
/// * `points_a`: A slice of [`Point<f32, 3>`], representing the source point cloud.
/// * `points_b`: A slice of [`Point<f32, 3>`], representing the target point cloud.
/// * `max_iterations`: a [`usize`], specifying for how many iterations to try converging before returning an error.
/// * `mse_threshold`: an [`f32`], if the MSE __differential__ is smaller than this number, we are considered converged[^convergence_note].
///
/// # Returns
/// An [`ICPSuccess`] struct with a 3D isometric matrix.
///
/// [^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration.
///
#[cfg(feature = "3d")]
pub fn icp_3d(
    points_a: &[Point<f32, 3>],
    points_b: &[Point<f32, 3>],
    max_iterations: usize,
    mse_threshold: f32,
) -> Result<ICPSuccess<3, nalgebra::UnitQuaternion<f32>>, String> {
    let mut current_transform = Isometry::identity();
    let mut current_mse = f32::MAX;

    let mut transformed_points = points_a
        .iter()
        .map(|point_a| current_transform.transform_point(point_a))
        .collect::<Vec<_>>();
    for iteration_num in 0..max_iterations {
        let closest_points = transformed_points
            .iter()
            .map(|transformed_point_a| find_closest_point(transformed_point_a, points_b))
            .collect::<Vec<_>>();

        let (rot_mat, mean_a, mean_b) =
            transform_using_centeroids(transformed_points.as_slice(), closest_points.as_slice());

        let svd = rot_mat.svd(true, true);
        let rotation = svd.v_t.unwrap().transpose() * svd.u.unwrap().transpose();
        let translation = mean_b.coords - (rotation * mean_a.coords);

        current_transform = Isometry::from_parts(
            translation.into(),
            nalgebra::UnitQuaternion::from_matrix(&rotation),
        ) * current_transform;

        for (idx, point_a) in points_a.iter().enumerate() {
            transformed_points[idx] = current_transform.transform_point(point_a)
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

#[cfg(test)]
mod tests {
    #[cfg(any(feature = "2d", feature = "3d"))]
    use crate::utils;

    #[test]
    #[cfg(feature = "2d")]
    fn test_csm_2d() {
        let translation = nalgebra::Vector2::new(-0.8, 1.3);
        let isom = nalgebra::Isometry2::new(translation, 0.1);
        let (points, points_transformed) = utils::tests::generate_points(isom);

        assert!(super::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            100,
            0.0001
        )
        .is_ok());
    }

    #[test]
    #[cfg(feature = "3d")]
    fn test_sm_3d() {
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.5, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let (points, points_transformed) = utils::tests::generate_points(isom);

        assert!(super::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            100,
            0.0001
        )
        .is_ok());
    }
}
