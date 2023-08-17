use crate::{
    types::{ConvergenceIntermediateStep, ConvergenceResult, ICPResult, ICPSuccess},
    utils::{get_mean_point_nd, get_point_cloud_centeroid_nd, transform_points_nd},
};
use helpers::{calculate_mse, get_closest_points};
use nalgebra::{
    AbstractRotation, Const, Dyn, Isometry, Isometry2, Isometry3, Matrix, Translation, UnitComplex,
    UnitQuaternion, VecStorage, SVD,
};

mod helpers;

#[inline]
fn icp_iteration<R, const N: usize>(
    points_a: &Matrix<f32, Const<N>, Dyn, VecStorage<f32, Const<N>, Dyn>>,
    points_b: &Matrix<f32, Const<N>, Dyn, VecStorage<f32, Const<N>, Dyn>>,
    current_isom: Isometry<f32, R, N>,
    current_mse: &mut f32,
    mse_threshold: f32,
) -> ConvergenceResult<N>
where
    R: AbstractRotation<f32, N>,
{
    let transformed_points_a = transform_points_nd(points_a, current_isom);
    let closest_points = get_closest_points(points_a, points_b);

    // Mean squared sum
    let new_mse = calculate_mse(&transformed_points_a, &closest_points);
    if (new_mse - *current_mse) < mse_threshold {
        // We've converged as much as we could, might as well stop here
        return ConvergenceResult::Converged;
    }
    *current_mse = new_mse;

    let transformed_points_a_mean = get_mean_point_nd(&transformed_points_a);
    let closest_points_mean = get_mean_point_nd(&closest_points);

    let transformed_points_a_centeroid =
        get_point_cloud_centeroid_nd(&transformed_points_a, transformed_points_a_mean);
    let closest_points_centeroid =
        get_point_cloud_centeroid_nd(&closest_points, closest_points_mean);

    ConvergenceResult::NotConverged(ConvergenceIntermediateStep {
        translation: transformed_points_a_mean - closest_points_mean,
        rotation: transformed_points_a_centeroid * closest_points_centeroid.transpose(),
    })
}

// TODO: prob implement this using a trait, and a macro

pub fn icp_2d(
    points_a: &Matrix<f32, Const<2>, Dyn, VecStorage<f32, Const<2>, Dyn>>,
    points_b: &Matrix<f32, Const<2>, Dyn, VecStorage<f32, Const<2>, Dyn>>,
    max_iterations: usize,
    mse_threshold: f32,
) -> ICPResult<2, UnitComplex<f32>> {
    let mut current_isom = Isometry2::identity();
    let mut current_mse = f32::INFINITY;

    for iteration in 0..max_iterations {
        let icp_result = icp_iteration(
            points_a,
            points_b,
            current_isom,
            &mut current_mse,
            mse_threshold,
        );

        match icp_result {
            ConvergenceResult::Converged => {
                return ICPResult::Ok(ICPSuccess {
                    transform: current_isom,
                    mse: current_mse,
                    iteration_num: iteration,
                })
            }
            ConvergenceResult::NotConverged(intermediate_result) => {
                let svd = SVD::new(intermediate_result.rotation, true, true);

                let rotation_matrix = svd.u.unwrap().transpose() * svd.v_t.unwrap().transpose();
                current_isom = Isometry2::from_parts(
                    Translation::from(intermediate_result.translation),
                    UnitComplex::from_matrix(&rotation_matrix),
                ) * current_isom;
            }
        }
    }
    ICPResult::Err("Could not converge".to_string())
}

pub fn icp_3d(
    points_a: &Matrix<f32, Const<3>, Dyn, VecStorage<f32, Const<3>, Dyn>>,
    points_b: &Matrix<f32, Const<3>, Dyn, VecStorage<f32, Const<3>, Dyn>>,
    max_iterations: usize,
    mse_threshold: f32,
) -> ICPResult<3, UnitQuaternion<f32>> {
    let mut current_isom = Isometry3::identity();
    let mut current_mse = f32::INFINITY;

    for iteration in 0..max_iterations {
        let icp_result = icp_iteration(
            points_a,
            points_b,
            current_isom,
            &mut current_mse,
            mse_threshold,
        );

        match icp_result {
            ConvergenceResult::Converged => {
                return ICPResult::Ok(ICPSuccess {
                    transform: current_isom,
                    mse: current_mse,
                    iteration_num: iteration,
                })
            }
            ConvergenceResult::NotConverged(intermediate_result) => {
                let svd = SVD::new(intermediate_result.rotation, true, true);

                let rotation_matrix = svd.u.unwrap().transpose() * svd.v_t.unwrap().transpose();
                current_isom = Isometry3::from_parts(
                    Translation::from(intermediate_result.translation),
                    UnitQuaternion::from_matrix(&rotation_matrix),
                ) * current_isom;
            }
        }
    }
    ICPResult::Err("Could not converge".to_string())
}

#[cfg(test)]
mod tests {
    use crate::{
        icp::{icp_2d, icp_3d},
        utils::test_utils::generate_points,
    };
    use nalgebra::{Isometry2, Isometry3, Vector2, Vector3};

    #[test]
    fn test_icp_2d() {
        let translation = Vector2::new(-0.8, 1.3);
        let rotation = 0.02;
        let isom = Isometry2::new(translation, rotation);
        let (points, points_transformed) = generate_points(isom);

        let icp_result = icp_2d(&points, &points_transformed, 20, 0.001);
        eprintln!("{icp_result:?}");
    }

    #[test]
    fn test_icp_3d() {
        let translation = Vector3::new(-0.8, 1.3, 0.2);
        let rotation = Vector3::new(0.01, 0.001, 0.0);
        let isom = Isometry3::new(translation, rotation);
        let (points, points_transformed) = generate_points(isom);

        let icp_result = icp_3d(&points, &points_transformed, 20, 0.001);
        eprintln!("{icp_result:?}");
    }
}
