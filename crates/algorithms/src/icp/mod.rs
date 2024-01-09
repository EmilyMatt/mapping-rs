use crate::{
    kd_tree::KDTree,
    types::IsometryAbstraction,
    utils::point_cloud::{downsample_point_cloud, find_closest_point},
    String, Sum, Vec,
};
use helpers::{calculate_mse, get_rotation_matrix_and_centeroids};
use nalgebra::{Point, RealField, SimdRealField};
use num_traits::{AsPrimitive, Float};
use types::{ICPConfiguration, ICPSuccess};

mod helpers;
/// Structs in use as part of the public API of the ICP algorithm.
pub mod types;

/// A single ICP iteration in `N`D space
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("ICP Algorithm Iteration", skip_all)
)]
pub fn icp_iteration<T, const N: usize, O>(
    points_a: &[Point<T, N>],
    transformed_points: &mut [Point<T, N>],
    points_b: &[Point<T, N>],
    target_points_tree: Option<&KDTree<T, N>>,
    current_transform: &mut O::IsometryType,
    current_mse: &mut T,
    config: &ICPConfiguration<T>,
) -> Result<T, (Point<T, N>, Point<T, N>)>
where
    T: RealField + SimdRealField + Copy + Default + Sum + Float,
    usize: AsPrimitive<T>,
    O: IsometryAbstraction<T, N>,
{
    let closest_points = transformed_points
        .iter()
        .map(|transformed_point_a| {
            target_points_tree
                .and_then(|kd_tree| kd_tree.nearest(transformed_point_a))
                .unwrap_or(find_closest_point(transformed_point_a, points_b))
        })
        .collect::<Vec<_>>();

    let (rot_mat, mean_a, mean_b) =
        get_rotation_matrix_and_centeroids(transformed_points, &closest_points);

    *current_transform = O::update_transform(current_transform, mean_a, mean_b, &rot_mat);

    for (idx, point_a) in points_a.iter().enumerate() {
        transformed_points[idx] = O::transform_point(current_transform, point_a)
    }
    let new_mse = calculate_mse(transformed_points, closest_points.as_slice());

    // If the MSE difference is lower than the threshold, then this is as good as it gets
    if config
        .mse_threshold
        .map(|thres| new_mse < thres)
        .unwrap_or_default()
        || Float::abs(*current_mse - new_mse) < config.mse_interval_threshold
    {
        return Ok(new_mse);
    }

    *current_mse = new_mse;
    Err((mean_a, mean_b))
}

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Full ICP Algorithm", skip_all)
)]
fn icp<T, const N: usize, O>(
    points_a: &[Point<T, N>],
    points_b: &[Point<T, N>],
    config: ICPConfiguration<T>,
) -> Result<ICPSuccess<T, N, O>, String>
where
    T: RealField + SimdRealField + Copy + Default + Float + Sum,
    usize: AsPrimitive<T>,
    O: IsometryAbstraction<T, N>,
{
    if points_a.is_empty() {
        return Err(String::from("Source point cloud is empty"));
    }

    if points_b.is_empty() {
        return Err(String::from("Target point cloud is empty"));
    }

    if config.max_iterations == 0 {
        return Err(String::from("Must have more than one iteration"));
    }

    if config
        .mse_threshold
        .map(|thres| thres <= T::epsilon())
        .unwrap_or_default()
    {
        return Err(String::from(
            "MSE Threshold too low, convergence impossible",
        ));
    }

    let downsampled_points_a = downsample_point_cloud(
        points_a,
        T::from_f32(0.1).expect("T must be a floating point"),
    );
    let downsampled_points_b = downsample_point_cloud(
        points_b,
        T::from_f32(0.1).expect("T must be a floating point"),
    );

    let mut transformed_points = downsampled_points_a.to_vec();
    let target_points_tree = config
        .with_kd
        .then_some(KDTree::from(downsampled_points_b.as_slice()));
    let mut current_transform = O::identity();
    let mut current_mse = <T as RealField>::max_value().expect("Must have MAX");

    for iteration_num in 0..config.max_iterations {
        if let Ok(mse) = icp_iteration::<T, N, O>(
            &downsampled_points_a,
            &mut transformed_points,
            &downsampled_points_b,
            target_points_tree.as_ref(),
            &mut current_transform,
            &mut current_mse,
            &config,
        ) {
            return Ok(ICPSuccess {
                transform: current_transform,
                mse,
                iteration_num,
            });
        }
    }

    Err(String::from("Could not converge"))
}

macro_rules! impl_icp_algorithm {
    ($nd: expr, $prec:expr) => {
        ::paste::paste! {
            #[doc = "An ICP algorithm in " $nd "D space."]
            #[doc = "# Arguments"]
            #[doc = "* `points_a`: A slice of [`Point<" $prec ", " $nd ">`], representing the source point cloud."]
            #[doc = "* `points_b`: A slice of [`Point<" $prec ", " $nd ">`], representing the target point cloud."]
            #[doc = "* `max_iterations`: a [`usize`], specifying for how many iterations to try converging before returning an error."]
            #[doc = "* `mse_threshold`: an `" $prec "`, if the MSE __differential__ is smaller than this number, we are considered converged[^convergence_note]."]
            #[doc = "* `with_kd`: Whether to use a KDTree data structure in order to locate nearest points, the more points in your point cloud, the greater the benefit for this, smaller 3D point clouds may actually be better off without it."]
            #[doc = ""]
            #[doc = "# Returns"]
            #[doc = "An [`ICPSuccess`] struct with an [`Isometry`](nalgebra::Isometry) transform with an `" $prec "` precision, or an error message explaining what went wrong."]
            #[doc = ""]
            #[doc = "[^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration."]
            pub fn [<icp _$nd d>](points_a: &[Point<$prec, $nd>],
                points_b: &[Point<$prec, $nd>],
                config: ICPConfiguration<$prec>) -> Result<ICPSuccess<$prec, $nd, nalgebra::Const<$nd>>, String> {
                    icp(points_a, points_b, config)
            }
        }
    };
}

/// A single-precision implementation of a basic ICP algorithm.
pub mod f32 {
    use super::*;
    impl_icp_algorithm!(2, f32);
    impl_icp_algorithm!(3, f32);
}

/// A double-precision implementation of a basic ICP algorithm.
pub mod f64 {
    use super::*;
    impl_icp_algorithm!(2, f64);
    impl_icp_algorithm!(3, f64);
}

#[cfg(test)]
mod tests {
    use crate::{
        icp::types::ICPConfiguration,
        utils::point_cloud::{generate_point_cloud, transform_point_cloud},
    };

    #[test]
    fn test_icp_2d() {
        let points = generate_point_cloud(1000, -15.0..=15.0);
        let translation = nalgebra::Vector2::new(-0.8, 1.3);
        let isom = nalgebra::Isometry2::new(translation, 0.1);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: false,
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
    }

    #[test]
    fn test_icp_2d_with_kd() {
        let points = generate_point_cloud(1000, -15.0..=15.0);
        let isom = nalgebra::Isometry2::new(nalgebra::Vector2::new(-0.8, 1.3), 0.1);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: true,
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
    }

    #[test]
    fn test_sm_3d() {
        let points = generate_point_cloud(5000, -15.0..=15.0);
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.5, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: false,
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
    }

    #[test]
    fn test_sm_3d_with_kd() {
        let points = generate_point_cloud(5000, -15.0..=15.0);
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.5, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: true,
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
    }
}
