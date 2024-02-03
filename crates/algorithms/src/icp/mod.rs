use crate::{
    kd_tree::KDTree,
    types::IsometryAbstraction,
    utils::point_cloud::{downsample_point_cloud, find_closest_point},
    String, Sum, Vec,
};
use helpers::{calculate_mse, get_rotation_matrix_and_centeroids};
use nalgebra::{ComplexField, Point, RealField};
use num_traits::AsPrimitive;
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
    T: RealField + Copy + Default + Sum,
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
    log::trace!("Found nearest neighbours");

    let (rot_mat, mean_a, mean_b) =
        get_rotation_matrix_and_centeroids(transformed_points, &closest_points);
    log::trace!("Generated covariance matrix");

    *current_transform = O::update_transform(current_transform, mean_a, mean_b, &rot_mat);

    for (idx, point_a) in points_a.iter().enumerate() {
        transformed_points[idx] = O::transform_point(current_transform, point_a)
    }
    let new_mse = calculate_mse(transformed_points, closest_points.as_slice());
    log::trace!("New MSE: {new_mse}");

    // If the MSE difference is lower than the threshold, then this is as good as it gets
    if config
        .mse_threshold
        .map(|thres| new_mse < thres)
        .unwrap_or_default()
        || <T as ComplexField>::abs(*current_mse - new_mse) < config.mse_interval_threshold
    {
        return Ok(new_mse);
    }

    *current_mse = new_mse;
    Err((mean_a, mean_b))
}

/// A free-form version of the ICP function, allowing for any input and output, under the constraints of the function
///
/// # Arguments
/// * `points_a`: A slice of [`Point<T, N>`], representing the source point cloud."]
/// * `points_b`: A slice of [`Point<T, N>`], representing the target point cloud."]
/// * `max_iterations`: a [`usize`], specifying for how many iterations to try converging before returning an error."]
/// * `mse_threshold`: a `T`, if the MSE __differential__ is smaller than this number, we are considered converged[^convergence_note]."]
/// * `with_kd`: Whether to use a KDTree data structure in order to locate nearest points, the more points in your point cloud, the greater the benefit for this, smaller 3D point clouds may actually be better off without it."]
///
/// # Returns"]
/// An [`ICPSuccess`] struct with an [`Isometry`](nalgebra::Isometry) transform with a `T` precision, or an error message explaining what went wrong."]
///
/// [^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration."]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Full ICP Algorithm", skip_all)
)]
pub fn icp<T, const N: usize, O>(
    points_a: &[Point<T, N>],
    points_b: &[Point<T, N>],
    config: ICPConfiguration<T>,
) -> Result<ICPSuccess<T, N, O>, String>
where
    T: RealField + Copy + Default + Sum,
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

    if config.mse_interval_threshold < T::default_epsilon() {
        return Err(String::from(
            "MSE interval threshold too low, convergence impossible",
        ));
    }

    if config
        .mse_threshold
        .map(|thres| thres < T::default_epsilon())
        .unwrap_or_default()
    {
        return Err(String::from(
            "Absolute MSE threshold too low, convergence impossible",
        ));
    }

    let (downsampled_points_a, downsampled_points_b) = config
        .downsample_interval
        .map(|intervals| {
            let ret = (
                downsample_point_cloud(points_a, intervals),
                downsample_point_cloud(points_b, intervals),
            );
            log::trace!("Downsampled point clouds");
            ret
        })
        .unwrap_or((points_a.to_vec(), points_b.to_vec()));

    let mut points_to_transform = downsampled_points_a.to_vec();
    let target_points_tree = config
        .with_kd
        .then_some(KDTree::from(downsampled_points_b.as_slice()));
    let mut current_transform = O::identity();
    let mut current_mse = <T as RealField>::max_value().expect("Must have MAX");

    for iteration_num in 0..config.max_iterations {
        log::trace!(
            "Running iteration number {iteration_num}/{}",
            config.max_iterations
        );
        if let Ok(mse) = icp_iteration::<T, N, O>(
            &downsampled_points_a,
            &mut points_to_transform,
            &downsampled_points_b,
            target_points_tree.as_ref(),
            &mut current_transform,
            &mut current_mse,
            &config,
        ) {
            log::trace!("Converged after {iteration_num} iterations with an MSE of {mse}");
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
    ($precision:expr, $nd:expr) => {
        ::paste::paste! {
            #[doc = "An ICP algorithm in " $nd "D space."]
            #[doc = "# Arguments"]
            #[doc = "* `points_a`: A slice of [`Point<" $precision ", " $nd ">`](super::Point), representing the source point cloud."]
            #[doc = "* `points_b`: A slice of [`Point<" $precision ", " $nd ">`](super::Point), representing the target point cloud."]
            #[doc = "* `max_iterations`: a [`usize`], specifying for how many iterations to try converging before returning an error."]
            #[doc = "* `mse_threshold`: an `" $precision "`, if the MSE __differential__ is smaller than this number, we are considered converged[^convergence_note]."]
            #[doc = "* `with_kd`: Whether to use a KDTree data structure in order to locate nearest points, the more points in your point cloud, the greater the benefit for this, smaller 3D point clouds may actually be better off without it."]
            #[doc = ""]
            #[doc = "# Returns"]
            #[doc = "An [`ICPSuccess`](super::ICPSuccess) struct with an [`Isometry`](nalgebra::Isometry) transform with an `" $precision "` precision, or an error message explaining what went wrong."]
            #[doc = ""]
            #[doc = "[^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration."]
            pub fn [<icp_$nd d>](points_a: &[nalgebra::Point<$precision, $nd>],
                points_b: &[nalgebra::Point<$precision, $nd>],
                config: super::types::ICPConfiguration<$precision>) -> Result<super::ICPSuccess<$precision, $nd, nalgebra::Const<$nd>>, String> {
                    super::icp(points_a, points_b, config)
            }
        }
    };

    ($precision:expr, doc $doc:tt) => {
        ::paste::paste! {
            #[doc = "A " $doc "-precision implementation of a basic ICP algorithm"]
            pub mod $precision {
                impl_icp_algorithm!($precision, 2);
                impl_icp_algorithm!($precision, 3);
            }
        }
    }
}

#[cfg(feature = "pregenerated")]
impl_icp_algorithm!(f32, doc single);
#[cfg(feature = "pregenerated")]
impl_icp_algorithm!(f64, doc double);

#[cfg(test)]
mod tests {
    use crate::{
        icp::types::ICPConfiguration,
        utils::point_cloud::{generate_point_cloud, transform_point_cloud},
        String,
    };

    #[test]
    fn test_icp_errors() {
        let points = generate_point_cloud(10, -15.0..=15.0);
        let config = ICPConfiguration {
            with_kd: false,
            downsample_interval: None,
            max_iterations: 10,
            mse_threshold: None,
            mse_interval_threshold: 0.01,
        };

        let res = super::f32::icp_2d(&[], points.as_slice(), config);
        assert_eq!(
            res.unwrap_err(),
            String::from("Source point cloud is empty")
        );

        let res = super::f32::icp_2d(points.as_slice(), &[], config);
        assert_eq!(
            res.unwrap_err(),
            String::from("Target point cloud is empty")
        );

        let res = super::f32::icp_2d(
            points.as_slice(),
            points.as_slice(),
            ICPConfiguration {
                max_iterations: 0,
                ..config
            },
        );
        assert_eq!(
            res.unwrap_err(),
            String::from("Must have more than one iteration")
        );

        let res = super::f32::icp_2d(
            points.as_slice(),
            points.as_slice(),
            ICPConfiguration {
                mse_interval_threshold: 0.0,
                ..config
            },
        );
        assert_eq!(
            res.unwrap_err(),
            String::from("MSE interval threshold too low, convergence impossible")
        );

        let res = super::f32::icp_2d(
            points.as_slice(),
            points.as_slice(),
            ICPConfiguration {
                mse_threshold: Some(0.0),
                ..config
            },
        );
        assert_eq!(
            res.unwrap_err(),
            String::from("Absolute MSE threshold too low, convergence impossible")
        );
    }

    #[test]
    // This is for code coverage purposes, ensure that absolute MSE is used instead of interval
    fn test_icp_absolute_threshold() {
        let points = generate_point_cloud(100, -15.0..=15.0);
        let translation = nalgebra::Vector2::new(-0.8, 1.3);
        let isom = nalgebra::Isometry2::new(translation, 0.1);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: false,
                downsample_interval: None,
                max_iterations: 10,
                mse_threshold: Some(0.1),
                mse_interval_threshold: 0.001,
            },
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.1);
    }

    #[test]
    fn test_icp_2d() {
        let points = generate_point_cloud(100, -15.0..=15.0);
        let translation = nalgebra::Vector2::new(-0.8, 1.3);
        let isom = nalgebra::Isometry2::new(translation, 0.1);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: false,
                downsample_interval: None,
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.01);
    }

    #[test]
    fn test_icp_2d_with_kd() {
        let points = generate_point_cloud(100, -15.0..=15.0);
        let isom = nalgebra::Isometry2::new(nalgebra::Vector2::new(-0.8, 1.3), 0.1);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: true,
                downsample_interval: None,
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.01);
    }

    #[test]
    fn test_icp_2d_downsample() {
        let points = generate_point_cloud(100, -15.0..=15.0);
        let translation = nalgebra::Vector2::new(-0.8, 1.3);
        let isom = nalgebra::Isometry2::new(translation, 0.1);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: false,
                downsample_interval: Some(0.5),
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.01);
    }

    #[test]
    fn test_icp_2d_with_kd_downsample() {
        let points = generate_point_cloud(100, -15.0..=15.0);
        let isom = nalgebra::Isometry2::new(nalgebra::Vector2::new(-0.8, 1.3), 0.1);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: true,
                downsample_interval: Some(0.5),
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.01);
    }

    #[test]
    fn test_icp_3d() {
        let points = generate_point_cloud(500, -15.0..=15.0);
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.2, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: false,
                downsample_interval: None,
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.05);
    }

    #[test]
    fn test_icp_3d_with_kd() {
        let points = generate_point_cloud(500, -15.0..=15.0);
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.2, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: true,
                downsample_interval: None,
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.05);
    }

    #[test]
    fn test_icp_3d_downsample() {
        let points = generate_point_cloud(500, -15.0..=15.0);
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.2, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: false,
                downsample_interval: Some(0.5),
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.05);
    }

    #[test]
    fn test_icp_3d_with_kd_downsample() {
        let points = generate_point_cloud(500, -15.0..=15.0);
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.2, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let points_transformed = transform_point_cloud(&points, isom);

        let res = super::f32::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            ICPConfiguration {
                with_kd: true,
                downsample_interval: Some(0.5),
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
            },
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.05);
    }
}
