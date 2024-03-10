use crate::{
    kd_tree::KDTree,
    types::{AbstractIsometry, IsometryAbstractor},
    utils::point_cloud::find_closest_point,
    Sum, Vec,
};
use helpers::{calculate_mse, get_rotation_matrix_and_centeroids};
use nalgebra::{ComplexField, Isometry, Point, RealField, SimdRealField};
use num_traits::{AsPrimitive, Bounded};
use types::{ICPConfiguration, ICPSuccess};

mod helpers;

/// Structs in use as part of the public API of the ICP algorithm.
pub mod types;

/// A single iteration of the ICP function, allowing for any input and output, usually used for debugging or visualization
///
/// # Arguments
/// * `points_a`: A slice of [`Point<T, N>`], representing the source point cloud.
/// * `transformed_points`: A mutable slice of [`Point<T, N>`], representing the transformed source point cloud, this will be transformed further by the function.
/// * `points_b`: A slice of [`Point<T, N>`], representing the target point cloud.
/// * `target_points_tree`: An [`Option<KDTree<T, N>>`], this is usually created by the ICP function if `config.use_kd` is `true`
/// * `current_transform`: A mutable reference to the [`Isometry`] used to transform the source points, this will gradually change with each iteration.
/// * `current_mse`: A mutable reference of a `T`, this will be updated by the function to the latest MSE, which is then used by the ICP function to determine an exit strategy.
/// * `config`: a reference to an [`ICPConfiguration`], specifying the behaviour of the algorithm.
///
/// # Generics
/// * `T`: Either [`prim@f32`] or [`prim@f64`].
/// * `R`: Either a [`UnitComplex`](nalgebra::UnitComplex) or a [`UnitQuaternion`](nalgebra::UnitQuaternion) of `T`, depednding on `N`.
/// * `N`: a usize, either `2` or `3`.
///
/// # Returns
/// An [`ICPSuccess`] struct with an [`Isometry`] transform with a `T` precision, or an error message explaining what went wrong.
///
/// [^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration.

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("ICP Algorithm Iteration", skip_all)
)]
pub fn icp_iteration<T, const N: usize>(
    points_a: &[Point<T, N>],
    transformed_points: &mut [Point<T, N>],
    points_b: &[Point<T, N>],
    target_points_tree: Option<&KDTree<T, N>>,
    current_transform: &mut Isometry<
        T,
        <IsometryAbstractor<T, N> as AbstractIsometry<T, N>>::RotType,
        N,
    >,
    current_mse: &mut T,
    config: &ICPConfiguration<T>,
) -> Result<T, (Point<T, N>, Point<T, N>)>
where
    T: Bounded + Copy + Default + RealField + Sum + SimdRealField,
    usize: AsPrimitive<T>,
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
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

    *current_transform =
        IsometryAbstractor::<T, N>::update_transform(current_transform, mean_a, mean_b, &rot_mat);

    for (idx, point_a) in points_a.iter().enumerate() {
        transformed_points[idx] = current_transform.transform_point(point_a);
    }
    let new_mse = calculate_mse(transformed_points, closest_points.as_slice());
    log::trace!("New MSE: {new_mse}");

    // If the MSE difference is lower than the threshold, then this is as good as it gets
    if config
        .mse_absolute_threshold
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
/// * `points_a`: A slice of [`Point<T, N>`], representing the source point cloud.
/// * `points_b`: A slice of [`Point<T, N>`], representing the target point cloud.
/// * `config`: a reference to an [`ICPConfiguration<T>`], specifying the behaviour of the algorithm.
///
/// # Generics
/// * `T`: Either [`prim@f32`] or [`prim@f64`]
/// * `R`: An ['SVDIsometry'] of `T` and `N`
/// * `N`: a usize, either `2` or `3`
///
/// # Returns
/// An [`ICPSuccess`] struct with an [`Isometry`] transform with a `T` precision, or an error message explaining what went wrong.
///
/// [^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Full ICP Algorithm", skip_all)
)]
pub fn icp<T, const N: usize>(
    points_a: &[Point<T, N>],
    points_b: &[Point<T, N>],
    config: ICPConfiguration<T>,
) -> Result<
    ICPSuccess<T, <IsometryAbstractor<T, N> as AbstractIsometry<T, N>>::RotType, N>,
    &'static str,
>
where
    T: Bounded + Copy + Default + RealField + Sum,
    usize: AsPrimitive<T>,
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
{
    if points_a.is_empty() {
        return Err("Source point cloud is empty");
    }

    if points_b.is_empty() {
        return Err("Target point cloud is empty");
    }

    if config.max_iterations == 0 {
        return Err("Must have more than one iteration");
    }

    if config.mse_interval_threshold < T::default_epsilon() {
        return Err("MSE interval threshold too low, convergence impossible");
    }

    if config
        .mse_absolute_threshold
        .map(|thres| thres < T::default_epsilon())
        .unwrap_or_default()
    {
        return Err("Absolute MSE threshold too low, convergence impossible");
    }

    let mut points_to_transform = points_a.to_vec();
    let target_points_tree = config.use_kd_tree.then_some(KDTree::from(points_b));
    let mut current_transform = Isometry::identity();
    let mut current_mse = <T as Bounded>::max_value();

    for iteration_num in 0..config.max_iterations {
        log::trace!(
            "Running iteration number {iteration_num}/{}",
            config.max_iterations
        );
        if let Ok(mse) = icp_iteration::<T, N>(
            points_a,
            &mut points_to_transform,
            points_b,
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

    Err("Could not converge")
}

#[cfg(feature = "pregenerated")]
macro_rules! impl_icp_algorithm {
    ($precision:expr, $nd:expr, $rot_type:expr) => {
        ::paste::paste! {
            #[doc = "An ICP algorithm in " $nd "D space."]
            #[doc = "# Arguments"]
            #[doc = "* `points_a`: A slice of [`Point<" $precision ", " $nd ">`](super::Point), representing the source point cloud."]
            #[doc = "* `points_b`: A slice of [`Point<" $precision ", " $nd ">`](super::Point), representing the target point cloud."]
            #[doc = "* `config`: a reference to an [`ICPConfiguration`](super::ICPConfiguration), specifying the behaviour of the algorithm."]
            #[doc = ""]
            #[doc = "# Returns"]
            #[doc = "An [`ICPSuccess`](super::ICPSuccess) struct with an [`Isometry`](nalgebra::Isometry) transform with an `" $precision "` precision, or an error message explaining what went wrong."]
            #[doc = ""]
            #[doc = "[^convergence_note]: This does not guarantee that the transformation is correct, only that no further benefit can be gained by running another iteration."]
            pub fn [<icp_$nd d>](points_a: &[nalgebra::Point<$precision, $nd>],
                points_b: &[nalgebra::Point<$precision, $nd>],
                config: super::types::ICPConfiguration<$precision>) -> Result<super::ICPSuccess<$precision, nalgebra::$rot_type<$precision>, $nd>, &'static str> {
                    super::icp(points_a, points_b, config)
            }
        }
    };

    ($precision:expr, doc $doc:tt) => {
        ::paste::paste! {
            #[doc = "A " $doc "-precision implementation of a basic ICP algorithm"]
            pub mod $precision {
                impl_icp_algorithm!($precision, 2, UnitComplex);
                impl_icp_algorithm!($precision, 3, UnitQuaternion);
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
    };

    #[test]
    fn test_icp_errors() {
        let points = generate_point_cloud(10, -15.0..=15.0);
        let config_builder = ICPConfiguration::builder();

        let res = super::f32::icp_2d(&[], points.as_slice(), config_builder.build());
        assert_eq!(res.unwrap_err(), "Source point cloud is empty");

        let res = super::f32::icp_2d(points.as_slice(), &[], config_builder.build());
        assert_eq!(res.unwrap_err(), "Target point cloud is empty");

        let res = super::f32::icp_2d(
            points.as_slice(),
            points.as_slice(),
            config_builder.with_max_iterations(0).build(),
        );
        assert_eq!(res.unwrap_err(), "Must have more than one iteration");

        let res = super::f32::icp_2d(
            points.as_slice(),
            points.as_slice(),
            config_builder.with_mse_interval_threshold(0.0).build(),
        );
        assert_eq!(
            res.unwrap_err(),
            "MSE interval threshold too low, convergence impossible"
        );

        let res = super::f32::icp_2d(
            points.as_slice(),
            points.as_slice(),
            config_builder
                .with_absolute_mse_threshold(Some(0.0))
                .build(),
        );
        assert_eq!(
            res.unwrap_err(),
            "Absolute MSE threshold too low, convergence impossible"
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
            ICPConfiguration::builder()
                .with_max_iterations(10)
                .with_absolute_mse_threshold(Some(0.1))
                .with_mse_interval_threshold(0.001)
                .build(),
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
            ICPConfiguration::builder()
                .with_max_iterations(10)
                .with_mse_interval_threshold(0.01)
                .build(),
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
            ICPConfiguration::builder()
                .with_kd_tree(true)
                .with_max_iterations(50)
                .with_mse_interval_threshold(0.01)
                .build(),
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
            ICPConfiguration::builder()
                .with_max_iterations(50)
                .with_mse_interval_threshold(0.01)
                .build(),
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
            ICPConfiguration::builder()
                .with_kd_tree(true)
                .with_max_iterations(50)
                .with_mse_interval_threshold(0.01)
                .build(),
        );
        assert!(res.is_ok());
        assert!(res.unwrap().mse < 0.05);
    }
}
