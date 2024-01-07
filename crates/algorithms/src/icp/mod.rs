use crate::{
    kd_tree::KDTree,
    types::{ICPSuccess, IsometryAbstration},
    utils::find_closest_point,
};
use helpers::{calculate_mse, transform_using_centeroids};
use nalgebra::{Point, RealField};
use num_traits::AsPrimitive;

#[cfg(not(feature = "std"))]
use {
    alloc::{string::String, vec::Vec},
    core::{default::Default, fmt::Debug, iter::Sum, ops::SubAssign},
    num_traits::float::FloatCore as Float,
};
#[cfg(feature = "std")]
use {
    num_traits::Float,
    std::{default::Default, fmt::Debug, iter::Sum, ops::SubAssign, string::String, vec::Vec},
};

#[cfg(feature = "tracing")]
use tracing::instrument;

mod helpers;

#[cfg_attr(feature = "tracing", instrument("Full ICP ALgorithm", skip_all))]
fn icp<T, const N: usize, O>(
    points_a: &[Point<T, N>],
    points_b: &[Point<T, N>],
    max_iterations: usize,
    mse_threshold: T,
    with_kd: bool,
) -> Result<ICPSuccess<T, N, O>, String>
where
    T: 'static + Float + Debug + SubAssign + Default + RealField + Sum,
    usize: AsPrimitive<T>,
    O: IsometryAbstration<T, N>,
{
    if points_a.is_empty() {
        return Err(String::from("Source point cloud is empty"));
    }

    if points_b.is_empty() {
        return Err(String::from("Target point cloud is empty"));
    }

    let mut current_transform: O::Isom = O::identity();
    let mut current_mse = <T as RealField>::max_value().expect("Must have MAX");

    let mut transformed_points = points_a
        .iter()
        .map(|point_a| O::transform_point(&current_transform, point_a))
        .collect::<Vec<_>>();

    let target_points_tree = with_kd.then_some(KDTree::from(points_b));
    for iteration_num in 0..max_iterations {
        let closest_points = transformed_points
            .iter()
            .map(|transformed_point_a| {
                target_points_tree
                    .as_ref()
                    .and_then(|kd_tree| kd_tree.nearest(transformed_point_a))
                    .unwrap_or(find_closest_point(transformed_point_a, points_b))
            })
            .collect::<Vec<_>>();

        let (rot_mat, mean_a, mean_b) =
            transform_using_centeroids(transformed_points.as_slice(), closest_points.as_slice());

        current_transform = O::update_transform(&current_transform, mean_a, mean_b, &rot_mat);

        for (idx, point_a) in points_a.iter().enumerate() {
            transformed_points[idx] = O::transform_point(&current_transform, point_a)
        }
        let new_mse = calculate_mse(transformed_points.as_slice(), closest_points.as_slice());

        if Float::abs(current_mse - new_mse) < mse_threshold {
            return Ok(ICPSuccess {
                transform: current_transform,
                mse: new_mse,
                iteration_num,
            });
        }

        current_mse = new_mse;
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
                max_iterations: usize,
                mse_threshold: $prec,
                with_kd: bool) -> Result<ICPSuccess<$prec, $nd, nalgebra::Const<$nd>>, String> {
                    icp(points_a, points_b, max_iterations, mse_threshold, with_kd)
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
    use crate::utils;

    #[test]
    fn test_icp_2d() {
        let translation = nalgebra::Vector2::new(-0.8, 1.3);
        let isom = nalgebra::Isometry2::new(translation, 0.1);
        let (points, points_transformed) = utils::tests::generate_points(5000, isom);

        assert!(super::f32::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            100,
            0.0001,
            false
        )
        .is_ok());
    }

    #[test]
    fn test_icp_2d_with_kd() {
        let translation = nalgebra::Vector2::new(-0.8, 1.3);
        let isom = nalgebra::Isometry2::new(translation, 0.1);
        let (points, points_transformed) = utils::tests::generate_points(5000, isom);

        assert!(super::f32::icp_2d(
            points.as_slice(),
            points_transformed.as_slice(),
            100,
            0.0001,
            true
        )
        .is_ok());
    }

    #[test]
    fn test_sm_3d() {
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.5, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let (points, points_transformed) = utils::tests::generate_points(5000, isom);

        assert!(super::f32::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            100,
            0.0001,
            false
        )
        .is_ok());

        assert!(super::f32::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            100,
            0.0001,
            true
        )
        .is_ok());
    }

    #[test]
    fn test_sm_3d_with_kd() {
        let translation = nalgebra::Vector3::new(-0.8, 1.3, 0.2);
        let rotation = nalgebra::Vector3::new(0.1, 0.5, -0.21);
        let isom = nalgebra::Isometry3::new(translation, rotation);
        let (points, points_transformed) = utils::tests::generate_points(5000, isom);

        assert!(super::f32::icp_3d(
            points.as_slice(),
            points_transformed.as_slice(),
            100,
            0.0001,
            true
        )
        .is_ok());
    }
}
