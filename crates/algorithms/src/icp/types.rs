use crate::{types::IsometryAbstraction, Debug};
use nalgebra::{RealField, SimdRealField};

/// Contains the resulting transform, the resulting Mean Squared Error, and the number of iterations taken for a successful ICP convergence.
#[derive(Debug)]
pub struct ICPSuccess<T, const N: usize, O>
where
    T: RealField + SimdRealField + Default + Copy,
    O: IsometryAbstraction<T, N>,
{
    /// An isometric matrix, containing the translation and rotation between the point sets.
    /// In 2D space, [`IsometryAbstraction::IsometryType`] would be a [`nalgebra::UnitComplex<T>`](nalgebra::UnitComplex), in 3D space it would be a [`nalgebra::UnitQuaternion<T>`](nalgebra::UnitQuaternion).
    pub transform: O::IsometryType,
    /// Mean Squared Error, this is the distances between each point in `points_a` and its corresponding point in `points_b`,
    /// This can be used to determine whether the ICP converged correctly, or simply on its local minimum.
    pub mse: T,
    /// The amount of iterations passed until convergence.
    pub iteration_num: usize,
}

/// A struct specifying configuration options for an ICP algorithm.
#[derive(Copy, Clone, Debug, Default)]
pub struct ICPConfiguration<T> {
    /// Whether to use a KDTree structure to find nearest neighbours, becomes increasingly effective with point cloud growth.
    pub with_kd: bool,
    /// Whether to downsample to point cloud to maintain while maintaining the following intervals between points
    pub downsample_interval: Option<T>,
    /// The amount of iterations before giving up and exiting the algorithm.
    pub max_iterations: usize,
    /// When provided, the algorithm will consider itself converged when the MSE is smaller than the given value, without any more iterations.
    pub mse_threshold: Option<T>,
    /// This will specify the interval between iteration MSE's than when reached, will declare ICP convergence.
    pub mse_interval_threshold: T,
}
