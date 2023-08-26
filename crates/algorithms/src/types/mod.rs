use nalgebra::{AbstractRotation, ArrayStorage, Const, Isometry, Matrix};

/// A shorthand way of specifying a symmetrical [`Matrix`] of `N` size.
/// Most common usage of this is a matrix that [`SVD`](nalgebra::SVD) can be run on.
pub(crate) type SameSizeMat<const N: usize> =
    Matrix<f32, Const<N>, Const<N>, ArrayStorage<f32, N, N>>;

/// Contains the resulting transform, the resulting Mean Squared Error, and the number of iterations taken for a successful ICP convergence.
#[derive(Debug)]
pub struct ICPSuccess<const N: usize, R>
where
    R: AbstractRotation<f32, N>,
{
    /// An isometric matrix, containing the translation and rotation between the point sets.
    /// In 2D space, `R` would be a [`UnitComplex<f32>`](nalgebra::UnitComplex), in 3D space it would be a [`UnitQuaternion<f32>`](nalgebra::UnitQuaternion)
    pub transform: Isometry<f32, R, N>,
    /// Mean Squared Error, this is the distances between each point in `points_a` and its corresponding point in `points_b`,
    /// This can be used to determine whether the ICP converged correctly, or simply on its local minimum.
    pub mse: f32,
    /// The amount of iterations passed until convergence.
    pub iteration_num: usize,
}
