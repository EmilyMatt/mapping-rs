use nalgebra::{
    AbstractRotation, ArrayStorage, ComplexField, Const, Isometry, Matrix, RealField,
    SimdComplexField, SimdRealField,
};
use std::fmt::Debug;

/// A shorthand way of specifying a symmetrical [`Matrix`] of `N` size.
/// Most common usage of this is a matrix that [`SVD`](nalgebra::SVD) can be run on.
pub(crate) type SameSizeMat<T, const N: usize> =
    Matrix<T, Const<N>, Const<N>, ArrayStorage<T, N, N>>;

/// Contains the resulting transform, the resulting Mean Squared Error, and the number of iterations taken for a successful ICP convergence.
#[derive(Debug)]
pub struct ICPSuccess<T, const N: usize, R>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField,
    R: AbstractRotation<T, N>,
{
    /// An isometric matrix, containing the translation and rotation between the point sets.
    /// In 2D space, `R` would be a [`UnitComplex<f32>`](nalgebra::UnitComplex), in 3D space it would be a [`UnitQuaternion<f32>`](nalgebra::UnitQuaternion)
    pub transform: Isometry<T, R, N>,
    /// Mean Squared Error, this is the distances between each point in `points_a` and its corresponding point in `points_b`,
    /// This can be used to determine whether the ICP converged correctly, or simply on its local minimum.
    pub mse: T,
    /// The amount of iterations passed until convergence.
    pub iteration_num: usize,
}
