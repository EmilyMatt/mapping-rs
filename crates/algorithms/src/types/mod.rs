use nalgebra::{
    ArrayStorage, ComplexField, Const, Isometry, Matrix, OMatrix, Point, RealField,
    SimdComplexField, SimdRealField, UnitComplex, UnitQuaternion, U1,
};
use std::fmt::Debug;

/// A shorthand way of specifying a symmetrical [`Matrix`] of `N` size.
/// Most common usage of this is a matrix that [`SVD`](nalgebra::SVD) can be run on.
pub(crate) type SameSizeMat<T, const N: usize> = OMatrix<T, Const<N>, Const<N>>;

/// Contains the resulting transform, the resulting Mean Squared Error, and the number of iterations taken for a successful ICP convergence.
#[derive(Debug)]
pub struct ICPSuccess<T, const N: usize, O>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
    O: IsometryAbstration<T, N>,
{
    /// An isometric matrix, containing the translation and rotation between the point sets.
    /// In 2D space, `R` would be a [`UnitComplex<f32>`](UnitComplex), in 3D space it would be a [`UnitQuaternion<f32>`](UnitQuaternion)
    pub transform: O::Isom,
    /// Mean Squared Error, this is the distances between each point in `points_a` and its corresponding point in `points_b`,
    /// This can be used to determine whether the ICP converged correctly, or simply on its local minimum.
    pub mse: T,
    /// The amount of iterations passed until convergence.
    pub iteration_num: usize,
}

pub trait IsometryAbstration<T, const N: usize>
where
    T: ComplexField + Copy + Default + RealField,
{
    type Isom;
    fn from_parts(
        translation: Matrix<T, Const<N>, U1, ArrayStorage<T, N, 1>>,
        mat: &SameSizeMat<T, N>,
    ) -> Self::Isom;
    fn svd(mat: &SameSizeMat<T, N>) -> (SameSizeMat<T, N>, SameSizeMat<T, N>);
    fn identity() -> Self::Isom;
    fn transform_point(isom: &Self::Isom, point: &Point<T, N>) -> Point<T, N>;
    fn update_transform(
        old_transform: &Self::Isom,
        translation: Matrix<T, Const<N>, U1, ArrayStorage<T, N, 1>>,
        mat: &SameSizeMat<T, N>,
    ) -> Self::Isom;
}

impl<T> IsometryAbstration<T, 2> for Const<2>
where
    T: ComplexField + Copy + Default + RealField,
{
    type Isom = Isometry<T, UnitComplex<T>, 2>;
    fn from_parts(
        translation: Matrix<T, Const<2>, U1, ArrayStorage<T, 2, 1>>,
        mat: &SameSizeMat<T, 2>,
    ) -> Self::Isom {
        Self::Isom::from_parts(translation.into(), UnitComplex::from_matrix(mat))
    }

    #[inline]
    fn svd(mat: &SameSizeMat<T, 2>) -> (SameSizeMat<T, 2>, SameSizeMat<T, 2>) {
        let svd = mat.svd(true, true);
        (svd.u.unwrap(), svd.v_t.unwrap())
    }

    #[inline]
    fn identity() -> Self::Isom {
        Self::Isom::identity()
    }

    #[inline]
    fn transform_point(isom: &Self::Isom, point: &Point<T, 2>) -> Point<T, 2> {
        isom.transform_point(point)
    }

    #[inline]
    fn update_transform(
        old_transform: &Self::Isom,
        translation: Matrix<T, Const<2>, U1, ArrayStorage<T, 2, 1>>,
        mat: &SameSizeMat<T, 2>,
    ) -> Self::Isom {
        Self::Isom::from_parts(translation.into(), UnitComplex::from_matrix(mat)) * old_transform
    }
}

impl<T> IsometryAbstration<T, 3> for Const<3>
where
    T: ComplexField + Copy + Default + RealField,
{
    type Isom = Isometry<T, UnitQuaternion<T>, 3>;

    #[inline]
    fn from_parts(
        translation: Matrix<T, Const<3>, U1, ArrayStorage<T, 3, 1>>,
        mat: &SameSizeMat<T, 3>,
    ) -> Self::Isom {
        Self::Isom::from_parts(translation.into(), UnitQuaternion::from_matrix(mat))
    }

    #[inline]
    fn svd(mat: &SameSizeMat<T, 3>) -> (SameSizeMat<T, 3>, SameSizeMat<T, 3>) {
        let svd = mat.svd(true, true);
        (svd.u.unwrap(), svd.v_t.unwrap())
    }

    #[inline]
    fn identity() -> Self::Isom {
        Self::Isom::identity()
    }

    #[inline]
    fn transform_point(isom: &Self::Isom, point: &Point<T, 3>) -> Point<T, 3> {
        isom.transform_point(point)
    }

    #[inline]
    fn update_transform(
        old_transform: &Self::Isom,
        translation: Matrix<T, Const<3>, U1, ArrayStorage<T, 3, 1>>,
        mat: &SameSizeMat<T, 3>,
    ) -> Self::Isom {
        Self::Isom::from_parts(translation.into(), UnitQuaternion::from_matrix(mat)) * old_transform
    }
}
