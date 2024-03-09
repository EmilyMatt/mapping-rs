use crate::{ops::RangeInclusive, utils::verify_rotation_matrix_determinant};
use nalgebra::{
    AbstractRotation, ArrayStorage, Const, Isometry, Matrix, Point, RealField, UnitComplex,
    UnitQuaternion,
};

/// A shorthand way of specifying a symmetrical [`Matrix`](Matrix) of `N` size.
/// Kind of similiar to nalgebra's [`nalgebra::SquareMatrix`] but simpler for our usecase
pub(crate) type SameSizeMat<T, const N: usize> =
    Matrix<T, Const<N>, Const<N>, ArrayStorage<T, N, N>>;

/// This trait acts as an abstraction for the process of creating an [`Isometry`] matrix from a NxN SVD,
/// without having to constrain every function by a million traits.
pub trait SVDIsometry<T, const N: usize>: AbstractRotation<T, N>
where
    T: Default + RealField,
{
    /// This function receives the old transform, the centeroids of both point clouds, and the covariance rotation mat
    /// It then performs [`SVD`](nalgebra::SVD) on the covariance matrix, and uses the resulting matrics
    /// and the translation between the two points to construct a new transform.
    /// This transform is multiplied by the old transform, and the result is returned
    fn update_transform(
        old_transform: &Isometry<T, Self, N>,
        mean_a: Point<T, N>,
        mean_b: Point<T, N>,
        rot_mat: &SameSizeMat<T, N>,
    ) -> Isometry<T, Self, N>;
}

impl<T> SVDIsometry<T, 2> for UnitComplex<T>
where
    T: Copy + Default + RealField,
{
    #[inline]
    fn update_transform(
        old_transform: &Isometry<T, Self, 2>,
        mean_a: Point<T, 2>,
        mean_b: Point<T, 2>,
        rot_mat: &SameSizeMat<T, 2>,
    ) -> Isometry<T, Self, 2> {
        let svd = rot_mat.svd(true, true);
        let rotation = verify_rotation_matrix_determinant(svd.u.unwrap(), svd.v_t.unwrap());
        let translation = mean_b.coords - (rotation * mean_a.coords);

        Isometry::from_parts(translation.into(), Self::from_matrix(&rotation)) * old_transform
    }
}

impl<T> SVDIsometry<T, 3> for UnitQuaternion<T>
where
    T: Copy + Default + RealField,
{
    #[inline]
    fn update_transform(
        old_transform: &Isometry<T, Self, 3>,
        mean_a: Point<T, 3>,
        mean_b: Point<T, 3>,
        rot_mat: &SameSizeMat<T, 3>,
    ) -> Isometry<T, Self, 3> {
        let svd = rot_mat.svd(true, true);
        let rotation = verify_rotation_matrix_determinant(svd.u.unwrap(), svd.v_t.unwrap());
        let translation = mean_b.coords - (rotation * mean_a.coords);

        Isometry::from_parts(translation.into(), Self::from_matrix(&rotation)) * old_transform
    }
}

/// A type which is simply an `N` length array of [`RangeInclusive`]s, representing the minimum and maximum coordinates for each dimension.
pub type PolygonExtents<T, const N: usize> = [RangeInclusive<T>; N];
