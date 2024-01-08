use crate::{ops::RangeInclusive, utils::verify_rotation_matrix_determinant};
use nalgebra::{
    ArrayStorage, Const, Isometry, Matrix, Point, RealField, SimdRealField, UnitComplex,
    UnitQuaternion, Vector,
};

/// A shorthand way of specifying a symmetrical [`Matrix`](Matrix) of `N` size.
/// Kind of similiar to nalgebra's [`nalgebra::SquareMatrix`] but simpler for our usecase
pub(crate) type SameSizeMat<T, const N: usize> =
    Matrix<T, Const<N>, Const<N>, ArrayStorage<T, N, N>>;

/// This trait acts as an abstraction of the [`Isometry`] matrix.
/// Since that crate does not contain a generic version, that will allow algorithms to be used easily between 2D and 3D.
pub trait IsometryAbstraction<T, const N: usize>
where
    T: Copy + Default + RealField + SimdRealField,
{
    /// This is the type of the isometry matrix itself, allowing us to specify it for 2D and 3D
    type IsometryType: Clone;

    /// This creates an identity [`Isometry`] Matrix.
    fn identity() -> Self::IsometryType;

    /// This function acts as a wrapper for the isometry's transform_point function
    fn transform_point(isom: &Self::IsometryType, point: &Point<T, N>) -> Point<T, N>;

    /// This function acts as a wrapper for the isometry's transform_vector function
    fn transform_vector(
        isom: &Self::IsometryType,
        vector: &Vector<T, Const<N>, ArrayStorage<T, N, 1>>,
    ) -> Vector<T, Const<N>, ArrayStorage<T, N, 1>>;

    /// This function receives the old transform, the centeroids of both point clouds, and the covariance rotation mat
    /// It then performs [`SVD`](nalgebra::SVD) on the covariance matrix, and uses the resulting matrics
    /// and the translation between the two points to construct a new transform.
    /// This transform is multiplied by the old transform, and the result is returned
    fn update_transform(
        old_transform: &Self::IsometryType,
        mean_b: Point<T, N>,
        mean_a: Point<T, N>,
        rot_mat: &SameSizeMat<T, N>,
    ) -> Self::IsometryType;
}

impl<T> IsometryAbstraction<T, 2> for Const<2>
where
    T: Copy + Default + RealField + SimdRealField,
{
    type IsometryType = Isometry<T, UnitComplex<T>, 2>;

    #[inline]
    fn identity() -> Self::IsometryType {
        Isometry::identity()
    }

    #[inline]
    fn transform_point(isom: &Self::IsometryType, point: &Point<T, 2>) -> Point<T, 2> {
        isom.transform_point(point)
    }

    #[inline]
    fn transform_vector(
        isom: &Self::IsometryType,
        vector: &Vector<T, Self, ArrayStorage<T, 2, 1>>,
    ) -> Vector<T, Self, ArrayStorage<T, 2, 1>> {
        isom.transform_vector(vector)
    }

    #[inline]
    fn update_transform(
        old_transform: &Self::IsometryType,
        mean_b: Point<T, 2>,
        mean_a: Point<T, 2>,
        rot_mat: &SameSizeMat<T, 2>,
    ) -> Self::IsometryType {
        let svd = rot_mat.svd(true, true);
        let rotation = verify_rotation_matrix_determinant(svd.u.unwrap(), svd.v_t.unwrap());
        let translation = mean_b.coords - (rotation * mean_a.coords);

        Isometry::from_parts(translation.into(), UnitComplex::from_matrix(&rotation))
            * old_transform
    }
}

impl<T> IsometryAbstraction<T, 3> for Const<3>
where
    T: Copy + Default + RealField + SimdRealField,
{
    type IsometryType = Isometry<T, UnitQuaternion<T>, 3>;

    #[inline]
    fn identity() -> Self::IsometryType {
        Isometry::identity()
    }

    #[inline]
    fn transform_point(isom: &Self::IsometryType, point: &Point<T, 3>) -> Point<T, 3> {
        isom.transform_point(point)
    }

    #[inline]
    fn transform_vector(
        isom: &Self::IsometryType,
        vector: &Vector<T, Self, ArrayStorage<T, 3, 1>>,
    ) -> Vector<T, Self, ArrayStorage<T, 3, 1>> {
        isom.transform_vector(vector)
    }

    #[inline]
    fn update_transform(
        old_transform: &Self::IsometryType,
        mean_b: Point<T, 3>,
        mean_a: Point<T, 3>,
        rot_mat: &SameSizeMat<T, 3>,
    ) -> Self::IsometryType {
        let svd = rot_mat.svd(true, true);
        let rotation = verify_rotation_matrix_determinant(svd.u.unwrap(), svd.v_t.unwrap());
        let translation = mean_b.coords - (rotation * mean_a.coords);

        Isometry::from_parts(translation.into(), UnitQuaternion::from_matrix(&rotation))
            * old_transform
    }
}

/// A type which is simply an `N` length array of [`RangeInclusive`]s, representing the minimum and maximum coordinates for each dimension.
pub type PolygonExtents<T, const N: usize> = [RangeInclusive<T>; N];
