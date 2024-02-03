use crate::{ops::RangeInclusive, utils::verify_rotation_matrix_determinant};
use nalgebra::{
    ArrayStorage, Const, Isometry, Matrix, Point, RealField, UnitComplex, UnitQuaternion, Vector,
};

/// A shorthand way of specifying a symmetrical [`Matrix`](Matrix) of `N` size.
/// Kind of similiar to nalgebra's [`nalgebra::SquareMatrix`] but simpler for our usecase
pub(crate) type SameSizeMat<T, const N: usize> =
    Matrix<T, Const<N>, Const<N>, ArrayStorage<T, N, N>>;

/// This trait acts as an abstraction of the [`Isometry`] matrix.
/// Since that crate does not contain a generic version, that will allow algorithms to be used easily between 2D and 3D.
pub trait IsometryAbstraction<T, const N: usize>
where
    T: Default + RealField,
{
    /// This is the type of the isometry matrix itself, allowing us to specify it for 2D and 3D
    type IsometryType: Copy;

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
        mean_a: Point<T, N>,
        mean_b: Point<T, N>,
        rot_mat: &SameSizeMat<T, N>,
    ) -> Self::IsometryType;
}

impl<T> IsometryAbstraction<T, 2> for Const<2>
where
    T: Copy + Default + RealField,
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
        mean_a: Point<T, 2>,
        mean_b: Point<T, 2>,
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
    T: Copy + Default + RealField,
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
        mean_a: Point<T, 3>,
        mean_b: Point<T, 3>,
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{
        Isometry2, Isometry3, Point2, Point3, Translation2, Translation3, UnitComplex, Vector3,
    };

    #[test]
    fn test_isometry_2d() {
        let point_2d = Point2::new(1.0, 2.0);
        let isometry_2d = Isometry2::from_parts(
            Translation2::new(3.0, 4.0),
            UnitComplex::new(30.0f32.to_radians()),
        );

        assert_eq!(
            <Const<2> as IsometryAbstraction<f32, 2>>::transform_point(&isometry_2d, &point_2d),
            isometry_2d.transform_point(&point_2d)
        );
        assert_eq!(
            <Const<2> as IsometryAbstraction<f32, 2>>::transform_vector(
                &isometry_2d,
                &point_2d.coords
            ),
            isometry_2d.transform_vector(&point_2d.coords)
        );
    }

    #[test]
    fn test_isometry_3d() {
        let point_3d = Point3::new(1.0, 2.0, 3.0);
        let isometry_3d = Isometry3::from_parts(
            Translation3::new(0.1, 0.2, 0.3),
            UnitQuaternion::new(Vector3::new(
                45.0f32.to_radians(),
                45.0f32.to_radians(),
                45.0f32.to_radians(),
            )),
        );

        assert_eq!(
            <Const<3> as IsometryAbstraction<f32, 3>>::transform_point(&isometry_3d, &point_3d),
            isometry_3d.transform_point(&point_3d)
        );
        assert_eq!(
            <Const<3> as IsometryAbstraction<f32, 3>>::transform_vector(
                &isometry_3d,
                &point_3d.coords
            ),
            isometry_3d.transform_vector(&point_3d.coords)
        );
    }
}
