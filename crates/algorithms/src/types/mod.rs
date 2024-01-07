use nalgebra::{
    ComplexField, Const, Isometry, OMatrix, Point, RealField, SimdComplexField, SimdRealField,
    UnitComplex, UnitQuaternion,
};

#[cfg(not(feature = "std"))]
use core::{fmt::Debug, ops::RangeInclusive};
#[cfg(feature = "std")]
use std::{fmt::Debug, ops::RangeInclusive};

/// A shorthand way of specifying a symmetrical [`Matrix`](nalgebra::Matrix) of `N` size.
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
    /// In 2D space, <O::Isom> would be a [`UnitComplex<T>`](UnitComplex), in 3D space it would be a [`UnitQuaternion<T>`](UnitQuaternion)
    pub transform: O::Isom,
    /// Mean Squared Error, this is the distances between each point in `points_a` and its corresponding point in `points_b`,
    /// This can be used to determine whether the ICP converged correctly, or simply on its local minimum.
    pub mse: T,
    /// The amount of iterations passed until convergence.
    pub iteration_num: usize,
}

/// This trait acts as an abstraction of the [`Isometry`] matrix.
/// Since that crate does not contain a generic version, that will allow algorithms to be used easily between 2D and 3D.
pub trait IsometryAbstration<T, const N: usize>
where
    T: ComplexField + Copy + Default + RealField,
{
    /// This is the type of the isometry matrix itself, allowing us to specify it for 2D and 3D
    type Isom;

    /// This creates an identity [`Isometry`] Matrix.
    fn identity() -> Self::Isom;

    /// This function acts as a wrapper for the isometry's transform_point function
    fn transform_point(isom: &Self::Isom, point: &Point<T, N>) -> Point<T, N>;

    /// This function receives the old transform, the centeroids of both point clouds, and the covariance rotation mat
    /// It then performs [`SVD`](nalgebra::SVD) on the covariance matrix, and uses the resulting matrics
    /// and the translation between the two points to construct a new transform.
    /// This transform is multiplied by the old transform, and the result is returned
    fn update_transform(
        old_transform: &Self::Isom,
        mean_b: Point<T, N>,
        mean_a: Point<T, N>,
        rot_mat: &SameSizeMat<T, N>,
    ) -> Self::Isom;
}

impl<T> IsometryAbstration<T, 2> for Const<2>
where
    T: ComplexField + Copy + Default + RealField,
{
    type Isom = Isometry<T, UnitComplex<T>, 2>;

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
        mean_b: Point<T, 2>,
        mean_a: Point<T, 2>,
        rot_mat: &SameSizeMat<T, 2>,
    ) -> Self::Isom {
        let svd = rot_mat.svd(true, true);
        let rotation = svd.u.unwrap() * svd.v_t.unwrap();
        let translation = mean_b.coords - (rotation * mean_a.coords);
        Self::Isom::from_parts(translation.into(), UnitComplex::from_matrix(&rotation))
            * old_transform
    }
}

impl<T> IsometryAbstration<T, 3> for Const<3>
where
    T: ComplexField + Copy + Default + RealField,
{
    type Isom = Isometry<T, UnitQuaternion<T>, 3>;

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
        mean_b: Point<T, 3>,
        mean_a: Point<T, 3>,
        rot_mat: &SameSizeMat<T, 3>,
    ) -> Self::Isom {
        let svd = rot_mat.svd(true, true);
        let rotation = svd.u.unwrap() * svd.v_t.unwrap();
        let translation = mean_b.coords - (rotation * mean_a.coords);
        Self::Isom::from_parts(translation.into(), UnitQuaternion::from_matrix(&rotation))
            * old_transform
    }
}

/// A type which is simply an `N` length array of [`RangeInclusive`]s, representing the minimum and maximum coordinates for each dimension.
pub type PolygonExtents<T, const N: usize> = [RangeInclusive<T>; N];
