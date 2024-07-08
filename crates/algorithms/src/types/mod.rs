// SPDX-License-Identifier: MIT
/*
 * Copyright (c) [2023 - Present] Emily Matheys <emilymatt96@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

use crate::{marker::PhantomData, ops::RangeInclusive, utils::verify_rotation_matrix_determinant};
use nalgebra::{
    AbstractRotation, ArrayStorage, Const, Isometry, Matrix, Point, RealField, UnitComplex,
    UnitQuaternion,
};

/// A shorthand way of specifying a symmetrical [`Matrix`](Matrix) of `N` size.
/// Kind of similiar to nalgebra's [`SquareMatrix`](nalgebra::SquareMatrix) but simpler for our usecase
pub(crate) type SameSizeMat<T, const N: usize> =
    Matrix<T, Const<N>, Const<N>, ArrayStorage<T, N, N>>;

/// This struct allows us to write functions in a generic way for both 2D and 3D dimensions, by implementing
pub struct IsometryAbstractor<T: RealField, const N: usize> {
    _precision: PhantomData<T>,
}

/// This trait acts as an abstraction for the process of creating an [`Isometry`] matrix from a NxN SVD,
/// without having to constrain every function by a million traits.
pub trait AbstractIsometry<T: RealField, const N: usize> {
    /// This type is a placeholder for either [`UnitComplex`] or [`UnitQuaternion`] depending on number of dimensions.
    type RotType: AbstractRotation<T, N> + Copy;

    /// This function receives the old transform, the centeroids of both point clouds, and the covariance rotation mat
    /// It then performs [`SVD`](nalgebra::SVD) on the covariance matrix, and uses the resulting matrics
    /// and the translation between the two points to construct a new transform.
    /// This transform is multiplied by the old transform, and the result is returned
    fn update_transform(
        old_transform: &Isometry<T, Self::RotType, N>,
        mean_a: Point<T, N>,
        mean_b: Point<T, N>,
        rot_mat: &SameSizeMat<T, N>,
    ) -> Isometry<T, Self::RotType, N>;
}

impl<T> AbstractIsometry<T, 2> for IsometryAbstractor<T, 2>
where
    T: Copy + RealField,
{
    type RotType = UnitComplex<T>;

    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Update 2D Transform Using SVD", skip_all, level = "debug")
    )]
    fn update_transform(
        old_transform: &Isometry<T, Self::RotType, 2>,
        mean_a: Point<T, 2>,
        mean_b: Point<T, 2>,
        rot_mat: &SameSizeMat<T, 2>,
    ) -> Isometry<T, Self::RotType, 2> {
        let svd = rot_mat.svd(true, true);
        let rotation = verify_rotation_matrix_determinant(svd.u.unwrap(), svd.v_t.unwrap());
        let translation = mean_b.coords - (rotation * mean_a.coords);

        Isometry::from_parts(translation.into(), Self::RotType::from_matrix(&rotation))
            * old_transform
    }
}

impl<T> AbstractIsometry<T, 3> for IsometryAbstractor<T, 3>
where
    T: Copy + RealField,
{
    type RotType = UnitQuaternion<T>;

    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Update 3D Transform Using SVD", skip_all, level = "debug")
    )]
    fn update_transform(
        old_transform: &Isometry<T, Self::RotType, 3>,
        mean_a: Point<T, 3>,
        mean_b: Point<T, 3>,
        rot_mat: &SameSizeMat<T, 3>,
    ) -> Isometry<T, Self::RotType, 3> {
        let svd = rot_mat.svd(true, true);
        let rotation = verify_rotation_matrix_determinant(svd.u.unwrap(), svd.v_t.unwrap());
        let translation = mean_b.coords - (rotation * mean_a.coords);

        Isometry::from_parts(translation.into(), Self::RotType::from_matrix(&rotation))
            * old_transform
    }
}

/// A type which is simply an `N` length array of [`RangeInclusive`]s, representing the minimum and maximum coordinates for each dimension.
pub type PolygonExtents<T, const N: usize> = [RangeInclusive<T>; N];
