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

pub use downsample::downsample_point_cloud_voxel;
pub use icp::{
    icp, icp_iteration, ICPConfiguration, ICPConfigurationBuilder, ICPError, ICPResult, ICPSuccess,
};
pub use lex_sort::{lex_sort, lex_sort_in_place, lex_sort_ref};
pub use nearest_neighbour::find_nearest_neighbour_naive;

use nalgebra::{
    AbstractRotation, ClosedAddAssign, ClosedDivAssign, Isometry, Point, RealField, Scalar,
};
use num_traits::{AsPrimitive, Zero};

use crate::{array, Vec};

mod downsample;
mod icp;
mod lex_sort;
mod nearest_neighbour;

#[cfg(feature = "pregenerated")]
#[doc = "Contains pregenerated functions for single precision point cloud mapping-algorithms."]
pub mod single_precision {
    pub use super::icp::single_precision::*;
}

#[cfg(feature = "pregenerated")]
#[doc = "Contains pregenerated functions for double precision point cloud mapping-algorithms."]
pub mod double_precision {
    pub use super::icp::double_precision::*;
}

/// Calculates the mean(centroid) of the point cloud.
///
/// # Arguments
/// * points: a slice of [`Point`], representing the point cloud.
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, representing the number of dimensions in the points.
///
/// # Returns
/// A [`Point`], representing the point cloud centroid.
/// Returns Point::default() if point cloud is empty.
#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Mean Point", skip_all)
)]
pub fn calculate_point_cloud_center<T, const N: usize>(points: &[Point<T, N>]) -> Point<T, N>
where
    T: ClosedAddAssign + ClosedDivAssign + Copy + Scalar + Zero,
    usize: AsPrimitive<T>,
{
    if points.is_empty() {
        return Point::default();
    }

    points
        .iter()
        .fold(Point::<T, N>::from([T::zero(); N]), |acc, it| {
            Point::from(acc.coords + it.coords)
        })
        / points.len().as_()
}

/// Generates a randomized points cloud within a specified spherical range.
///
/// # Arguments
/// * `num_points`: a [`usize`], specifying the amount of points to generate
/// * `range`: a [`crate::ops::RangeInclusive`] specifying the normal distribution of points.
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, representing the number of dimensions to use.
///
/// # Returns
/// A [`Vec`] of [`Point`] representing the point cloud.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Generate Randomized Point Cloud", skip_all, level = "debug")
)]
pub fn generate_point_cloud<T, const N: usize>(
    num_points: usize,
    ranges: [crate::ops::RangeInclusive<T>; N],
) -> Vec<Point<T, N>>
where
    T: PartialOrd + rand::distributions::uniform::SampleUniform + Scalar,
{
    use rand::{Rng, SeedableRng};
    let mut rng = rand::rngs::SmallRng::seed_from_u64(3765665954583626552);

    (0..num_points)
        .map(|_| Point::from(array::from_fn(|idx| rng.gen_range(ranges[idx].clone()))))
        .collect()
} // Just calls a different function a number of times, no specific test needed

/// Transform a point cloud, returning a transformed copy.
/// This function does not mutate the original point cloud.
///
/// # Arguments
/// * `source_points`: a slice of [`Point`], representing the point cloud
/// * `isometry_matrix`: a transform that implements [`AbstractRotation`], to use for the transformation.
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, either `2` or `3`.
/// * `R`: An [`AbstractRotation`] for `T` and `N`.
///
/// # Returns
/// A [`Vec`] of [`Point`] containing the transformed point cloud.
#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Transform Point Cloud", skip_all)
)]
pub fn transform_point_cloud<T, const N: usize, R>(
    source_points: &[Point<T, N>],
    isometry_matrix: Isometry<T, R, N>,
) -> Vec<Point<T, N>>
where
    T: RealField,
    R: AbstractRotation<T, N>,
{
    source_points
        .iter()
        .map(|point| isometry_matrix.transform_point(point))
        .collect()
} // Just calls a different function a number of times, no specific test needed

#[cfg(test)]
mod tests {
    use nalgebra::{Point2, Point3};

    use super::*;

    #[test]
    fn test_empty_point_cloud_center() {
        assert_eq!(calculate_point_cloud_center(&[]), Point2::new(0.0, 0.0));
    }

    #[test]
    fn test_calculate_point_cloud_center() {
        let point_cloud = [
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(2.0, 3.0, 4.0),
            Point3::new(3.0, 4.0, 5.0),
            Point3::new(-2.0, -1.0, 0.0),
            Point3::new(-5.0, -2.0, -3.0),
            Point3::new(1.0, 0.0, 0.0),
        ];

        assert_eq!(
            calculate_point_cloud_center(point_cloud.as_slice()),
            Point3::new(0.0, 1.0, 1.5)
        );
    }
}
