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

use nalgebra::{ComplexField, Point, Scalar};
use num_traits::{AsPrimitive, NumAssign};

use crate::{array, HashMap, Vec};

/// Downsample a points cloud, returning a new point cloud, with all points within each voxel combined into their mean.
///
/// # Arguments
/// * `points`: a slice of [`Point`], representing the point cloud.
/// * `voxel_size`: a floating point number, specifying the size for each voxel, all points inside that voxel will be downsampled to their centroid.
///
/// # Generics
/// * `T`: Either an [`f32`] or [`f64`].
/// * `N`: A const usize, representing the number of dimensions in the points.
///
/// # Returns
/// A [`Vec`] of [`Point`] representing the downsampled point cloud.
///
/// # Warnings
/// * Point cloud order is *never* guaranteed.
/// * When compiling for no_std, a `BTreeMap` from the `alloc` crate is used in place of a [`HashMap`].
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Downsample Point Cloud Using Voxels", skip_all)
)]
pub fn downsample_point_cloud_voxel<T, O, const N: usize>(
    points: &[Point<T, N>],
    voxel_size: O,
) -> Vec<Point<T, N>>
where
    O: AsPrimitive<isize> + ComplexField + Copy,
    T: AsPrimitive<O> + Scalar + NumAssign,
    usize: AsPrimitive<T>,
{
    let mut voxel_map: HashMap<[isize; N], Vec<Point<T, N>>> = HashMap::new();

    // Assign points to voxels
    for point in points {
        let voxel_coords: [isize; N] = array::from_fn(|idx| {
            (AsPrimitive::<O>::as_(point[idx]) / voxel_size)
                .floor()
                .as_()
        });
        voxel_map.entry(voxel_coords).or_default().push(*point);
    }

    // Compute centroid for each voxel and collect them as the downsampled points
    voxel_map
        .into_values()
        .map(|points_in_voxel| {
            let num_points = points_in_voxel.len().as_();
            let sum = points_in_voxel
                .into_iter()
                .fold(Point::default(), |acc, p| acc + p.coords);
            sum / num_points
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_downsample_point_cloud() {
        let point_cloud = [
            Point3::new(-5.9, -5.0, -3.9), // These two are very close now
            Point3::new(-6.0, -5.0, -4.0), // Will end up in the same voxel
            Point3::new(-1.0, -2.0, -3.0),
            Point3::new(0.0, 0.0, 0.0),    // These two are also very close
            Point3::new(0.05, 0.08, 0.01), // Will end up in the same voxel
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(6.0, 5.0, 4.0),
        ];

        // We should be left with 5 voxels
        let res = downsample_point_cloud_voxel(point_cloud.as_slice(), 0.5);
        assert_eq!(res.len(), 5);

        // Moreover, the most negative voxel had two points, (-5.9, -5.0, -4.0) and (-6.0, -5.0, -4.0)
        // Meaning there should be a voxel resulting in the elements' centroid
        assert!(res
            .iter()
            .any(|element| *element == Point3::new(-5.95, -5.0, -3.95)));
    }
}
