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

use crate::Sum;
use builder::EmptyHectorMapperBuilder;
use grid_map::GridMap;
use mapping_algorithms::{
    icp::{icp, types::ICPConfiguration},
    types::{AbstractIsometry, IsometryAbstractor},
};
use nalgebra::RealField;
use num_traits::{AsPrimitive, Bounded};

mod builder;
mod grid_map;

///
pub struct HectorMapper<T, const N: usize>
where
    T: RealField,
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
{
    grid_map: GridMap<T, N>,
    resolution: T,
    with_odometry: bool,

    current_pose:
        nalgebra::Similarity<T, <IsometryAbstractor<T, N> as AbstractIsometry<T, N>>::RotType, N>,
    last_point_cloud: Vec<nalgebra::Point<T, N>>,
    frame_index: u8,
}

impl<T, const N: usize> HectorMapper<T, N>
where
    T: Bounded + Copy + Default + RealField + Sum + AsPrimitive<usize>,
    f32: AsPrimitive<T>,
    usize: AsPrimitive<T>,
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
{
    ///
    pub fn builder() -> EmptyHectorMapperBuilder<T, N> {
        EmptyHectorMapperBuilder::default()
    }

    ///
    pub fn push_point_cloud(&mut self, point_cloud: &[nalgebra::Point<T, N>], is_new_frame: bool) {
        if self.with_odometry && is_new_frame {
            if !self.last_point_cloud.is_empty() {
                if let Ok(res) = icp::<T, N>(
                    &self.last_point_cloud,
                    point_cloud,
                    ICPConfiguration::builder()
                        .with_kd_tree(true)
                        .with_max_iterations(20)
                        .with_mse_interval_threshold(0.01.as_())
                        .build(),
                )
                    .map_err(|err| println!("{err}"))
                {
                    self.current_pose
                        .append_translation_mut(&res.transform.translation);
                    self.current_pose
                        .append_rotation_wrt_center_mut(&res.transform.rotation);
                }
            }
            self.last_point_cloud = point_cloud.to_vec();
        }

        if is_new_frame {
            if self.frame_index == 255 {
                self.frame_index = 1;
            } else {
                self.frame_index += 1;
            }
        }
        // for point in point_cloud {
        //     let point_scaled = self.current_pose.transform_point(point);
        //     let bresenham_points = plot_bresenham_line(
        //         self.current_pose.isometry.translation.vector.into(),
        //         point_scaled,
        //     );
        //     for bresenham_point in bresenham_points.iter().take(bresenham_points.len() - 1) {
        //         self.grid_map.update_free(bresenham_point, self.frame_index)
        //     }
        //     let last_point = bresenham_points[bresenham_points.len() - 1];
        //     self.grid_map.update_taken(&last_point, self.frame_index);
        // }
    }

    ///
    pub fn get_current_pose(
        &self,
    ) -> nalgebra::Isometry<T, <IsometryAbstractor<T, N> as AbstractIsometry<T, N>>::RotType, N>
    {
        self.current_pose.isometry
    }
}

#[cfg(test)]
mod tests {
    use crate::hector_mapper::HectorMapper;
    use mapping_algorithms::utils::point_cloud::{generate_point_cloud, transform_point_cloud};

    #[test]
    fn test_something() {
        let mut hector_mapper = HectorMapper::builder()
            .with_resolution(0.1)
            .with_odometry_calculation(true)
            .with_dimensions([1024; 3])
            .build();

        let point_cloud = generate_point_cloud(300, [-9.0..=9.0, 0.0..=1.0, -9.0..=9.0]);
        hector_mapper.push_point_cloud(&point_cloud, true);

        let mut isom: nalgebra::Isometry<f32, nalgebra::UnitQuaternion<f32>, 3> =
            nalgebra::Isometry::identity();
        isom.append_translation_mut(&nalgebra::Translation3::new(0.0, 0.0, 0.5));
        isom.append_rotation_wrt_center_mut(&nalgebra::UnitQuaternion::from_euler_angles(
            0.0,
            0.0,
            1.0f32.to_radians(),
        ));
        let point_cloud_transformed = transform_point_cloud(&point_cloud, isom);
        hector_mapper.push_point_cloud(&point_cloud_transformed, true);
    }
}
