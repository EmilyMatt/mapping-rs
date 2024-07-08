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

use super::{grid_map::GridMap, HectorMapper};
use crate::{array, PhantomData};
use mapping_algorithms::types::{AbstractIsometry, IsometryAbstractor};
use nalgebra::{AbstractRotation, RealField};
use num_traits::AsPrimitive;

pub mod odometry {
    pub struct WantsOdometryConfig;
    pub struct HasOdometryConfig;
}

pub mod dimensions {
    pub struct WantsDimensions;
    pub struct HasDimensions;
}

pub type EmptyHectorMapperBuilder<T, const N: usize> =
    HectorMapperBuilder<T, N, odometry::WantsOdometryConfig, dimensions::WantsDimensions>;

pub struct HectorMapperBuilder<
    T: RealField,
    const N: usize,
    MaybeCalculatesOdometry,
    MaybeHasDimensions,
> where
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
{
    pub(super) _internal: HectorMapper<T, N>,
    pub(super) dimensions: [usize; N],
    occupied_factor: T,
    free_factor: T,
    max_confidence: T,

    // Types Phantom Data
    pub(super) _calculates_odometry: PhantomData<MaybeCalculatesOdometry>,
    pub(super) _has_dimensions: PhantomData<MaybeHasDimensions>,
}

impl<T: Copy + Default + RealField, const N: usize>
    HectorMapperBuilder<T, N, odometry::WantsOdometryConfig, dimensions::WantsDimensions>
where
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
{
    pub fn default() -> Self {
        Self {
            _internal: HectorMapper {
                with_odometry: false,
                grid_map: GridMap::create(&[0; N], T::zero(), T::zero(), T::zero()),
                resolution: T::one(),
                current_pose: Default::default(),
                last_point_cloud: Vec::new(),
                frame_index: 1,
            },
            dimensions: [0; N],
            occupied_factor: T::zero(),
            free_factor: T::zero(),
            max_confidence: T::zero(),
            _calculates_odometry: Default::default(),
            _has_dimensions: Default::default(),
        }
    }
}

impl<T: RealField, const N: usize, MaybeHasDimensions>
    HectorMapperBuilder<T, N, odometry::WantsOdometryConfig, MaybeHasDimensions>
where
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
{
    pub fn with_odometry_calculation(
        self,
        calculate_odometry: bool,
    ) -> HectorMapperBuilder<T, N, odometry::HasOdometryConfig, MaybeHasDimensions> {
        HectorMapperBuilder {
            _internal: HectorMapper {
                with_odometry: calculate_odometry,
                ..self._internal
            },
            dimensions: self.dimensions,
            occupied_factor: self.occupied_factor,
            free_factor: self.free_factor,
            max_confidence: self.max_confidence,
            _calculates_odometry: Default::default(),
            _has_dimensions: Default::default(),
        }
    }
}

impl<T: RealField, const N: usize, MaybeCalculatesOdometry>
    HectorMapperBuilder<T, N, MaybeCalculatesOdometry, dimensions::WantsDimensions>
where
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
{
    pub fn with_dimensions(
        self,
        dimensions: [usize; N],
    ) -> HectorMapperBuilder<T, N, MaybeCalculatesOdometry, dimensions::HasDimensions> {
        HectorMapperBuilder {
            _internal: HectorMapper { ..self._internal },
            dimensions,
            occupied_factor: self.occupied_factor,
            free_factor: self.free_factor,
            max_confidence: self.max_confidence,
            _calculates_odometry: Default::default(),
            _has_dimensions: Default::default(),
        }
    }
}

impl<T: RealField, const N: usize, MaybeCalculatesOdometry, MaybeHasDimensions>
    HectorMapperBuilder<T, N, MaybeCalculatesOdometry, MaybeHasDimensions>
where
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
{
    pub fn with_resolution(self, resolution: T) -> Self {
        Self {
            _internal: HectorMapper {
                resolution,
                ..self._internal
            },
            ..self
        }
    }
    pub fn with_occupied_confidence_factor(self, occupied_factor: T) -> Self {
        Self {
            occupied_factor,
            ..self
        }
    }

    pub fn with_free_confidence_factor(self, free_factor: T) -> Self {
        Self {
            free_factor,
            ..self
        }
    }

    pub fn with_maximum_confidence(self, max_confidence: T) -> Self {
        Self {
            max_confidence,
            ..self
        }
    }
}

impl<T: Copy + Default + RealField, const N: usize>
    HectorMapperBuilder<T, N, odometry::HasOdometryConfig, dimensions::HasDimensions>
where
    IsometryAbstractor<T, N>: AbstractIsometry<T, N>,
    usize: AsPrimitive<T>,
    f32: AsPrimitive<T>,
{
    pub fn build(self) -> HectorMapper<T, N> {
        HectorMapper {
            grid_map: GridMap::create(
                &self.dimensions,
                self.occupied_factor,
                self.free_factor,
                self.max_confidence,
            ),
            current_pose: nalgebra::Similarity::from_parts(
                nalgebra::Translation::from(nalgebra::OVector::from_array_storage(
                    nalgebra::ArrayStorage(
                        [array::from_fn(|idx| self.dimensions[idx].as_() / 2.0.as_()); 1],
                    ),
                )),
                <IsometryAbstractor<T, N> as AbstractIsometry<T, N>>::RotType::identity(),
                self._internal.resolution,
            ),
            ..self._internal
        }
    }
}
