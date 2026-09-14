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

pub(crate) use scan::ScanUpdater;
pub(crate) use types::{CellIndex, GridMapConfig, MapSample, RayTermination};
pub use types::{GridMapError, GridMapResult};

use nalgebra::{ComplexField, Point, RealField, SVector};
use num_traits::AsPrimitive;

use crate::{Box, Vec, array, fmt, ops::RangeInclusive};

mod scan;
mod types;

/// A dense, fixed-extent occupancy grid storing per-cell log-odds.
///
/// Index-space only: positions are in fractional cell coordinates, where `1.0` is one cell.
/// Converting metres to cells is the caller's responsibility. Axis `0` varies fastest.
///
/// # Generics
/// * `T`: Either an [`prim@f32`] or [`prim@f64`].
/// * `N`: a usize, representing the number of dimensions.
#[derive(Clone, PartialEq)]
pub(crate) struct GridMap<T, const N: usize> {
    /// Flat log-odds array - the interpolation hot path reads only this.
    odds: Box<[T]>,
    /// Encoded as `frame << 1 | marked_occupied`.
    last_frame_to_update: Box<[u32]>,
    dimensions: [usize; N],
    strides: [usize; N],
    interp_limits: [T; N],
    occupied_delta: T,
    free_delta: T,
    min_log_odds: T,
    max_log_odds: T,
    frame: u32,
}

impl<T: fmt::Debug, const N: usize> fmt::Debug for GridMap<T, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GridMap")
            .field("dimensions", &self.dimensions)
            .field("cell_count", &self.odds.len())
            .field("occupied_delta", &self.occupied_delta)
            .field("free_delta", &self.free_delta)
            .field("min_log_odds", &self.min_log_odds)
            .field("max_log_odds", &self.max_log_odds)
            .field("frame", &self.frame)
            .finish_non_exhaustive()
    }
}

impl<T, const N: usize> GridMap<T, N>
where
    T: AsPrimitive<isize> + AsPrimitive<usize> + Copy + RealField,
    usize: AsPrimitive<T>,
{
    /// Allocates a new, fully unknown occupancy grid.
    ///
    /// # Arguments
    /// * `dimensions`: the extent of each axis in cells; every entry must be at least `2`.
    /// * `config`: the inverse sensor model and saturation confidence.
    ///
    /// # Returns
    /// A [`GridMap`] with every cell at a probability of one half.
    ///
    /// # Errors
    /// * [`GridMapError::DimensionTooSmall`] or [`GridMapError::CapacityOverflow`]: if `dimensions`
    ///   is unusable.
    /// * [`GridMapError::InvalidOccupiedProbability`], [`GridMapError::InvalidFreeProbability`] or
    ///   [`GridMapError::InvalidMaxConfidence`]: if `config` is inconsistent.
    /// * [`GridMapError::AllocationFailed`]: if the allocator cannot satisfy the request.
    ///
    /// # Warnings
    /// Storage is `product(dimensions)` cells twice over, so a three-dimensional map grows cubically.
    #[cfg_attr(feature = "tracing", tracing::instrument("Create Grid Map", skip_all))]
    pub(crate) fn new(dimensions: [usize; N], config: &GridMapConfig<T>) -> GridMapResult<Self> {
        for (axis, &extent) in dimensions.iter().enumerate() {
            if extent < 2 {
                return Err(GridMapError::DimensionTooSmall { axis, extent });
            }
        }

        let cells = dimensions
            .iter()
            .try_fold(1usize, |acc, &extent| acc.checked_mul(extent))
            .ok_or(GridMapError::CapacityOverflow)?;

        let (occupied_delta, free_delta, max_log_odds) = config.resolve()?;

        let mut odds = Vec::new();
        odds.try_reserve_exact(cells)
            .map_err(|_| GridMapError::AllocationFailed { cells })?;
        odds.resize(cells, T::zero());

        let mut stamps = Vec::new();
        stamps
            .try_reserve_exact(cells)
            .map_err(|_| GridMapError::AllocationFailed { cells })?;
        stamps.resize(cells, 0u32);

        Ok(Self {
            odds: odds.into_boxed_slice(),
            last_frame_to_update: stamps.into_boxed_slice(),
            dimensions,
            strides: array::from_fn(|idx| dimensions.iter().take(idx).product()),
            interp_limits: array::from_fn(|idx| (dimensions[idx] - 1).as_()),
            occupied_delta,
            free_delta,
            min_log_odds: -max_log_odds,
            max_log_odds,
            // last_frame_to_update is zeroized, so we set this to 1 to immediately start updating.
            frame: 1,
        })
    }

    /// The log-odds range at which cells saturate, symmetric about zero.
    ///
    /// # Returns
    /// A [`RangeInclusive`] of `T`, the interval every cell value lies within.
    fn log_odds_bounds(&self) -> RangeInclusive<T> {
        self.min_log_odds..=self.max_log_odds
    }

    /// Iterates the log-odds of every cell, with axis `0` varying fastest.
    ///
    /// # Returns
    /// An [`ExactSizeIterator`] of `T` of length [`cell_count`](Self::cell_count).
    fn iter_log_odds(&self) -> impl ExactSizeIterator<Item = T> + '_ {
        self.odds.iter().copied()
    }
}

// Addressing. Every read and write in the crate funnels through these functions.
impl<T, const N: usize> GridMap<T, N>
where
    T: AsPrimitive<isize> + AsPrimitive<usize> + Copy + RealField,
    usize: AsPrimitive<T>,
{
    /// Converts a cell index into a flat index into the log-odds array.
    ///
    /// Each axis is range-checked individually *before* it contributes to the sum; checking only the
    /// final sum is unsound, since an overshoot on one axis lands on a valid cell in the next row.
    ///
    /// # Arguments
    /// * `index`: the cell to resolve, which may be negative or out of range.
    ///
    /// # Returns
    /// An [`Option`] of [`prim@usize`], or [`None`] if any axis is out of range.
    #[inline]
    fn linearize(&self, index: &CellIndex<N>) -> Option<usize> {
        let mut linear = 0usize;
        for axis in 0..N {
            // Reinterpreting as unsigned folds the negative test into the upper-bound test: any
            // negative value becomes enormous and fails the very same comparison.
            let coordinate = index[axis].cast_unsigned();
            if coordinate >= self.dimensions[axis] {
                return None;
            }
            linear += coordinate * self.strides[axis];
        }
        Some(linear)
    }

    /// As [`linearize`](Self::linearize), but naming the axis that failed.
    ///
    /// # Arguments
    /// * `index`: the cell to resolve, which may be negative or out of range.
    ///
    /// # Returns
    /// A [`prim@usize`], the flat index.
    ///
    /// # Errors
    /// * [`GridMapError::OutOfBounds`]: naming the first failing axis in ascending order.
    #[inline]
    fn linearize_checked(&self, index: &CellIndex<N>) -> GridMapResult<usize> {
        let mut linear = 0usize;
        for axis in 0..N {
            let coordinate = index[axis].cast_unsigned();
            if coordinate >= self.dimensions[axis] {
                return Err(GridMapError::OutOfBounds {
                    axis,
                    index: index[axis],
                    extent: self.dimensions[axis],
                });
            }
            linear += coordinate * self.strides[axis];
        }
        Ok(linear)
    }

    /// Locates the low corner of the `2^N` interpolation stencil containing a point.
    ///
    /// Verifying the stencil once here is what lets the sampler address all `2^N` corners directly.
    ///
    /// # Arguments
    /// * `point`: a position in fractional cell coordinates.
    ///
    /// # Returns
    /// An [`Option`] of the stencil's flat base index and the fractional offset within it, or
    /// [`None`] if `point` is non-finite or outside the map.
    #[inline]
    fn stencil_base(&self, point: &Point<T, N>) -> Option<(usize, [T; N])> {
        let mut base = 0usize;
        let mut frac = [T::zero(); N];
        for axis in 0..N {
            let coordinate = point[axis];
            // Every comparison against NaN is false, so a NaN coordinate fails this test and no
            // separate check is needed.
            if !(T::zero()..=self.interp_limits[axis]).contains(&coordinate) {
                return None;
            }
            // Non-negativity is proven above, so this truncating cast *is* a floor. Going through
            // `ComplexField::floor` instead would emit a libm call under `--no-default-features`.
            let mut cell: usize = coordinate.as_();
            let mut offset = coordinate - cell.as_();

            // The very top of the domain has no cell above it to interpolate against. Folding it
            // into the last stencil at full weight keeps the domain a closed interval and yields
            // the same value, and it is what stops the corner gather from running off the end of
            // the row into the *next* one, which would silently produce a neighbour from the wrong
            // line and a meaningless gradient.
            if cell + 1 >= self.dimensions[axis] {
                cell = self.dimensions[axis] - 2;
                offset = T::one();
            }

            frac[axis] = offset;
            base += cell * self.strides[axis];
        }
        Some((base, frac))
    }

    /// Resolves a fractional coordinate to the cell containing it.
    ///
    /// Explicitly fallible, because a float-to-integer cast saturates and maps NaN to zero.
    ///
    /// # Arguments
    /// * `point`: a position in fractional cell coordinates.
    ///
    /// # Returns
    /// The [`CellIndex`] whose cell contains `point`.
    ///
    /// # Errors
    /// * [`GridMapError::NonFiniteCoordinate`]: if any coordinate is NaN or infinite.
    /// * [`GridMapError::OutOfBounds`]: naming the first axis outside the map.
    pub(crate) fn cell_containing(&self, point: &Point<T, N>) -> GridMapResult<CellIndex<N>> {
        let mut index = CellIndex::<N>::origin();
        for axis in 0..N {
            let coordinate = point[axis];
            if !ComplexField::is_finite(&coordinate) {
                return Err(GridMapError::NonFiniteCoordinate { axis });
            }

            let extent = self.dimensions[axis];
            // A coordinate of exactly `extent` is already past the last cell, so the valid
            // fractional range is half-open.
            if !(T::zero()..extent.as_()).contains(&coordinate) {
                return Err(GridMapError::OutOfBounds {
                    axis,
                    // Truncating toward zero, saturating at the ends of the range; for the
                    // in-range-adjacent coordinates a caller is likely to be debugging this is
                    // the cell they meant.
                    index: coordinate.as_(),
                    extent,
                });
            }

            // Non-negativity is proven above, so truncation is a floor.
            index[axis] = coordinate.as_();
        }
        Ok(index)
    }
}

// Reads, single-cell writes, and the scan entry point.
impl<T, const N: usize> GridMap<T, N>
where
    T: AsPrimitive<isize> + AsPrimitive<usize> + Copy + RealField,
    usize: AsPrimitive<T>,
{
    /// The occupancy probability corresponding to a log-odds value.
    ///
    /// # Arguments
    /// * `log_odds`: the value to convert.
    ///
    /// # Returns
    /// A `T` in the range `0.0..=1.0`.
    #[inline]
    fn logistic(log_odds: T) -> T {
        T::one() / (T::one() + ComplexField::exp(-log_odds))
    }

    /// Maps a log-odds sample through the logistic function, carrying its gradient by the chain rule.
    ///
    /// # Arguments
    /// * `sample`: a sample whose value is log-odds.
    ///
    /// # Returns
    /// A sample whose value is a probability, for the cost of one exponential.
    #[inline]
    fn sample_to_probability(sample: MapSample<T, N>) -> MapSample<T, N> {
        let probability = Self::logistic(sample.value);
        MapSample {
            value: probability,
            gradient: sample.gradient * (probability * (T::one() - probability)),
        }
    }

    /// Multilinearly interpolates the log-odds field and its analytic gradient.
    ///
    /// The value is a weighted sum over the `2^N` corners of the stencil containing `point`, and
    /// each gradient component is the same sum over the differences across that axis, with the
    /// axis's own weight left out. Performs no transcendental operations.
    ///
    /// # Arguments
    /// * `point`: a position in fractional cell coordinates.
    ///
    /// # Returns
    /// An [`Option`] of [`MapSample`], or [`None`] if `point` is non-finite or outside the map.
    ///
    /// # Warnings
    /// This interpolates log-odds and sigmoids afterwards, whereas Hector SLAM interpolates
    /// already-sigmoided probabilities. The fields agree only at cell corners, so gains and
    /// thresholds taken from such an implementation do not carry over unscaled.
    pub(crate) fn sample_log_odds(&self, point: &Point<T, N>) -> Option<MapSample<T, N>> {
        let (base, frac) = self.stencil_base(point)?;

        // A corner is a bit pattern: bit `axis` set means the high side of that axis. The stencil
        // has been proven in range, so every index built this way addresses a real cell.
        let corner_odds = |corner: usize| -> T {
            self.odds[(0..N).fold(base, |acc, axis| {
                acc + if corner >> axis & 1 == 1 {
                    self.strides[axis]
                } else {
                    0
                }
            })]
        };

        // The product of the per-axis weights, optionally leaving one axis out; omitting axis `k`
        // is what turns the value's weighted sum into its derivative along `k`.
        let weight = |corner: usize, skip: Option<usize>| -> T {
            (0..N)
                .filter(|axis| Some(*axis) != skip)
                .fold(T::one(), |acc, axis| {
                    acc * if corner >> axis & 1 == 1 {
                        frac[axis]
                    } else {
                        T::one() - frac[axis]
                    }
                })
        };

        let corners = 1usize << N;
        let value = (0..corners).fold(T::zero(), |acc, corner| {
            acc + weight(corner, None) * corner_odds(corner)
        });

        let mut gradient = SVector::<T, N>::zeros();
        for axis in 0..N {
            // Pair every low corner with its neighbour across `axis`; the difference between the
            // two is that edge's derivative.
            gradient[axis] = (0..corners).filter(|corner| corner >> axis & 1 == 0).fold(
                T::zero(),
                |acc, corner| {
                    acc + weight(corner, Some(axis))
                        * (corner_odds(corner | (1 << axis)) - corner_odds(corner))
                },
            );
        }

        Some(MapSample { value, gradient })
    }

    /// Multilinearly interpolates the log-odds field, then maps it through the logistic function.
    ///
    /// Costs one exponential per query rather than one per corner. See the warning on
    /// [`sample_log_odds`](Self::sample_log_odds).
    ///
    /// # Arguments
    /// * `point`: a position in fractional cell coordinates.
    ///
    /// # Returns
    /// An [`Option`] of [`MapSample`] whose value is a probability in the range `0.0..=1.0`, or
    /// [`None`] if `point` is non-finite or outside the map.
    pub(crate) fn sample_probability(&self, point: &Point<T, N>) -> Option<MapSample<T, N>> {
        self.sample_log_odds(point).map(Self::sample_to_probability)
    }

    /// Reads the log-odds of a single cell.
    ///
    /// # Arguments
    /// * `index`: the cell to read, which may be negative or out of range.
    ///
    /// # Returns
    /// An [`Option`] of `T`, or [`None`] if `index` lies outside the map.
    #[inline]
    pub(crate) fn log_odds_at(&self, index: &CellIndex<N>) -> Option<T> {
        self.linearize(index).map(|linear| self.odds[linear])
    }

    /// Reads the occupancy probability of a single cell.
    ///
    /// # Arguments
    /// * `index`: the cell to read, which may be negative or out of range.
    ///
    /// # Returns
    /// An [`Option`] of `T` in the range `0.0..=1.0`, or [`None`] if `index` lies outside the map.
    /// An unwritten cell reads as exactly one half.
    #[inline]
    pub(crate) fn probability_at(&self, index: &CellIndex<N>) -> Option<T> {
        self.log_odds_at(index).map(Self::logistic)
    }

    /// Overwrites a cell's log-odds, bypassing the sensor model and per-scan deduplication.
    ///
    /// # Arguments
    /// * `index`: the cell to overwrite.
    /// * `log_odds`: the value to store, clamped into [`log_odds_bounds`](Self::log_odds_bounds).
    ///
    /// # Errors
    /// * [`GridMapError::OutOfBounds`]: naming the first offending axis.
    pub(crate) fn set_log_odds(&mut self, index: &CellIndex<N>, log_odds: T) -> GridMapResult<()> {
        let linear = self.linearize_checked(index)?;
        self.odds[linear] = log_odds.clamp(self.min_log_odds, self.max_log_odds);
        Ok(())
    }

    /// Overwrites a cell's occupancy probability. See [`set_log_odds`](Self::set_log_odds).
    ///
    /// # Arguments
    /// * `index`: the cell to overwrite.
    /// * `probability`: strictly between `0.0` and `1.0`.
    ///
    /// # Errors
    /// * [`GridMapError::OutOfBounds`]: naming the first offending axis.
    /// * [`GridMapError::InvalidOccupiedProbability`]: if `probability` is not strictly between zero
    ///   and one, since either endpoint is infinite in log-odds.
    pub(crate) fn set_probability(
        &mut self,
        index: &CellIndex<N>,
        probability: T,
    ) -> GridMapResult<()> {
        if !(probability > T::zero() && probability < T::one()) {
            return Err(GridMapError::InvalidOccupiedProbability);
        }
        self.set_log_odds(index, types::logit(probability))
    }

    /// Returns every cell to unknown, preserving the dimensions, configuration and allocation.
    #[cfg_attr(feature = "tracing", tracing::instrument("Reset Grid Map", skip_all))]
    pub(crate) fn reset(&mut self) {
        self.odds.fill(T::zero());
        self.last_frame_to_update.fill(0);
        self.frame = 1;
    }

    /// Opens a scan-scoped update session.
    ///
    /// Each cell absorbs at most one update per scan, however many beams cross it. The returned guard
    /// borrows the map mutably, so no second scan can begin while it lives.
    ///
    /// # Returns
    /// A [`ScanUpdater`] borrowing this map.
    #[must_use = "a ScanUpdater performs no work until beams are integrated into it"]
    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Begin Scan", skip_all, level = "debug")
    )]
    pub(crate) fn begin_scan(&mut self) -> ScanUpdater<'_, T, N> {
        // Bit 0 of a stamp records "marked occupied during this scan", so the generation occupies
        // bits 1 upwards. Exhausting 2^31 scans takes about 248 days at 100Hz; the sweep below is
        // a correctness backstop rather than an expected path, and without it a stale stamp would
        // eventually alias the current generation and silently skip that cell's update.
        if self.frame >= u32::MAX >> 1 {
            self.last_frame_to_update.fill(0);
            self.frame = 1;
        } else {
            self.frame += 1;
        }

        ScanUpdater::new(self)
    }

    /// Forces the scan generation counter, so the wraparound sweep can be tested directly.
    ///
    /// # Arguments
    /// * `frame`: the generation to set.
    #[cfg(test)]
    pub(super) fn force_frame(&mut self, frame: u32) {
        self.frame = frame;
    }
}

// The per-scan update machinery. Public access is through [`ScanUpdater`], which owns the
// invariant that a generation has been advanced first.
impl<T, const N: usize> GridMap<T, N>
where
    T: AsPrimitive<isize> + AsPrimitive<usize> + Copy + RealField,
    usize: AsPrimitive<T>,
{
    /// Clips a segment to the map's bounding box using the slab method.
    ///
    /// This is what bounds a ray cast: a step count follows from a beam's length, so an absurd but
    /// finite endpoint would otherwise be walked cell by cell only to be discarded.
    ///
    /// # Arguments
    /// * `origin`: the segment's start, in fractional cell coordinates.
    /// * `endpoint`: the segment's end, in fractional cell coordinates.
    ///
    /// # Returns
    /// An [`Option`] of the clipped start, the clipped end, and whether `endpoint` itself lay inside
    /// the map; [`None`] if the segment never enters it.
    fn clip_segment(
        &self,
        origin: &Point<T, N>,
        endpoint: &Point<T, N>,
    ) -> Option<(Point<T, N>, Point<T, N>, bool)> {
        let direction = endpoint - origin;
        let mut entry_fraction = T::zero();
        let mut exit_fraction = T::one();

        for axis in 0..N {
            let extent: T = self.dimensions[axis].as_();
            let delta = direction[axis];

            if delta.is_zero() {
                // Parallel to this slab: either wholly inside it, or the segment misses entirely.
                // The upper bound is exclusive, matching `cell_containing`: a coordinate of
                // exactly `extent` already lies in cell `extent`, which is off the map.
                if origin[axis] < T::zero() || origin[axis] >= extent {
                    return None;
                }
                continue;
            }

            let near = (T::zero() - origin[axis]) / delta;
            let far = (extent - origin[axis]) / delta;
            let (near, far) = if near > far { (far, near) } else { (near, far) };

            entry_fraction = entry_fraction.max(near);
            exit_fraction = exit_fraction.min(far);

            if entry_fraction > exit_fraction {
                return None;
            }
        }

        let entry = origin + direction * entry_fraction;
        let exit = origin + direction * exit_fraction;

        // The slab test above intersects the *closed* box, whereas the addressable domain is
        // half-open on every axis. A segment can therefore survive the clip while lying wholly
        // within an upper face, addressing only cells off the map; the projection below would
        // then slide it onto a row of real cells it never crossed. Both clipped endpoints being
        // on the face is exactly that case, the segment between them being linear.
        for axis in 0..N {
            let extent: T = self.dimensions[axis].as_();
            if entry[axis] >= extent && exit[axis] >= extent {
                return None;
            }
        }

        // Clipping lands the endpoints *on* the bounding faces, and a coordinate of exactly
        // `extent` belongs to cell `extent`, which is off the map. Worse, the plotter substitutes
        // the exact endpoint for its final step, so leaving it on the face would drop the last
        // cell inside the map as well as the one outside it. Pulling each coordinate back to the
        // centre of the cell it is leaving keeps the beam's final cell addressable, and never
        // changes which cell an already-interior coordinate falls in.
        let half = T::one() / (T::one() + T::one());
        let into_last_cell = |point: Point<T, N>| -> Point<T, N> {
            let mut point = point;
            for axis in 0..N {
                let limit: T = self.dimensions[axis].as_();
                point[axis] = point[axis].clamp(T::zero(), limit - half);
            }
            point
        };

        Some((
            into_last_cell(entry),
            into_last_cell(exit),
            // `exit_fraction` starts at one and only ever shrinks, so it is still exactly one
            // precisely when the caller's endpoint was never clipped away.
            // An endpoint resting on an upper face escaped clipping too,
            // but it addresses a cell off the map, so it is no more evidence of a return
            // than a clipped endpoint is:
            // reporting it as inside would plant a phantom obstacle in the boundary
            // cell the projection lands it in.
            exit_fraction >= T::one()
                && (0..N).all(|axis| {
                    let extent: T = self.dimensions[axis].as_();
                    (T::zero()..extent).contains(&exit[axis])
                }),
        ))
    }

    /// Records that a beam passed through a cell, applying the free increment.
    ///
    /// A cell already touched this scan, in either state, is left alone.
    ///
    /// # Arguments
    /// * `index`: the cell observed as free.
    ///
    /// # Returns
    /// A [`prim@bool`], whether the cell was updated.
    #[inline]
    fn mark_free_at(&mut self, index: &CellIndex<N>) -> bool {
        let Some(linear) = self.linearize(index) else {
            return false;
        };
        if self.last_frame_to_update[linear] >> 1 == self.frame {
            return false;
        }

        self.last_frame_to_update[linear] = self.frame << 1;
        self.odds[linear] =
            (self.odds[linear] + self.free_delta).clamp(self.min_log_odds, self.max_log_odds);
        true
    }

    /// Records that a beam terminated in a cell, applying the occupied increment.
    ///
    /// Occupancy wins within a scan: an earlier free update on the same cell is retracted first,
    /// exactly, unless the cell has saturated free - there what that update contributed is no
    /// longer recoverable, and nothing is retracted.
    ///
    /// # Arguments
    /// * `index`: the cell observed as occupied.
    ///
    /// # Returns
    /// A [`prim@bool`], whether the cell was updated.
    #[inline]
    fn mark_occupied_at(&mut self, index: &CellIndex<N>) -> bool {
        let Some(linear) = self.linearize(index) else {
            return false;
        };

        let current = self.frame << 1;
        let delta = if self.last_frame_to_update[linear] == current | 1 {
            return false;
        } else if self.last_frame_to_update[linear] == current {
            // Marked free earlier in this scan, so retract that first: the free increment is
            // negative, and subtracting it adds its magnitude back.
            //
            // Except on the floor, where the free update was clamped and how much of it survived
            // is gone. Crediting the full increment back there leaves the cell above where a lone
            // return would have put it, which makes a scan's outcome depend on the order its beams
            // happened to arrive in - the one thing the stamps exist to prevent.
            // A cell resting on the floor was almost certainly already there,
            // so retract nothing: exact in that case, and never optimistic in the narrow band above it.
            if self.odds[linear] <= self.min_log_odds {
                self.occupied_delta
            } else {
                self.occupied_delta - self.free_delta
            }
        } else {
            self.occupied_delta
        };

        self.last_frame_to_update[linear] = current | 1;
        self.odds[linear] = (self.odds[linear] + delta).clamp(self.min_log_odds, self.max_log_odds);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point2, Point3};

    /// A map with the default sensor model, which every test in this module shares.
    fn map<const N: usize>(dimensions: [usize; N]) -> GridMap<f32, N> {
        GridMap::new(dimensions, &GridMapConfig::default()).unwrap()
    }

    #[test]
    fn test_new_rejects_dimension_too_small() {
        // Both a zero extent and a single-cell extent are unusable: neither admits a stencil.
        for extent in [0, 1] {
            for axis in 0..3 {
                let mut dimensions = [4usize; 3];
                dimensions[axis] = extent;

                assert_eq!(
                    GridMap::<f32, 3>::new(dimensions, &GridMapConfig::default()).unwrap_err(),
                    GridMapError::DimensionTooSmall { axis, extent }
                );
            }
        }
    }

    #[test]
    fn test_new_rejects_capacity_overflow() {
        assert_eq!(
            GridMap::<f32, 3>::new([usize::MAX, usize::MAX, 2], &GridMapConfig::default())
                .unwrap_err(),
            GridMapError::CapacityOverflow
        );
    }

    #[test]
    fn test_new_propagates_config_errors() {
        let config = GridMapConfig::<f32>::builder()
            .with_free_probability(0.9)
            .build();

        assert_eq!(
            GridMap::<f32, 2>::new([4, 4], &config).unwrap_err(),
            GridMapError::InvalidFreeProbability
        );
    }

    #[test]
    fn test_new_reports_expected_shape() {
        let grid = map([3usize, 5, 7]);

        assert_eq!(grid.dimensions, [3, 5, 7]);
        assert_eq!(grid.odds.len(), 105);
        assert_eq!(grid.iter_log_odds().len(), 105);
    }

    #[test]
    fn test_new_initialises_every_cell_to_unknown() {
        let grid = map([3usize, 5]);

        assert!(grid.iter_log_odds().all(|odds| odds == 0.0));
        assert_eq!(
            grid.probability_at(&Point2::new(1, 2)),
            Some(0.5),
            "a never-written cell must read as exactly one half"
        );
    }

    #[test]
    fn test_log_odds_bounds_are_symmetric() {
        let grid = map([4usize, 4]);
        let bounds = grid.log_odds_bounds();

        assert_eq!(*bounds.start(), -*bounds.end());
        assert!((*bounds.end() - 3.476_098_7).abs() < 1e-5);
    }

    #[test]
    fn test_strides_2d_non_square() {
        // Axis 0 is fastest-varying, so its stride is always one.
        assert_eq!(map([3usize, 5]).strides, [1, 3]);
    }

    #[test]
    fn test_strides_3d_non_cubic() {
        assert_eq!(map([3usize, 5, 7]).strides, [1, 3, 15]);
    }

    #[test]
    fn test_linearize_is_bijective_2d() {
        let grid = map([3usize, 5]);
        let mut seen = Vec::new();

        for y in 0..5 {
            for x in 0..3 {
                seen.push(grid.linearize(&Point2::new(x, y)).unwrap());
            }
        }

        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), grid.odds.len());
        assert_eq!(*seen.last().unwrap(), grid.odds.len() - 1);
    }

    #[test]
    fn test_linearize_is_bijective_3d() {
        let grid = map([3usize, 5, 7]);
        let mut seen = Vec::new();

        for z in 0..7 {
            for y in 0..5 {
                for x in 0..3 {
                    seen.push(grid.linearize(&Point3::new(x, y, z)).unwrap());
                }
            }
        }

        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), grid.odds.len());
        assert_eq!(*seen.last().unwrap(), grid.odds.len() - 1);
    }

    /// The load-bearing test of the whole addressing scheme. Summing `coord * stride` and then
    /// bounds-checking the sum is unsound: in a `[4, 4]` map the cell `[4, 0]` sums to `4`, which
    /// is the perfectly valid cell `[0, 1]`. It is not enough to assert that the call fails -
    /// the aliased cell must be shown to be untouched.
    #[test]
    fn test_row_wrap_is_rejected() {
        let mut grid = map([4usize, 4]);
        let aliased = Point2::new(0, 1);
        assert_eq!(grid.linearize(&aliased), Some(4));

        assert_eq!(grid.linearize(&Point2::new(4, 0)), None);
        assert_eq!(
            grid.set_log_odds(&Point2::new(4, 0), 1.0).unwrap_err(),
            GridMapError::OutOfBounds {
                axis: 0,
                index: 4,
                extent: 4
            }
        );
        assert_eq!(
            grid.log_odds_at(&aliased),
            Some(0.0),
            "the write must not have landed on the cell the bad index aliases"
        );
    }

    #[test]
    fn test_column_wrap_is_rejected_3d() {
        let mut grid = map([4usize, 4, 4]);
        let aliased = Point3::new(0, 0, 1);
        assert_eq!(grid.linearize(&aliased), Some(16));

        // [0, 4, 0] sums to 16, which is the valid cell one slab along.
        assert_eq!(grid.linearize(&Point3::new(0, 4, 0)), None);
        assert!(grid.set_log_odds(&Point3::new(0, 4, 0), 1.0).is_err());
        assert_eq!(grid.log_odds_at(&aliased), Some(0.0));
    }

    #[test]
    fn test_negative_index_is_rejected_not_wrapped() {
        let grid = map([4usize, 4]);

        for index in [Point2::new(-1, 0), Point2::new(0, -1), Point2::new(-1, -1)] {
            assert_eq!(grid.linearize(&index), None);
            assert_eq!(grid.log_odds_at(&index), None);
        }
    }

    #[test]
    fn test_out_of_bounds_reports_first_failing_axis() {
        let grid = map([4usize, 5, 6]);

        assert_eq!(
            grid.linearize_checked(&Point3::new(9, 9, 9)).unwrap_err(),
            GridMapError::OutOfBounds {
                axis: 0,
                index: 9,
                extent: 4
            },
            "axes are checked in ascending order"
        );
        assert_eq!(
            grid.linearize_checked(&Point3::new(0, 9, 9)).unwrap_err(),
            GridMapError::OutOfBounds {
                axis: 1,
                index: 9,
                extent: 5
            }
        );
        assert_eq!(
            grid.linearize_checked(&Point3::new(0, 0, -3)).unwrap_err(),
            GridMapError::OutOfBounds {
                axis: 2,
                index: -3,
                extent: 6
            }
        );
    }

    #[test]
    fn test_cell_containing_resolves_interior() {
        let grid = map([10usize, 10]);

        assert_eq!(
            grid.cell_containing(&Point2::new(3.0, 4.0)).unwrap(),
            Point2::new(3, 4)
        );
        assert_eq!(
            grid.cell_containing(&Point2::new(3.9, 4.1)).unwrap(),
            Point2::new(3, 4),
            "a fractional coordinate resolves to the cell containing it"
        );
        assert_eq!(
            grid.cell_containing(&Point2::new(0.0, 0.0)).unwrap(),
            Point2::new(0, 0)
        );
    }

    /// A float-to-integer cast in Rust saturates and maps NaN to zero, so an unchecked conversion
    /// would silently redirect a NaN coordinate to the map origin rather than rejecting it.
    #[test]
    fn test_cell_containing_rejects_non_finite() {
        let grid = map([10usize, 10]);

        for (point, axis) in [
            (Point2::new(f32::NAN, 1.0), 0),
            (Point2::new(1.0, f32::NAN), 1),
            (Point2::new(f32::INFINITY, 1.0), 0),
            (Point2::new(1.0, f32::NEG_INFINITY), 1),
        ] {
            assert_eq!(
                grid.cell_containing(&point).unwrap_err(),
                GridMapError::NonFiniteCoordinate { axis }
            );
        }
    }

    #[test]
    fn test_cell_containing_rejects_out_of_range() {
        let grid = map([10usize, 10]);

        // A coordinate of exactly the extent is already past the final cell.
        assert!(grid.cell_containing(&Point2::new(10.0, 1.0)).is_err());
        assert!(grid.cell_containing(&Point2::new(9.999, 1.0)).is_ok());
        // Truncation toward zero would fold -0.5 into cell zero; it must be rejected instead.
        assert!(grid.cell_containing(&Point2::new(-0.5, 1.0)).is_err());
    }

    #[test]
    fn test_set_log_odds_clamps_to_bounds() {
        let mut grid = map([4usize, 4]);
        let index = Point2::new(1, 1);
        let bounds = grid.log_odds_bounds();

        grid.set_log_odds(&index, 1000.0).unwrap();
        assert_eq!(grid.log_odds_at(&index), Some(*bounds.end()));

        grid.set_log_odds(&index, -1000.0).unwrap();
        assert_eq!(grid.log_odds_at(&index), Some(*bounds.start()));
    }

    #[test]
    fn test_set_probability_round_trips_through_log_odds() {
        let mut grid = map([4usize, 4]);
        let index = Point2::new(2, 3);

        grid.set_probability(&index, 0.8).unwrap();
        assert!((grid.probability_at(&index).unwrap() - 0.8).abs() < 1e-6);
    }

    #[test]
    fn test_set_probability_rejects_zero_and_one() {
        let mut grid = map([4usize, 4]);
        let index = Point2::new(0, 0);

        // Either endpoint is infinite in log-odds, so both are refused.
        for probability in [0.0, 1.0, -0.5, 1.5] {
            assert_eq!(
                grid.set_probability(&index, probability).unwrap_err(),
                GridMapError::InvalidOccupiedProbability
            );
        }
        assert_eq!(grid.log_odds_at(&index), Some(0.0));
    }

    #[test]
    fn test_log_odds_and_probability_agree() {
        let mut grid = map([4usize, 4]);
        let index = Point2::new(1, 2);
        grid.set_log_odds(&index, 1.5).unwrap();

        let expected = 1.0 / (1.0 + (-1.5f32).exp());
        assert!((grid.probability_at(&index).unwrap() - expected).abs() < 1e-6);
    }

    #[test]
    fn test_reset_returns_every_cell_to_unknown() {
        let mut grid = map([4usize, 4]);
        grid.set_log_odds(&Point2::new(1, 1), 2.0).unwrap();
        grid.set_log_odds(&Point2::new(2, 2), -2.0).unwrap();

        grid.reset();

        assert!(grid.iter_log_odds().all(|odds| odds == 0.0));
        assert_eq!(grid.dimensions, [4, 4], "the shape must survive a reset");
    }

    #[test]
    fn test_iter_log_odds_order_is_axis_zero_fastest() {
        let mut grid = map([3usize, 5]);
        grid.set_log_odds(&Point2::new(1, 0), 1.0).unwrap();
        grid.set_log_odds(&Point2::new(0, 1), 2.0).unwrap();

        let odds = grid.iter_log_odds().collect::<Vec<_>>();
        assert_eq!(odds.len(), 15);
        assert_eq!(odds[1], 1.0, "a step along axis 0 moves one element");
        assert_eq!(
            odds[3], 2.0,
            "a step along axis 1 moves dimensions[0] elements"
        );
    }

    #[test]
    fn test_reads_take_shared_reference() {
        let grid = map([4usize, 4]);

        // Two live shared borrows, both querying: a scan matcher needs exactly this.
        let first = &grid;
        let second = &grid;
        assert_eq!(
            first.log_odds_at(&Point2::new(1, 1)),
            second.log_odds_at(&Point2::new(1, 1))
        );
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_debug_omits_cell_contents() {
        let rendered = format!("{:?}", map([64usize, 64]));

        assert!(rendered.contains("dimensions"));
        assert!(rendered.contains("cell_count"));
        assert!(
            rendered.len() < 512,
            "Debug must not render four thousand cells: {rendered}"
        );
    }

    #[test]
    fn test_clip_segment_entirely_inside_is_identity() {
        let grid = map([10usize, 10]);
        let (entry, exit, inside) = grid
            .clip_segment(&Point2::new(1.0, 1.0), &Point2::new(5.0, 5.0))
            .unwrap();

        assert_eq!(entry, Point2::new(1.0, 1.0));
        assert_eq!(exit, Point2::new(5.0, 5.0));
        assert!(inside);
    }

    #[test]
    fn test_clip_segment_entirely_outside_is_none() {
        let grid = map([10usize, 10]);

        assert!(
            grid.clip_segment(&Point2::new(-5.0, -5.0), &Point2::new(-1.0, -1.0))
                .is_none()
        );
        assert!(
            grid.clip_segment(&Point2::new(20.0, 1.0), &Point2::new(30.0, 1.0))
                .is_none()
        );
        // Parallel to the slab and outside it: the axis-aligned early exit.
        assert!(
            grid.clip_segment(&Point2::new(1.0, -3.0), &Point2::new(8.0, -3.0))
                .is_none()
        );
    }

    #[test]
    fn test_clip_segment_trims_the_far_end() {
        let grid = map([10usize, 10]);
        let (entry, exit, inside) = grid
            .clip_segment(&Point2::new(5.0, 5.0), &Point2::new(25.0, 5.0))
            .unwrap();

        assert_eq!(entry, Point2::new(5.0, 5.0));
        assert_eq!(
            exit,
            Point2::new(9.5, 5.0),
            "the exit is pulled back to the centre of the last cell, since a coordinate of              exactly 10.0 addresses cell 10, which is off the map"
        );
        assert!(!inside, "the caller's endpoint was clipped away");
    }

    #[test]
    fn test_clip_segment_trims_the_near_end() {
        let grid = map([10usize, 10]);
        let (entry, exit, inside) = grid
            .clip_segment(&Point2::new(-10.0, 5.0), &Point2::new(5.0, 5.0))
            .unwrap();

        assert_eq!(entry, Point2::new(0.0, 5.0));
        assert_eq!(exit, Point2::new(5.0, 5.0));
        assert!(inside, "the caller's endpoint was inside all along");
    }

    #[test]
    fn test_clip_segment_passing_clean_through() {
        let grid = map([10usize, 10, 10]);
        let (entry, exit, inside) = grid
            .clip_segment(&Point3::new(-8.0, 5.0, 5.0), &Point3::new(18.0, 5.0, 5.0))
            .unwrap();

        assert_eq!(entry, Point3::new(0.0, 5.0, 5.0));
        assert_eq!(exit, Point3::new(9.5, 5.0, 5.0));
        assert!(!inside);
    }

    /// The addressable domain is half-open, so a beam lying along `y == extent` is in row 8 of an
    /// eight-row map: off it. Accepting the segment and then projecting it into the last cell
    /// would write free space across a row the beam never crossed, and the acceptance turned on
    /// an exact float equality - one ulp further out the same beam was rejected outright.
    #[test]
    fn test_clip_segment_rejects_a_segment_lying_on_the_upper_face() {
        let grid = map([8usize, 8]);

        assert!(
            grid.clip_segment(&Point2::new(2.0, 8.0), &Point2::new(5.0, 8.0))
                .is_none()
        );
        // The same beam a hair inside is still a row-7 beam and must survive.
        let (entry, exit, _) = grid
            .clip_segment(&Point2::new(2.0, 7.9999), &Point2::new(5.0, 7.9999))
            .unwrap();
        assert_eq!(entry, Point2::new(2.0, 7.5));
        assert_eq!(exit, Point2::new(5.0, 7.5));
    }

    /// The non-parallel counterpart, which the slab test lets through: `entry_fraction` and
    /// `exit_fraction` both collapse to zero, leaving a degenerate segment sitting on the face.
    #[test]
    fn test_clip_segment_rejects_a_segment_leaving_from_the_upper_face() {
        let grid = map([8usize, 8]);

        assert!(
            grid.clip_segment(&Point2::new(8.0, 3.0), &Point2::new(9.0, 3.0))
                .is_none()
        );
        // Starting on the same face but aiming *inward* genuinely enters the map.
        let (entry, exit, _) = grid
            .clip_segment(&Point2::new(8.0, 3.0), &Point2::new(7.0, 3.0))
            .unwrap();
        assert_eq!(entry, Point2::new(7.5, 3.0));
        assert_eq!(exit, Point2::new(7.0, 3.0));
    }

    /// An endpoint on an upper face escapes clipping, so `exit_fraction` is still exactly one -
    /// but the cell it addresses is off the map, and the caller reads "inside" as "the beam really
    /// stopped here". It must read as clipped, exactly as the endpoint one ulp beyond it does.
    #[test]
    fn test_clip_segment_endpoint_on_the_upper_face_is_not_inside() {
        let grid = map([8usize, 8]);

        let (_, on_face, inside) = grid
            .clip_segment(&Point2::new(4.5, 4.5), &Point2::new(8.0, 4.5))
            .unwrap();
        assert!(!inside);

        let (_, past_face, still_outside) = grid
            .clip_segment(&Point2::new(4.5, 4.5), &Point2::new(8.0001, 4.5))
            .unwrap();
        assert!(!still_outside);
        assert_eq!(
            on_face, past_face,
            "the cells covered must not turn on an exact float equality either"
        );
    }

    /// A small deterministic generator, so the differential tests are reproducible without
    /// pulling `rand` into this crate.
    struct Lcg(u64);

    impl Lcg {
        fn next_unit(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
        }
    }

    /// Fills a map with pseudo-random log-odds spanning most of the clamp band.
    fn noisy_map<const N: usize>(dimensions: [usize; N], seed: u64) -> GridMap<f64, N> {
        let mut grid = GridMap::<f64, N>::new(dimensions, &GridMapConfig::default()).unwrap();
        let mut rng = Lcg(seed);
        for cell in grid.odds.iter_mut() {
            *cell = rng.next_unit() * 6.0 - 3.0;
        }
        grid
    }

    #[test]
    fn test_bilinear_matches_hand_computed() {
        let mut grid = GridMap::<f64, 2>::new([2, 2], &GridMapConfig::default()).unwrap();
        grid.set_log_odds(&Point2::new(0, 0), 0.1).unwrap();
        grid.set_log_odds(&Point2::new(1, 0), 0.2).unwrap();
        grid.set_log_odds(&Point2::new(0, 1), 0.3).unwrap();
        grid.set_log_odds(&Point2::new(1, 1), 0.4).unwrap();

        // lower = 0.1 + 0.1*0.25 = 0.125; upper = 0.3 + 0.1*0.25 = 0.325
        // value = 0.125 + (0.325 - 0.125)*0.75 = 0.275
        let sample = grid.sample_log_odds(&Point2::new(0.25, 0.75)).unwrap();
        assert!((sample.value - 0.275).abs() < 1e-12);
        assert!((sample.gradient[0] - 0.1).abs() < 1e-12);
        assert!((sample.gradient[1] - 0.2).abs() < 1e-12);
    }

    #[test]
    fn test_trilinear_matches_hand_computed() {
        let mut grid = GridMap::<f64, 3>::new([2, 2, 2], &GridMapConfig::default()).unwrap();
        for z in 0..2 {
            for y in 0..2 {
                for x in 0..2 {
                    let value = 0.1 * f64::from(1 + x + 2 * y + 4 * z);
                    grid.set_log_odds(&Point3::new(x as isize, y as isize, z as isize), value)
                        .unwrap();
                }
            }
        }

        // At the stencil centre the value is the mean of all eight corners.
        let sample = grid.sample_log_odds(&Point3::new(0.5, 0.5, 0.5)).unwrap();
        assert!((sample.value - 0.45).abs() < 1e-12);
        assert!((sample.gradient[0] - 0.1).abs() < 1e-12);
        assert!((sample.gradient[1] - 0.2).abs() < 1e-12);
        assert!((sample.gradient[2] - 0.4).abs() < 1e-12);
    }

    /// On a field that is exactly linear, interpolation must be exact and the gradient constant.
    #[test]
    fn test_linear_ramp_is_exact_with_constant_gradient() {
        let mut grid = GridMap::<f64, 2>::new([8, 8], &GridMapConfig::default()).unwrap();
        let ramp = |x: f64, y: f64| 0.25 * x - 0.125 * y + 0.5;

        for y in 0..8 {
            for x in 0..8 {
                grid.set_log_odds(
                    &Point2::new(x as isize, y as isize),
                    ramp(f64::from(x), f64::from(y)),
                )
                .unwrap();
            }
        }

        let mut rng = Lcg(0x1234);
        for _ in 0..500 {
            let (x, y) = (rng.next_unit() * 7.0, rng.next_unit() * 7.0);
            let sample = grid.sample_log_odds(&Point2::new(x, y)).unwrap();

            assert!((sample.value - ramp(x, y)).abs() < 1e-12);
            assert!((sample.gradient[0] - 0.25).abs() < 1e-12);
            assert!((sample.gradient[1] - -0.125).abs() < 1e-12);
        }
    }

    #[test]
    fn test_value_is_exact_at_integer_coordinates() {
        let grid = noisy_map([6usize, 6], 0xABCD);

        for y in 0..5 {
            for x in 0..5 {
                let sample = grid
                    .sample_log_odds(&Point2::new(f64::from(x), f64::from(y)))
                    .unwrap();
                let cell = grid
                    .log_odds_at(&Point2::new(x as isize, y as isize))
                    .unwrap();
                assert!((sample.value - cell).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn test_gradient_matches_central_differences_2d() {
        let grid = noisy_map([9usize, 9], 0x2222);
        let step = 1e-5;
        let mut rng = Lcg(0x3333);

        for _ in 0..500 {
            let point = Point2::new(1.0 + rng.next_unit() * 6.0, 1.0 + rng.next_unit() * 6.0);
            let sample = grid.sample_log_odds(&point).unwrap();

            for axis in 0..2 {
                let (mut low, mut high) = (point, point);
                low[axis] -= step;
                high[axis] += step;
                let numeric = (grid.sample_log_odds(&high).unwrap().value
                    - grid.sample_log_odds(&low).unwrap().value)
                    / (2.0 * step);

                assert!(
                    (sample.gradient[axis] - numeric).abs() < 1e-6,
                    "axis {axis} at {point:?}: {} vs {numeric}",
                    sample.gradient[axis]
                );
            }
        }
    }

    #[test]
    fn test_gradient_matches_central_differences_3d() {
        let grid = noisy_map([7usize, 7, 7], 0x4444);
        let step = 1e-5;
        let mut rng = Lcg(0x5555);

        for _ in 0..500 {
            let point = Point3::new(
                1.0 + rng.next_unit() * 4.0,
                1.0 + rng.next_unit() * 4.0,
                1.0 + rng.next_unit() * 4.0,
            );
            let sample = grid.sample_log_odds(&point).unwrap();

            for axis in 0..3 {
                let (mut low, mut high) = (point, point);
                low[axis] -= step;
                high[axis] += step;
                let numeric = (grid.sample_log_odds(&high).unwrap().value
                    - grid.sample_log_odds(&low).unwrap().value)
                    / (2.0 * step);

                assert!(
                    (sample.gradient[axis] - numeric).abs() < 1e-6,
                    "axis {axis}"
                );
            }
        }
    }

    /// A non-square map whose field varies along x alone. A transposed stride would show up here
    /// as a gradient pointing the wrong way, which a square map would hide.
    #[test]
    fn test_gradient_axis_order_2d() {
        let mut grid = GridMap::<f64, 2>::new([3, 9], &GridMapConfig::default()).unwrap();
        for y in 0..9 {
            for x in 0..3 {
                grid.set_log_odds(&Point2::new(x, y as isize), 0.5 * x as f64)
                    .unwrap();
            }
        }

        let sample = grid.sample_log_odds(&Point2::new(1.5, 4.5)).unwrap();
        assert!((sample.gradient[0] - 0.5).abs() < 1e-12);
        assert!(sample.gradient[1].abs() < 1e-12);
    }

    /// Interpolation needs a neighbour on the far side, so the usable domain stops at
    /// `dimensions[axis] - 1`, inclusive. The final row contributes only its low corner.
    #[test]
    fn test_sample_at_upper_domain_edge_is_inclusive() {
        let grid = noisy_map([4usize, 4], 0x6666);

        assert!(grid.sample_log_odds(&Point2::new(3.0, 3.0)).is_some());
        assert!(grid.sample_log_odds(&Point2::new(3.0001, 3.0)).is_none());
        assert!(grid.sample_log_odds(&Point2::new(3.0, 3.0001)).is_none());
        assert!(grid.sample_log_odds(&Point2::new(4.0, 1.0)).is_none());
    }

    #[test]
    fn test_sample_at_upper_domain_edge_is_inclusive_3d() {
        let grid = noisy_map([4usize, 5, 6], 0x7777);

        assert!(grid.sample_log_odds(&Point3::new(3.0, 4.0, 5.0)).is_some());
        assert!(
            grid.sample_log_odds(&Point3::new(3.0, 4.0, 5.0001))
                .is_none()
        );
    }

    /// `f64::NAN as usize` is zero, so an unchecked cast would sample the map origin and return a
    /// plausible number for a meaningless query.
    #[test]
    fn test_sample_rejects_non_finite_coordinate() {
        let grid = noisy_map([8usize, 8], 0x8888);

        for point in [
            Point2::new(f64::NAN, 2.0),
            Point2::new(2.0, f64::NAN),
            Point2::new(f64::INFINITY, 2.0),
            Point2::new(2.0, f64::NEG_INFINITY),
        ] {
            assert!(grid.sample_log_odds(&point).is_none(), "{point:?}");
            assert!(grid.sample_probability(&point).is_none(), "{point:?}");
        }
    }

    #[test]
    fn test_sample_rejects_non_finite_coordinate_3d() {
        let grid = noisy_map([6usize, 6, 6], 0x9999);
        assert!(
            grid.sample_log_odds(&Point3::new(1.0, f64::NAN, 1.0))
                .is_none()
        );
    }

    /// Truncation toward zero would fold any coordinate in `(-1, 0)` into cell zero, so a point
    /// just off the low corner must be rejected rather than silently clamped inward.
    #[test]
    fn test_sample_rejects_negative_coordinate() {
        let grid = noisy_map([8usize, 8], 0xAAAA);

        assert!(grid.sample_log_odds(&Point2::new(-0.5, 2.0)).is_none());
        assert!(grid.sample_log_odds(&Point2::new(2.0, -0.5)).is_none());
        assert!(grid.sample_log_odds(&Point2::new(-1e-9, 2.0)).is_none());
        assert!(grid.sample_log_odds(&Point2::new(0.0, 2.0)).is_some());
    }

    #[test]
    fn test_sample_probability_applies_the_chain_rule() {
        let grid = noisy_map([8usize, 8], 0xBBBB);
        let mut rng = Lcg(0xCCCC);

        for _ in 0..500 {
            let point = Point2::new(rng.next_unit() * 7.0, rng.next_unit() * 7.0);
            let log_odds = grid.sample_log_odds(&point).unwrap();
            let probability = grid.sample_probability(&point).unwrap();

            let expected = 1.0 / (1.0 + (-log_odds.value).exp());
            assert!((probability.value - expected).abs() < 1e-12);

            // dP/dx = P * (1 - P) * dl/dx
            let scale = expected * (1.0 - expected);
            assert!((probability.gradient - log_odds.gradient * scale).norm() < 1e-12);
        }
    }

    #[test]
    fn test_sample_probability_gradient_matches_central_differences() {
        let grid = noisy_map([9usize, 9], 0xDDDD);
        let step = 1e-5;
        let mut rng = Lcg(0xEEEE);

        for _ in 0..300 {
            let point = Point2::new(1.0 + rng.next_unit() * 6.0, 1.0 + rng.next_unit() * 6.0);
            let sample = grid.sample_probability(&point).unwrap();

            for axis in 0..2 {
                let (mut low, mut high) = (point, point);
                low[axis] -= step;
                high[axis] += step;
                let numeric = (grid.sample_probability(&high).unwrap().value
                    - grid.sample_probability(&low).unwrap().value)
                    / (2.0 * step);

                assert!(
                    (sample.gradient[axis] - numeric).abs() < 1e-6,
                    "axis {axis}"
                );
            }
        }
    }

    #[test]
    fn test_sample_probability_3d_matches_logistic() {
        let grid = noisy_map([5usize, 5, 5], 0xF0F0);
        let point = Point3::new(1.25, 2.5, 3.75);

        let log_odds = grid.sample_log_odds(&point).unwrap();
        let probability = grid.sample_probability(&point).unwrap();

        let expected = 1.0 / (1.0 + (-log_odds.value).exp());
        assert!((probability.value - expected).abs() < 1e-12);
        assert!(probability.value > 0.0 && probability.value < 1.0);
    }

    /// An unwritten map is uniform, so every gradient must vanish exactly.
    #[test]
    fn test_gradient_is_zero_in_a_uniform_field() {
        let grid = GridMap::<f64, 2>::new([8, 8], &GridMapConfig::default()).unwrap();
        let sample = grid.sample_log_odds(&Point2::new(3.25, 4.75)).unwrap();

        assert_eq!(sample.value, 0.0);
        assert_eq!(sample.gradient, SVector::<f64, 2>::zeros());
        assert_eq!(
            grid.sample_probability(&Point2::new(3.25, 4.75))
                .unwrap()
                .value,
            0.5
        );
    }

    /// A sample sitting exactly on the last column of an *interior* row is the dangerous case:
    /// the neighbour gather would still succeed, having quietly walked into the next row, and
    /// return a plausible value with a gradient built from a cell that is nowhere near the query.
    /// The far corner merely fails; this one lies.
    #[test]
    fn test_upper_edge_does_not_borrow_the_next_row() {
        let mut grid = GridMap::<f64, 2>::new([4, 4], &GridMapConfig::default()).unwrap();

        // A field varying along x alone, so any leakage across a row boundary is unmistakable.
        for y in 0..4 {
            for x in 0..4 {
                grid.set_log_odds(&Point2::new(x, y), f64::from(x as i32))
                    .unwrap();
            }
        }

        let sample = grid.sample_log_odds(&Point2::new(3.0, 1.0)).unwrap();

        assert_eq!(
            sample.value, 3.0,
            "the value must be the cell itself, not a blend with the next row"
        );
        assert!(
            (sample.gradient[0] - 1.0).abs() < 1e-12,
            "the x gradient must come from cells (2,1) and (3,1); borrowing (0,2) would give -3"
        );
        assert!(sample.gradient[1].abs() < 1e-12);

        // The same must hold along the other axis, and at the far corner of the map.
        let corner = grid.sample_log_odds(&Point2::new(3.0, 3.0)).unwrap();
        assert_eq!(corner.value, 3.0);
        assert!((corner.gradient[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_upper_edge_does_not_borrow_the_next_slab_3d() {
        let mut grid = GridMap::<f64, 3>::new([3, 3, 3], &GridMapConfig::default()).unwrap();
        for z in 0..3 {
            for y in 0..3 {
                for x in 0..3 {
                    grid.set_log_odds(&Point3::new(x, y, z), f64::from(y as i32))
                        .unwrap();
                }
            }
        }

        let sample = grid.sample_log_odds(&Point3::new(1.0, 2.0, 1.0)).unwrap();
        assert_eq!(sample.value, 2.0);
        assert!(sample.gradient[0].abs() < 1e-12);
        assert!((sample.gradient[1] - 1.0).abs() < 1e-12);
        assert!(sample.gradient[2].abs() < 1e-12);
    }
}
