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

use nalgebra::{Point, RealField, SVector};
use num_traits::AsPrimitive;

use super::{GridMap, MapSample};

// The interpolation kernels are written once per supported dimension, as plain inherent impls on
// the concrete `N`. Rust permits several inherent impls on disjoint concrete const-generic
// arguments, so no trait and no marker type are needed here.
//
// The kernels are unrolled rather than looped over `2^N` corners because the gradient with respect
// to axis `k` replaces the `k`th interpolation weight with a difference. A corner loop therefore
// computes `N + 1` independent weighted sums that share nothing, whereas the factored form below
// reuses almost every intermediate: roughly 9 operations against 24 in two dimensions, and 22
// against 128 in three.
//
// Each pair of x-neighbours is adjacent in memory, so `first_chunk` fetches it with a single
// length check and no further bounds checking. `stencil_base` has already proven the whole
// stencil is in range, so those checks never fail; they are what keeps this safe code.

impl<T> GridMap<T, 2>
where
    T: AsPrimitive<isize> + AsPrimitive<usize> + Copy + RealField,
    usize: AsPrimitive<T>,
{
    /// Bilinearly interpolates the log-odds field and its analytic gradient.
    ///
    /// Performs no transcendental operations; the four corner loads are shared between the value and
    /// both gradient components.
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
    #[inline]
    pub(crate) fn sample_log_odds(&self, point: &Point<T, 2>) -> Option<MapSample<T, 2>> {
        let (base, [fx, fy]) = self.stencil_base(point)?;

        let [c00, c10] = *self.odds.get(base..)?.first_chunk::<2>()?;
        let [c01, c11] = *self
            .odds
            .get(base + self.strides[1]..)?
            .first_chunk::<2>()?;

        let dx0 = c10 - c00;
        let dx1 = c11 - c01;
        let lower = c00 + dx0 * fx;
        let upper = c01 + dx1 * fx;

        Some(MapSample {
            value: lower + (upper - lower) * fy,
            gradient: SVector::from([dx0 + (dx1 - dx0) * fy, upper - lower]),
        })
    }

    /// Bilinearly interpolates the log-odds field, then maps it through the logistic function.
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
    #[inline]
    pub(crate) fn sample_probability(&self, point: &Point<T, 2>) -> Option<MapSample<T, 2>> {
        self.sample_log_odds(point).map(Self::sample_to_probability)
    }
}

impl<T> GridMap<T, 3>
where
    T: AsPrimitive<isize> + AsPrimitive<usize> + Copy + RealField,
    usize: AsPrimitive<T>,
{
    /// Trilinearly interpolates the log-odds field and its analytic gradient.
    ///
    /// The three-dimensional counterpart of [`GridMap::<T, 2>::sample_log_odds`](GridMap::sample_log_odds),
    /// carrying the same warning about the choice of field.
    ///
    /// # Arguments
    /// * `point`: a position in fractional cell coordinates.
    ///
    /// # Returns
    /// An [`Option`] of [`MapSample`], or [`None`] if `point` is non-finite or outside the map.
    #[inline]
    pub(crate) fn sample_log_odds(&self, point: &Point<T, 3>) -> Option<MapSample<T, 3>> {
        let (base, [fx, fy, fz]) = self.stencil_base(point)?;
        let (stride_y, stride_z) = (self.strides[1], self.strides[2]);

        let [c000, c100] = *self.odds.get(base..)?.first_chunk::<2>()?;
        let [c010, c110] = *self.odds.get(base + stride_y..)?.first_chunk::<2>()?;
        let [c001, c101] = *self.odds.get(base + stride_z..)?.first_chunk::<2>()?;
        let [c011, c111] = *self
            .odds
            .get(base + stride_y + stride_z..)?
            .first_chunk::<2>()?;

        // Collapse along x, keeping the differences: they are the x-derivative of each edge.
        let (dx00, dx10) = (c100 - c000, c110 - c010);
        let (dx01, dx11) = (c101 - c001, c111 - c011);
        let (a00, a10) = (c000 + dx00 * fx, c010 + dx10 * fx);
        let (a01, a11) = (c001 + dx01 * fx, c011 + dx11 * fx);

        // Then along y, and the remaining difference is the z-derivative outright.
        let (dy0, dy1) = (a10 - a00, a11 - a01);
        let (b0, b1) = (a00 + dy0 * fy, a01 + dy1 * fy);
        let dz = b1 - b0;

        // The x-derivative of each z-face, interpolated along y.
        let ex0 = dx00 + (dx10 - dx00) * fy;
        let ex1 = dx01 + (dx11 - dx01) * fy;

        Some(MapSample {
            value: b0 + dz * fz,
            gradient: SVector::from([ex0 + (ex1 - ex0) * fz, dy0 + (dy1 - dy0) * fz, dz]),
        })
    }

    /// Trilinearly interpolates the log-odds field, then maps it through the logistic function.
    ///
    /// # Arguments
    /// * `point`: a position in fractional cell coordinates.
    ///
    /// # Returns
    /// An [`Option`] of [`MapSample`] whose value is a probability in the range `0.0..=1.0`, or
    /// [`None`] if `point` is non-finite or outside the map.
    #[inline]
    pub(crate) fn sample_probability(&self, point: &Point<T, 3>) -> Option<MapSample<T, 3>> {
        self.sample_log_odds(point).map(Self::sample_to_probability)
    }
}

#[cfg(test)]
mod tests {
    use super::super::GridMapConfig;
    use super::*;
    use nalgebra::{Point2, Point3};

    /// A straightforward `2^N`-corner implementation, kept only to test the unrolled kernels
    /// against. It is written for clarity rather than speed: the value is a weighted sum over
    /// every corner, and each gradient component is a weighted sum of the differences across that
    /// axis, with the axis's own weight left out.
    fn reference_sample<T, const N: usize>(
        grid: &GridMap<T, N>,
        point: &Point<T, N>,
    ) -> Option<MapSample<T, N>>
    where
        T: AsPrimitive<isize> + AsPrimitive<usize> + Copy + RealField,
        usize: AsPrimitive<T>,
    {
        let (base, frac) = grid.stencil_base(point)?;

        // The product of per-axis weights for one corner, optionally omitting one axis.
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
        let corner_value = |corner: usize| -> T {
            grid.odds[(0..N).fold(base, |acc, axis| {
                acc + if corner >> axis & 1 == 1 {
                    grid.strides[axis]
                } else {
                    0
                }
            })]
        };

        let corners = 1usize << N;
        let value = (0..corners).fold(T::zero(), |acc, corner| {
            acc + weight(corner, None) * corner_value(corner)
        });

        let mut gradient = SVector::<T, N>::zeros();
        for axis in 0..N {
            gradient[axis] = (0..corners).filter(|corner| corner >> axis & 1 == 0).fold(
                T::zero(),
                |acc, corner| {
                    acc + weight(corner, Some(axis))
                        * (corner_value(corner | (1 << axis)) - corner_value(corner))
                },
            );
        }

        Some(MapSample { value, gradient })
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

    #[test]
    fn test_unrolled_matches_generic_reference_2d() {
        let grid = noisy_map([9usize, 7], 0x5EED_2D);
        let mut rng = Lcg(0xC0FFEE);

        for _ in 0..2000 {
            let point = Point2::new(rng.next_unit() * 8.0, rng.next_unit() * 6.0);
            let unrolled = grid.sample_log_odds(&point).unwrap();
            let reference = reference_sample(&grid, &point).unwrap();

            assert!(
                (unrolled.value - reference.value).abs() < 1e-12,
                "{point:?}"
            );
            assert!(
                (unrolled.gradient - reference.gradient).norm() < 1e-12,
                "{point:?}"
            );
        }
    }

    #[test]
    fn test_unrolled_matches_generic_reference_3d() {
        let grid = noisy_map([7usize, 6, 5], 0x5EED_3D);
        let mut rng = Lcg(0xBADC0DE);

        for _ in 0..2000 {
            let point = Point3::new(
                rng.next_unit() * 6.0,
                rng.next_unit() * 5.0,
                rng.next_unit() * 4.0,
            );
            let unrolled = grid.sample_log_odds(&point).unwrap();
            let reference = reference_sample(&grid, &point).unwrap();

            assert!(
                (unrolled.value - reference.value).abs() < 1e-12,
                "{point:?}"
            );
            assert!(
                (unrolled.gradient - reference.gradient).norm() < 1e-12,
                "{point:?}"
            );
        }
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
    /// the paired load would still succeed, having quietly walked into the next row, and return a
    /// plausible value with a gradient built from a cell that is nowhere near the query. The
    /// far corner merely fails; this one lies.
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
