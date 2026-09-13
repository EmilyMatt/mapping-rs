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

use mapping_algorithms::lines::{BresenhamError, BresenhamLine};
use nalgebra::{ComplexField, Point, RealField};
use num_traits::AsPrimitive;

use super::{CellIndex, GridMap, GridMapError, GridMapResult, RayTermination};

/// A scan-scoped update session over a [`GridMap`], from [`GridMap::begin_scan`].
///
/// Each cell absorbs at most one update per scan, and an occupied observation beats a free one.
///
/// # Generics
/// * `T`: Either an [`prim@f32`] or [`prim@f64`].
/// * `N`: a usize, representing the number of dimensions.
#[derive(Debug)]
pub(crate) struct ScanUpdater<'a, T, const N: usize> {
    map: &'a mut GridMap<T, N>,
}

impl<'a, T, const N: usize> ScanUpdater<'a, T, N>
where
    T: AsPrimitive<isize> + AsPrimitive<usize> + Copy + RealField,
    usize: AsPrimitive<T>,
{
    /// Wraps a map whose generation counter has already been advanced.
    ///
    /// # Arguments
    /// * `map`: the map to borrow for the duration of the scan.
    ///
    /// # Returns
    /// A [`ScanUpdater`] borrowing `map`.
    pub(super) fn new(map: &'a mut GridMap<T, N>) -> Self {
        Self { map }
    }

    /// Borrows the map for reading mid-scan.
    ///
    /// # Returns
    /// A shared reference to the [`GridMap`], reflecting the scan so far.
    pub(crate) fn map(&self) -> &GridMap<T, N> {
        self.map
    }

    /// Records that a beam passed through a cell.
    ///
    /// # Arguments
    /// * `index`: the cell observed as free.
    ///
    /// # Errors
    /// * [`GridMapError::OutOfBounds`]: naming the first offending axis. Use
    ///   [`update_ray`](Self::update_ray) for geometry expected to leave the map, which clips instead.
    pub(crate) fn mark_free(&mut self, index: &CellIndex<N>) -> GridMapResult<()> {
        self.map.linearize_checked(index)?;
        self.map.mark_free_at(index);
        Ok(())
    }

    /// Records that a beam terminated in a cell, retracting any free update from this same scan.
    ///
    /// # Arguments
    /// * `index`: the cell observed as occupied.
    ///
    /// # Errors
    /// * [`GridMapError::OutOfBounds`]: naming the first offending axis.
    pub(crate) fn mark_occupied(&mut self, index: &CellIndex<N>) -> GridMapResult<()> {
        self.map.linearize_checked(index)?;
        self.map.mark_occupied_at(index);
        Ok(())
    }

    /// Integrates one range measurement, marking the cells it crosses free and applying
    /// `termination` to the cell it stops in.
    ///
    /// The segment is clipped to the map first, so a beam aimed far outside costs no more than one
    /// crossing it. Allocates nothing.
    ///
    /// # Arguments
    /// * `origin`: the sensor position, in fractional cell coordinates.
    /// * `endpoint`: where the beam stopped, in fractional cell coordinates.
    /// * `termination`: whether the beam stopped on a return or at its maximum range.
    ///
    /// # Returns
    /// A [`prim@usize`], the number of cells updated; zero if the beam missed the map.
    ///
    /// # Errors
    /// * [`GridMapError::Ray`]: if either point carries a NaN or infinite coordinate.
    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Update Ray", skip_all, level = "trace")
    )]
    pub(crate) fn update_ray(
        &mut self,
        origin: &Point<T, N>,
        endpoint: &Point<T, N>,
        termination: RayTermination,
    ) -> GridMapResult<usize> {
        // Checked up front rather than left to the plotter: clipping an infinite endpoint would
        // silently collapse the beam to zero length instead of reporting the bad input.
        for axis in 0..N {
            if !ComplexField::is_finite(&origin[axis]) || !ComplexField::is_finite(&endpoint[axis])
            {
                return Err(GridMapError::Ray(BresenhamError::NonFiniteCoordinate));
            }
        }

        let Some((entry, exit, endpoint_is_inside)) = self.map.clip_segment(origin, endpoint)
        else {
            return Ok(0);
        };

        let mut line = BresenhamLine::<T, isize, N>::plotter(entry, exit)?;
        // Hold one cell behind, so the last cell falls out of the loop without needing the line's
        // length or a second pass over it.
        let Some(mut previous) = line.next() else {
            return Ok(0);
        };

        let mut updated = 0;
        for cell in line {
            updated += usize::from(self.map.mark_free_at(&previous));
            previous = cell;
        }

        // A clipped endpoint is not where the beam really stopped, so it is evidence of free space
        // only; treating it as a return would plant a phantom obstacle on the map boundary.
        updated += usize::from(match termination {
            RayTermination::Hit if endpoint_is_inside => self.map.mark_occupied_at(&previous),
            _ => self.map.mark_free_at(&previous),
        });

        Ok(updated)
    }

    /// Integrates a whole scan sharing one origin, as [`update_ray`](Self::update_ray) does for each.
    ///
    /// # Arguments
    /// * `origin`: the sensor position shared by every beam, in fractional cell coordinates.
    /// * `beams`: each beam's endpoint and termination.
    ///
    /// # Generics
    /// * `I`: any [`IntoIterator`] over endpoint and termination pairs.
    ///
    /// # Returns
    /// A [`prim@usize`], the total number of cells updated.
    ///
    /// # Errors
    /// * [`GridMapError::Ray`]: for the first beam carrying a non-finite coordinate. Beams before it
    ///   have already been applied.
    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Update Scan", skip_all, level = "debug")
    )]
    pub(crate) fn update_scan<I>(&mut self, origin: &Point<T, N>, beams: I) -> GridMapResult<usize>
    where
        I: IntoIterator<Item = (Point<T, N>, RayTermination)>,
    {
        beams
            .into_iter()
            .try_fold(0usize, |updated, (endpoint, termination)| {
                Ok(updated + self.update_ray(origin, &endpoint, termination)?)
            })
    }
}

#[cfg(test)]
mod tests {
    use super::super::GridMapConfig;
    use super::*;
    use crate::Vec;
    use nalgebra::Point2;

    fn map(dimensions: [usize; 2]) -> GridMap<f32, 2> {
        GridMap::new(dimensions, &GridMapConfig::default()).unwrap()
    }

    /// The log-odds increments the default sensor model produces.
    fn increments(grid: &GridMap<f32, 2>) -> (f32, f32) {
        (grid.occupied_delta, grid.free_delta)
    }

    #[test]
    fn test_begin_scan_advances_the_generation() {
        let mut grid = map([8, 8]);
        let first = grid.frame;

        drop(grid.begin_scan());
        assert_eq!(grid.frame, first + 1);

        drop(grid.begin_scan());
        assert_eq!(grid.frame, first + 2);
    }

    /// A scan crosses the same cell from many beams. Counting each crossing would make the map
    /// confident in proportion to beam density rather than to evidence.
    #[test]
    fn test_repeated_mark_free_in_one_scan_applies_once() {
        let mut grid = map([8, 8]);
        let (_, free) = increments(&grid);
        let cell = Point2::new(3, 3);

        let mut scan = grid.begin_scan();
        for _ in 0..10 {
            scan.mark_free(&cell).unwrap();
        }
        drop(scan);

        assert!((grid.log_odds_at(&cell).unwrap() - free).abs() < 1e-6);
    }

    #[test]
    fn test_repeated_mark_occupied_in_one_scan_applies_once() {
        let mut grid = map([8, 8]);
        let (occupied, _) = increments(&grid);
        let cell = Point2::new(3, 3);

        let mut scan = grid.begin_scan();
        for _ in 0..10 {
            scan.mark_occupied(&cell).unwrap();
        }
        drop(scan);

        assert!((grid.log_odds_at(&cell).unwrap() - occupied).abs() < 1e-6);
    }

    #[test]
    fn test_mark_free_after_occupied_in_same_scan_is_ignored() {
        let mut grid = map([8, 8]);
        let (occupied, _) = increments(&grid);
        let cell = Point2::new(2, 2);

        let mut scan = grid.begin_scan();
        scan.mark_occupied(&cell).unwrap();
        scan.mark_free(&cell).unwrap();
        drop(scan);

        assert!((grid.log_odds_at(&cell).unwrap() - occupied).abs() < 1e-6);
    }

    /// A beam passing through a cell, then another terminating in it, must leave the cell exactly
    /// as though only the return had been seen.
    #[test]
    fn test_mark_occupied_after_free_in_same_scan_nets_to_occupied_only() {
        let mut grid = map([8, 8]);
        let (occupied, _) = increments(&grid);
        let cell = Point2::new(2, 2);

        let mut scan = grid.begin_scan();
        scan.mark_free(&cell).unwrap();
        scan.mark_occupied(&cell).unwrap();
        drop(scan);

        assert!((grid.log_odds_at(&cell).unwrap() - occupied).abs() < 1e-6);
    }

    #[test]
    fn test_same_cell_in_two_scans_applies_twice() {
        let mut grid = map([8, 8]);
        let (_, free) = increments(&grid);
        let cell = Point2::new(1, 1);

        for _ in 0..2 {
            grid.begin_scan().mark_free(&cell).unwrap();
        }

        assert!((grid.log_odds_at(&cell).unwrap() - free * 2.0).abs() < 1e-6);
    }

    /// Without the sweep, a stamp left by an old generation would compare equal to the reused
    /// counter and that cell would silently stop updating.
    #[test]
    fn test_generation_wraparound_sweeps_stamps() {
        let mut grid = map([8, 8]);
        let (_, free) = increments(&grid);
        let cell = Point2::new(4, 4);

        // Stamp the cell during generation 1.
        grid.force_frame(1);
        grid.begin_scan().mark_free(&cell).unwrap();
        let after_first = grid.log_odds_at(&cell).unwrap();
        assert!(grid.stamps.iter().any(|&stamp| stamp != 0));

        // Wrap: the counter returns to 1, so every stale stamp must be cleared.
        grid.force_frame(u32::MAX >> 1);
        let mut scan = grid.begin_scan();
        scan.mark_free(&cell).unwrap();
        drop(scan);

        assert_eq!(grid.frame, 1);
        assert!(
            (grid.log_odds_at(&cell).unwrap() - (after_first + free)).abs() < 1e-6,
            "the cell must still update after the counter wraps"
        );
    }

    #[test]
    fn test_mark_out_of_bounds_reports_the_axis() {
        let mut grid = map([4, 4]);
        let mut scan = grid.begin_scan();

        assert_eq!(
            scan.mark_free(&Point2::new(4, 0)).unwrap_err(),
            GridMapError::OutOfBounds {
                axis: 0,
                index: 4,
                extent: 4
            }
        );
        assert_eq!(
            scan.mark_occupied(&Point2::new(0, -1)).unwrap_err(),
            GridMapError::OutOfBounds {
                axis: 1,
                index: -1,
                extent: 4
            }
        );
    }

    #[test]
    fn test_repeated_updates_saturate_at_the_bounds() {
        let mut grid = map([4, 4]);
        let bounds = grid.log_odds_bounds();
        let (occupied, free) = (Point2::new(1, 1), Point2::new(2, 2));

        for _ in 0..1000 {
            let mut scan = grid.begin_scan();
            scan.mark_occupied(&occupied).unwrap();
            scan.mark_free(&free).unwrap();
        }

        assert_eq!(grid.log_odds_at(&occupied), Some(*bounds.end()));
        assert_eq!(grid.log_odds_at(&free), Some(*bounds.start()));
    }

    /// The regression test for an unbounded free cell. Without a lower clamp, ten thousand free
    /// observations leave a cell at around -4000, needing thousands of contrary hits to recover —
    /// a permanently blind cell, which defeats the point of a probabilistic grid.
    #[test]
    fn test_saturated_cell_recovers_under_opposite_updates() {
        let mut grid = map([4, 4]);
        let cell = Point2::new(1, 1);

        for _ in 0..10_000 {
            grid.begin_scan().mark_free(&cell).unwrap();
        }
        assert_eq!(
            grid.log_odds_at(&cell),
            Some(*grid.log_odds_bounds().start())
        );

        let mut scans = 0;
        while grid.probability_at(&cell).unwrap() <= 0.5 {
            grid.begin_scan().mark_occupied(&cell).unwrap();
            scans += 1;
            assert!(scans < 20, "a saturated cell must recover promptly");
        }

        assert_eq!(
            scans, 5,
            "ceil(3.476 / 0.847) observations to cross one half"
        );
    }

    #[test]
    fn test_updater_map_reflects_in_progress_updates() {
        let mut grid = map([8, 8]);
        let cell = Point2::new(3, 3);

        let mut scan = grid.begin_scan();
        scan.mark_occupied(&cell).unwrap();

        assert!(
            scan.map().log_odds_at(&cell).unwrap() > 0.0,
            "a read through the guard must see the scan so far"
        );
    }

    #[test]
    fn test_update_ray_marks_free_and_endpoint_occupied() {
        let mut grid = map([16, 16]);

        let updated = grid
            .begin_scan()
            .update_ray(
                &Point2::new(0.5, 0.5),
                &Point2::new(5.5, 0.5),
                RayTermination::Hit,
            )
            .unwrap();

        assert_eq!(updated, 6);
        for x in 0..5 {
            assert!(
                grid.log_odds_at(&Point2::new(x, 0)).unwrap() < 0.0,
                "cell {x} along the beam should read as free"
            );
        }
        assert!(
            grid.log_odds_at(&Point2::new(5, 0)).unwrap() > 0.0,
            "the cell the beam terminated in should read as occupied"
        );
    }

    /// A beam that ran to its maximum range without a return is evidence of free space at its far
    /// end. Marking it occupied would plant a phantom obstacle at the sensor's range limit.
    #[test]
    fn test_max_range_beam_marks_endpoint_free() {
        let mut grid = map([16, 16]);

        grid.begin_scan()
            .update_ray(
                &Point2::new(0.5, 0.5),
                &Point2::new(5.5, 0.5),
                RayTermination::MaxRange,
            )
            .unwrap();

        assert!(
            grid.log_odds_at(&Point2::new(5, 0)).unwrap() < 0.0,
            "no obstacle may be recorded where the beam simply ran out"
        );
    }

    #[test]
    fn test_update_ray_zero_length_marks_only_the_endpoint() {
        let mut grid = map([16, 16]);
        let point = Point2::new(4.5, 4.5);

        let updated = grid
            .begin_scan()
            .update_ray(&point, &point, RayTermination::Hit)
            .unwrap();

        assert_eq!(updated, 1);
        assert!(grid.log_odds_at(&Point2::new(4, 4)).unwrap() > 0.0);
    }

    #[test]
    fn test_update_ray_clips_at_the_map_edge() {
        let mut grid = map([8, 8]);

        let updated = grid
            .begin_scan()
            .update_ray(
                &Point2::new(0.5, 0.5),
                &Point2::new(40.5, 0.5),
                RayTermination::Hit,
            )
            .unwrap();

        assert!(
            updated <= 9,
            "a clipped beam must not update more cells than the map holds along that axis"
        );
        assert!(grid.log_odds_at(&Point2::new(7, 0)).unwrap() != 0.0);
    }

    /// The endpoint of a clipped beam is not where the beam really stopped, so it carries no
    /// evidence of occupancy — the same phantom-obstacle failure as a max-range return.
    #[test]
    fn test_clipped_hit_endpoint_is_marked_free_not_occupied() {
        let mut grid = map([8, 8]);

        grid.begin_scan()
            .update_ray(
                &Point2::new(0.5, 4.5),
                &Point2::new(40.5, 4.5),
                RayTermination::Hit,
            )
            .unwrap();

        for x in 0..8 {
            assert!(
                grid.log_odds_at(&Point2::new(x, 4)).unwrap() < 0.0,
                "cell {x} is on the clipped run and must read free, obstacle-free"
            );
        }
    }

    #[test]
    fn test_update_ray_entirely_outside_returns_ok_zero() {
        let mut grid = map([8, 8]);

        let updated = grid
            .begin_scan()
            .update_ray(
                &Point2::new(-50.0, -50.0),
                &Point2::new(-40.0, -40.0),
                RayTermination::Hit,
            )
            .unwrap();

        assert_eq!(updated, 0);
        assert!(grid.iter_log_odds().all(|odds| odds == 0.0));
    }

    #[test]
    fn test_update_ray_rejects_non_finite_coordinates() {
        let mut grid = map([8, 8]);
        let mut scan = grid.begin_scan();
        let good = Point2::new(1.0, 1.0);

        for bad in [
            Point2::new(f32::NAN, 1.0),
            Point2::new(1.0, f32::NAN),
            Point2::new(f32::INFINITY, 1.0),
            Point2::new(1.0, f32::NEG_INFINITY),
        ] {
            assert_eq!(
                scan.update_ray(&good, &bad, RayTermination::Hit)
                    .unwrap_err(),
                GridMapError::Ray(BresenhamError::NonFiniteCoordinate),
                "endpoint {bad:?}"
            );
            assert_eq!(
                scan.update_ray(&bad, &good, RayTermination::Hit)
                    .unwrap_err(),
                GridMapError::Ray(BresenhamError::NonFiniteCoordinate),
                "origin {bad:?}"
            );
        }

        drop(scan);
        assert!(
            grid.iter_log_odds().all(|odds| odds == 0.0),
            "a rejected beam must not have written anything, least of all to the origin cell"
        );
    }

    /// A step count is derived from a beam's length, and a merely absurd endpoint is never
    /// rejected. Without clipping the segment first, this beam would walk a billion cells to throw
    /// every one of them away.
    #[test]
    fn test_far_endpoint_does_not_walk_unbounded() {
        let mut grid = map([16, 16]);

        let updated = grid
            .begin_scan()
            .update_ray(
                &Point2::new(8.5, 8.5),
                &Point2::new(1.0e9, 8.5),
                RayTermination::Hit,
            )
            .unwrap();

        assert!(
            updated <= 32,
            "the walk must be bounded by the map, not by the caller's endpoint: {updated}"
        );
    }

    #[test]
    fn test_update_scan_sums_updated_cells() {
        let mut grid = map([16, 16]);
        let beams = Vec::from([
            (Point2::new(5.5, 0.5), RayTermination::Hit),
            (Point2::new(0.5, 5.5), RayTermination::Hit),
        ]);

        let updated = grid
            .begin_scan()
            .update_scan(&Point2::new(0.5, 0.5), beams)
            .unwrap();

        // Six cells each, less the shared origin cell, which is deduplicated across the scan.
        assert_eq!(updated, 11);
    }

    #[test]
    fn test_update_scan_dedups_overlapping_beams() {
        let mut grid = map([16, 16]);
        let (_, free) = increments(&grid);
        let origin = Point2::new(0.5, 0.5);

        // Two collinear beams of different lengths, so the shorter one's cells are all revisited.
        let beams = Vec::from([
            (Point2::new(4.5, 0.5), RayTermination::MaxRange),
            (Point2::new(8.5, 0.5), RayTermination::MaxRange),
        ]);
        grid.begin_scan().update_scan(&origin, beams).unwrap();

        for x in 0..4 {
            assert!(
                (grid.log_odds_at(&Point2::new(x, 0)).unwrap() - free).abs() < 1e-6,
                "cell {x} was crossed twice but must be counted once"
            );
        }
    }

    #[test]
    fn test_update_scan_reports_the_first_non_finite_beam() {
        let mut grid = map([16, 16]);
        let beams = Vec::from([
            (Point2::new(5.5, 0.5), RayTermination::Hit),
            (Point2::new(f32::NAN, 0.5), RayTermination::Hit),
            (Point2::new(0.5, 5.5), RayTermination::Hit),
        ]);

        assert_eq!(
            grid.begin_scan()
                .update_scan(&Point2::new(0.5, 0.5), beams)
                .unwrap_err(),
            GridMapError::Ray(BresenhamError::NonFiniteCoordinate)
        );
        assert!(
            grid.log_odds_at(&Point2::new(5, 0)).unwrap() > 0.0,
            "beams preceding the failure have already been applied"
        );
    }

    #[test]
    fn test_clipped_beam_still_covers_the_last_cell_in_the_map() {
        let mut grid = map([8, 8]);

        grid.begin_scan()
            .update_ray(
                &Point2::new(0.5, 2.5),
                &Point2::new(99.5, 2.5),
                RayTermination::MaxRange,
            )
            .unwrap();

        // Every cell of the row, including the last, must have been visited: pulling the clipped
        // exit back into cell 7 is what stops the plotter's endpoint substitution from skipping it.
        for x in 0..8 {
            assert!(
                grid.log_odds_at(&Point2::new(x, 2)).unwrap() < 0.0,
                "cell {x} of the clipped row was never visited"
            );
        }
    }

    /// An end-to-end pass: sweep a synthetic wall, then confirm the map reads the way a scan
    /// matcher would need it to.
    #[test]
    fn test_synthetic_scan_builds_a_readable_wall() {
        let mut grid = GridMap::<f32, 2>::new([256, 256], &GridMapConfig::default()).unwrap();
        let origin = Point2::new(128.5, 128.5);
        let wall_x = 200.5;

        // Ten sweeps of a flat wall, so the cells there saturate well clear of one half.
        for _ in 0..10 {
            let beams = (118..=138)
                .map(|y| (Point2::new(wall_x, y as f32 + 0.5), RayTermination::Hit))
                .collect::<Vec<_>>();
            grid.begin_scan().update_scan(&origin, beams).unwrap();
        }

        // The wall reads as occupied.
        assert!(
            grid.probability_at(&Point2::new(200, 128)).unwrap() > 0.9,
            "the wall should be confidently occupied"
        );
        // The space the beams crossed reads as free.
        assert!(
            grid.probability_at(&Point2::new(160, 128)).unwrap() < 0.1,
            "the swept space should be confidently free"
        );
        // Everything the sensor never saw is still unknown, including behind the wall.
        assert_eq!(grid.probability_at(&Point2::new(230, 128)), Some(0.5));
        assert_eq!(grid.probability_at(&Point2::new(60, 60)), Some(0.5));

        // Approaching the wall from the free side, the log-odds field must rise toward it, which
        // is the signal a Gauss-Newton scan matcher follows.
        let sample = grid.sample_log_odds(&Point2::new(199.5, 128.5)).unwrap();
        assert!(
            sample.gradient[0] > 0.0,
            "the gradient must point toward the wall, not away from it"
        );
        assert!(
            sample.gradient[1].abs() < sample.gradient[0],
            "a wall parallel to y should produce almost no gradient along it"
        );

        // The probability field agrees in direction, being a monotone map of the log-odds one.
        let probability = grid.sample_probability(&Point2::new(199.5, 128.5)).unwrap();
        assert!(probability.gradient[0] > 0.0);
        assert!(probability.value > 0.0 && probability.value < 1.0);
    }
}
