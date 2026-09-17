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

use mapping_algorithms::lines::BresenhamError;
use nalgebra::{Point, RealField, SVector, Scalar};
use num_traits::{AsPrimitive, ConstOne, ConstZero};

use crate::fmt;

/// The errors that can arise while building or updating an occupancy grid.
///
/// Also derives [`thiserror::Error`] under the `std` feature.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "std", derive(thiserror::Error))]
pub enum GridMapError {
    /// An axis was shorter than the two cells an interpolation stencil requires.
    DimensionTooSmall {
        /// The offending axis.
        axis: usize,
        /// The requested extent of that axis.
        extent: usize,
    },
    /// The product of the requested dimensions does not fit in a [`prim@usize`].
    CapacityOverflow,
    /// The allocator could not provide storage for the requested number of cells.
    AllocationFailed {
        /// The number of cells whose allocation failed.
        cells: usize,
    },
    /// A cell index fell outside the map. Axes are checked in ascending order, and only the first
    /// failing one is reported.
    OutOfBounds {
        /// The offending axis.
        axis: usize,
        /// The offending index.
        index: isize,
        /// The exclusive upper bound of that axis.
        extent: usize,
    },
    /// A coordinate was NaN or infinite.
    NonFiniteCoordinate {
        /// The offending axis.
        axis: usize,
    },
    /// The occupied probability was not strictly between `0.5` and `1.0`.
    InvalidOccupiedProbability,
    /// The free probability was not strictly between `0.0` and `0.5`.
    InvalidFreeProbability,
    /// The maximum confidence was not strictly between `0.5` and `1.0`.
    InvalidMaxConfidence,
    /// A beam could not be plotted.
    Ray(BresenhamError),
}

impl fmt::Display for GridMapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DimensionTooSmall { axis, extent } => write!(
                f,
                "axis {axis} has an extent of {extent}, but at least 2 cells are required"
            ),
            Self::CapacityOverflow => {
                write!(f, "the requested dimensions overflow a usize")
            }
            Self::AllocationFailed { cells } => {
                write!(f, "could not allocate storage for {cells} cells")
            }
            Self::OutOfBounds {
                axis,
                index,
                extent,
            } => write!(
                f,
                "index {index} on axis {axis} is outside the extent 0..{extent}"
            ),
            Self::NonFiniteCoordinate { axis } => {
                write!(f, "the coordinate on axis {axis} is not finite")
            }
            Self::InvalidOccupiedProbability => {
                write!(f, "the occupied probability must be between 0.5 and 1.0")
            }
            Self::InvalidFreeProbability => {
                write!(f, "the free probability must be between 0.0 and 0.5")
            }
            Self::InvalidMaxConfidence => {
                write!(f, "the maximum confidence must be between 0.5 and 1.0")
            }
            Self::Ray(err) => write!(f, "the beam could not be plotted: {err}"),
        }
    }
}

impl From<BresenhamError> for GridMapError {
    fn from(value: BresenhamError) -> Self {
        Self::Ray(value)
    }
}

/// The result of a fallible grid map operation, carrying a [`GridMapError`] on failure.
pub type GridMapResult<T> = Result<T, GridMapError>;

/// Why a beam stopped where it did.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) enum RayTermination {
    /// The sensor received a return, so the final cell is evidence of occupancy.
    Hit,
    /// The beam reached its maximum range, so the final cell is evidence of free space.
    MaxRange,
}

/// One range measurement: where the beam started, where it stopped, and why.
///
/// # Generics
/// * `T`: Either an [`prim@f32`] or [`prim@f64`]. Bounded by [`Scalar`], as [`Point`] itself is.
/// * `N`: a usize, representing the number of dimensions.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct ScanBeam<T: Scalar, const N: usize> {
    /// The sensor position, in fractional cell coordinates.
    pub origin: Point<T, N>,
    /// Where the beam stopped, in fractional cell coordinates.
    pub endpoint: Point<T, N>,
    /// Whether the beam stopped on a return or at its maximum range.
    pub termination: RayTermination,
}

/// An interpolated value and its analytic spatial gradient, both from one `2^N`-corner gather.
///
/// # Generics
/// * `T`: Either an [`prim@f32`] or [`prim@f64`].
/// * `N`: a usize, representing the number of dimensions.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct MapSample<T, const N: usize> {
    /// The interpolated value; either log-odds, or a probability in the range `0.0..=1.0`.
    pub value: T,
    /// The gradient of [`value`](Self::value), in value per *cell*, not per metre.
    pub gradient: SVector<T, N>,
}

impl<T: ConstOne + ConstZero + Copy + RealField, const N: usize> MapSample<T, N> {
    /// The log-odds sample of an unobserved region.
    ///
    /// # Returns
    /// A [`MapSample`] whose value and gradient are both zero.
    pub(crate) fn unknown_log_odds() -> Self {
        Self {
            value: T::ZERO,
            gradient: SVector::zeros(),
        }
    }

    /// The probability sample of an unobserved region.
    ///
    /// # Returns
    /// A [`MapSample`] carrying a value of `0.5` and a zero gradient.
    pub(crate) fn unknown_probability() -> Self {
        Self {
            value: T::ONE / (T::ONE + T::ONE),
            gradient: SVector::zeros(),
        }
    }
}

/// The inverse sensor model and saturation behaviour of a grid map.
///
/// The values are validated when the map is built, not by the builder.
///
/// # Generics
/// * `T`: Either an [`prim@f32`] or [`prim@f64`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GridMapConfig<T> {
    /// `p(occupied | beam terminated here)`, strictly between `0.5` and `1.0`.
    pub(crate) occupied_probability: T,
    /// `p(occupied | beam passed through here)`, strictly between `0.0` and `0.5`.
    pub(crate) free_probability: T,
    /// The confidence at which a cell saturates, strictly between `0.5` and `1.0`. Saturating
    /// keeps the map able to revise a long-held belief.
    pub(crate) max_confidence: T,
}

impl<T: 'static + Copy> Default for GridMapConfig<T>
where
    f32: AsPrimitive<T>,
{
    /// An occupied probability of `0.7`, a free probability of `0.4`, and saturation at `0.97`.
    fn default() -> Self {
        Self {
            occupied_probability: 0.7.as_(),
            free_probability: 0.4.as_(),
            max_confidence: 0.97.as_(),
        }
    }
}

impl<T: 'static + Copy> GridMapConfig<T>
where
    f32: AsPrimitive<T>,
{
    /// Returns a builder for the configuration struct.
    ///
    /// # Returns
    /// A [`GridMapConfigBuilder`], pre-populated with the defaults.
    pub(crate) fn builder() -> GridMapConfigBuilder<T> {
        GridMapConfigBuilder {
            _internal: Self::default(),
        }
    }
}

impl<T: ConstOne + ConstZero + Copy + RealField> GridMapConfig<T> {
    /// Converts the sensor model into the log-odds increments a map stores.
    ///
    /// Taking an uninformative prior, each Bayesian update reduces to adding the inverse sensor
    /// model's logit, so both increments are additions and the free one is negative.
    ///
    /// # Returns
    /// The occupied increment, the free increment, and the saturation bound.
    ///
    /// # Errors
    /// * [`GridMapError::InvalidOccupiedProbability`], [`GridMapError::InvalidFreeProbability`] or
    ///   [`GridMapError::InvalidMaxConfidence`]: if that value lies outside its open interval.
    ///   The intervals are open because zero and one are infinite in log-odds.
    pub(crate) fn resolve(&self) -> GridMapResult<(T, T, T)> {
        let half = T::ONE / (T::ONE + T::ONE);

        if !(self.occupied_probability > half && self.occupied_probability < T::ONE) {
            return Err(GridMapError::InvalidOccupiedProbability);
        }

        if !(self.free_probability > T::ZERO && self.free_probability < half) {
            return Err(GridMapError::InvalidFreeProbability);
        }

        if !(self.max_confidence > half && self.max_confidence < T::ONE) {
            return Err(GridMapError::InvalidMaxConfidence);
        }

        Ok((
            logit(self.occupied_probability),
            logit(self.free_probability),
            logit(self.max_confidence),
        ))
    }
}

/// The log-odds of a probability, `ln(p / (1 - p))`.
///
/// # Arguments
/// * `probability`: a value strictly between zero and one; either endpoint gives an infinity.
///
/// # Generics
/// * `T`: Either an [`prim@f32`] or [`prim@f64`].
///
/// # Returns
/// The log-odds, positive above one half and negative below it.
#[inline]
pub(crate) fn logit<T: ConstOne + Copy + RealField>(probability: T) -> T {
    nalgebra::ComplexField::ln(probability / (T::ONE - probability))
}

/// A builder for [`GridMapConfig`].
///
/// # Generics
/// * `T`: Either an [`prim@f32`] or [`prim@f64`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GridMapConfigBuilder<T> {
    _internal: GridMapConfig<T>,
}

impl<T: Copy> GridMapConfigBuilder<T> {
    /// The probability that a cell is occupied, given that a beam terminated inside it.
    ///
    /// # Arguments
    /// * `occupied_probability`: strictly between `0.5` and `1.0`.
    ///
    /// # Returns
    /// A copy of self, with that value replaced.
    pub(crate) fn with_occupied_probability(&self, occupied_probability: T) -> Self {
        Self {
            _internal: GridMapConfig {
                occupied_probability,
                ..self._internal
            },
        }
    }

    /// The probability that a cell is occupied, given that a beam passed through it.
    ///
    /// # Arguments
    /// * `free_probability`: strictly between `0.0` and `0.5`.
    ///
    /// # Returns
    /// A copy of self, with that value replaced.
    pub(crate) fn with_free_probability(&self, free_probability: T) -> Self {
        Self {
            _internal: GridMapConfig {
                free_probability,
                ..self._internal
            },
        }
    }

    /// The confidence at which a cell saturates.
    ///
    /// # Arguments
    /// * `max_confidence`: strictly between `0.5` and `1.0`.
    ///
    /// # Returns
    /// A copy of self, with that value replaced.
    pub(crate) fn with_max_confidence(&self, max_confidence: T) -> Self {
        Self {
            _internal: GridMapConfig {
                max_confidence,
                ..self._internal
            },
        }
    }

    /// Generates a [`GridMapConfig`] from the builder's current contents.
    ///
    /// # Returns
    /// A [`GridMapConfig`]. Takes `&self`, so the builder stays intact for another use.
    pub(crate) fn build(&self) -> GridMapConfig<T> {
        self._internal
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_default_matches_builder_defaults() {
        assert_eq!(
            GridMapConfig::<f32>::builder().build(),
            GridMapConfig::<f32>::default()
        );
    }

    #[test]
    fn test_config_builder_setters() {
        let config = GridMapConfig::<f32>::builder()
            .with_occupied_probability(0.8)
            .with_free_probability(0.3)
            .with_max_confidence(0.99)
            .build();

        assert_eq!(config.occupied_probability, 0.8);
        assert_eq!(config.free_probability, 0.3);
        assert_eq!(config.max_confidence, 0.99);
    }

    #[test]
    fn test_config_builder_does_not_consume_self() {
        let builder = GridMapConfig::<f32>::builder().with_occupied_probability(0.8);

        // `build` takes &self, so a builder may be reused; both results must agree.
        assert_eq!(builder.build(), builder.build());
        assert_eq!(builder.build().occupied_probability, 0.8);
    }

    #[test]
    fn test_logit_derivation() {
        // ln(0.7 / 0.3), the increment a single occupied observation contributes.
        assert!((logit(0.7f64) - 0.8472978603872034_f64).abs() < 1e-12);
        // ln(0.4 / 0.6); negative, so the free update is an addition like the occupied one.
        assert!((logit(0.4f64) - -0.4054651081081643_f64).abs() < 1e-12);
        // A probability of one half carries no information at all.
        assert_eq!(logit(0.5f64), 0.0);
    }

    #[test]
    fn test_resolve_produces_expected_increments() {
        let (occupied, free, max) = GridMapConfig::<f64>::builder()
            .with_occupied_probability(0.7)
            .with_free_probability(0.4)
            .with_max_confidence(0.97)
            .build()
            .resolve()
            .unwrap();

        assert!(
            occupied > 0.0,
            "an occupied observation must raise the odds"
        );
        assert!(free < 0.0, "a free observation must lower the odds");
        assert!((occupied - 0.8472978603872034).abs() < 1e-12);
        assert!((free - -0.4054651081081643).abs() < 1e-12);
        assert!((max - 3.4760986898352724).abs() < 1e-12);
    }

    /// The defaults are written as [`prim@f32`] literals and converted, matching
    /// [`ICPConfiguration`](mapping_algorithms::point_clouds::ICPConfiguration), so a
    /// [`prim@f64`] map's defaults carry single-precision rounding. That is a deliberate
    /// consequence of the convention rather than a bug, but it should not drift unnoticed.
    #[test]
    fn test_defaults_carry_single_precision_rounding() {
        let (occupied, _, _) = GridMapConfig::<f64>::default().resolve().unwrap();

        assert!((occupied - 0.8472978603872034).abs() < 1e-6);
        assert!((occupied - 0.8472978603872034).abs() > 1e-12);
        assert_eq!(
            GridMapConfig::<f32>::default().occupied_probability,
            0.7f32,
            "a single-precision map should see its defaults exactly"
        );
    }

    /// The prior implementation derived its increments as `(p - 1/p).ln()`, which is `ln` of a
    /// negative number for any sensible free probability, and so silently NaN. Sweep the whole
    /// valid domain and insist every derived quantity is finite.
    #[test]
    fn test_no_config_in_valid_range_produces_nan() {
        for occupied_step in 1..100 {
            let occupied_probability = 0.5 + f64::from(occupied_step) * 0.005;
            for free_step in 1..100 {
                let free_probability = f64::from(free_step) * 0.005;

                let config = GridMapConfig::<f64>::builder()
                    .with_occupied_probability(occupied_probability)
                    .with_free_probability(free_probability)
                    .build();

                let (occupied, free, max) = config.resolve().unwrap();
                assert!(
                    occupied.is_finite() && free.is_finite() && max.is_finite(),
                    "non-finite increment for p_occ={occupied_probability}, p_free={free_probability}"
                );
                assert!(occupied > 0.0);
                assert!(free < 0.0);
            }
        }
    }

    #[test]
    fn test_resolve_rejects_occupied_probability_out_of_range() {
        for probability in [0.5, 0.4, 1.0, 0.0, 1.5, -0.1, f64::NAN] {
            let config = GridMapConfig::<f64>::builder()
                .with_occupied_probability(probability)
                .build();
            assert_eq!(
                config.resolve().unwrap_err(),
                GridMapError::InvalidOccupiedProbability,
                "accepted an occupied probability of {probability}"
            );
        }
    }

    #[test]
    fn test_resolve_rejects_free_probability_out_of_range() {
        for probability in [0.5, 0.6, 0.0, 1.0, -0.1, f64::NAN] {
            let config = GridMapConfig::<f64>::builder()
                .with_free_probability(probability)
                .build();
            assert_eq!(
                config.resolve().unwrap_err(),
                GridMapError::InvalidFreeProbability,
                "accepted a free probability of {probability}"
            );
        }
    }

    #[test]
    fn test_resolve_rejects_max_confidence_out_of_range() {
        for confidence in [0.5, 0.4, 1.0, 0.0, 1.5, f64::NAN] {
            let config = GridMapConfig::<f64>::builder()
                .with_max_confidence(confidence)
                .build();
            assert_eq!(
                config.resolve().unwrap_err(),
                GridMapError::InvalidMaxConfidence,
                "accepted a maximum confidence of {confidence}"
            );
        }
    }

    #[test]
    fn test_unknown_samples() {
        let log_odds = MapSample::<f32, 2>::unknown_log_odds();
        assert_eq!(log_odds.value, 0.0);
        assert_eq!(log_odds.gradient, SVector::<f32, 2>::zeros());

        let probability = MapSample::<f32, 3>::unknown_probability();
        assert_eq!(probability.value, 0.5);
        assert_eq!(probability.gradient, SVector::<f32, 3>::zeros());
    }

    #[test]
    fn test_bresenham_error_converts() {
        let converted: GridMapError = BresenhamError::NonFiniteCoordinate.into();
        assert_eq!(
            converted,
            GridMapError::Ray(BresenhamError::NonFiniteCoordinate)
        );
    }

    #[cfg(feature = "std")]
    #[test]
    fn test_error_display_for_each_variant() {
        fn assert_error<E: std::error::Error>(error: E) -> String {
            error.to_string()
        }

        // Every variant must render; these are the arms coverage would otherwise never reach.
        assert_eq!(
            assert_error(GridMapError::DimensionTooSmall { axis: 1, extent: 1 }),
            "axis 1 has an extent of 1, but at least 2 cells are required"
        );
        assert_eq!(
            assert_error(GridMapError::CapacityOverflow),
            "the requested dimensions overflow a usize"
        );
        assert_eq!(
            assert_error(GridMapError::AllocationFailed { cells: 7 }),
            "could not allocate storage for 7 cells"
        );
        assert_eq!(
            assert_error(GridMapError::OutOfBounds {
                axis: 0,
                index: -1,
                extent: 512
            }),
            "index -1 on axis 0 is outside the extent 0..512"
        );
        assert_eq!(
            assert_error(GridMapError::NonFiniteCoordinate { axis: 2 }),
            "the coordinate on axis 2 is not finite"
        );
        assert_eq!(
            assert_error(GridMapError::InvalidOccupiedProbability),
            "the occupied probability must be between 0.5 and 1.0"
        );
        assert_eq!(
            assert_error(GridMapError::InvalidFreeProbability),
            "the free probability must be between 0.0 and 0.5"
        );
        assert_eq!(
            assert_error(GridMapError::InvalidMaxConfidence),
            "the maximum confidence must be between 0.5 and 1.0"
        );
        assert!(
            assert_error(GridMapError::Ray(BresenhamError::ZeroDimensions))
                .starts_with("the beam could not be plotted:")
        );
    }
}
