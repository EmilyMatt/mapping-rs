use super::{grid_map::GridMap, HectorMapper};
use crate::PhantomData;
use nalgebra::ComplexField;

pub(super) mod odometry {
    pub(crate) struct WantsOdometryConfig;
    pub(crate) struct HasOdometryConfig;
}

pub(super) mod dimensions {
    pub(crate) struct WantsDimensions;
    pub(crate) struct HasDimensions;
}

pub(super) mod resolution {
    pub(crate) struct WantsResolution;
    pub(crate) struct HasResolution;
}

pub(super) struct HectorMapperBuilder<
    T,
    const N: usize,
    MaybeCalculatesOdometry,
    MaybeHasDimensions,
    MaybeHasResolution,
> {
    pub(super) _internal: HectorMapper<T, N>,
    pub(super) _calculates_odometry: PhantomData<MaybeCalculatesOdometry>,
    pub(super) _has_dimensions: PhantomData<MaybeHasDimensions>,
    pub(super) _has_resolution: PhantomData<MaybeHasResolution>,
}

impl<T, const N: usize>
    HectorMapperBuilder<
        T,
        N,
        odometry::WantsOdometryConfig,
        dimensions::WantsDimensions,
        resolution::WantsResolution,
    >
where
    T: ComplexField,
{
    pub fn default() -> Self {
        Self {
            _internal: HectorMapper {
                with_odometry: false,
                grid_map: GridMap::create([0; N]),
                resolution: T::zero(),
            },
            _calculates_odometry: Default::default(),
            _has_dimensions: Default::default(),
            _has_resolution: Default::default(),
        }
    }
}

impl<T, const N: usize, MaybeHasDimensions, MaybeHasResolution>
    HectorMapperBuilder<T, N, odometry::WantsOdometryConfig, MaybeHasDimensions, MaybeHasResolution>
{
    pub fn with_odometry_calculation(
        self,
        calculate_odometry: bool,
    ) -> HectorMapperBuilder<
        T,
        N,
        odometry::HasOdometryConfig,
        MaybeHasDimensions,
        MaybeHasResolution,
    > {
        HectorMapperBuilder {
            _internal: HectorMapper {
                with_odometry: calculate_odometry,
                ..self._internal
            },
            _calculates_odometry: Default::default(),
            _has_dimensions: Default::default(),
            _has_resolution: Default::default(),
        }
    }
}

impl<T, const N: usize, MaybeCalculatesOdometry, MaybeHasResolution>
    HectorMapperBuilder<
        T,
        N,
        MaybeCalculatesOdometry,
        dimensions::WantsDimensions,
        MaybeHasResolution,
    >
{
    pub fn with_dimensions(
        self,
        dimensions: [usize; N],
    ) -> HectorMapperBuilder<
        T,
        N,
        MaybeCalculatesOdometry,
        dimensions::HasDimensions,
        MaybeHasResolution,
    > {
        HectorMapperBuilder {
            _internal: HectorMapper {
                grid_map: GridMap::create(dimensions),
                ..self._internal
            },
            _calculates_odometry: Default::default(),
            _has_dimensions: Default::default(),
            _has_resolution: Default::default(),
        }
    }
}

impl<T, const N: usize, MaybeCalculatesOdometry, MaybeHasDimensions>
    HectorMapperBuilder<
        T,
        N,
        MaybeCalculatesOdometry,
        MaybeHasDimensions,
        resolution::WantsResolution,
    >
{
    pub fn with_resolution(
        self,
        resolution: T,
    ) -> HectorMapperBuilder<
        T,
        N,
        MaybeCalculatesOdometry,
        MaybeHasDimensions,
        resolution::HasResolution,
    > {
        HectorMapperBuilder {
            _internal: HectorMapper {
                resolution,
                ..self._internal
            },
            _calculates_odometry: Default::default(),
            _has_dimensions: Default::default(),
            _has_resolution: Default::default(),
        }
    }
}

impl<T, const N: usize>
    HectorMapperBuilder<
        T,
        N,
        odometry::HasOdometryConfig,
        dimensions::HasDimensions,
        resolution::HasResolution,
    >
{
    pub fn build(self) -> HectorMapper<T, N> {
        self._internal
    }
}
