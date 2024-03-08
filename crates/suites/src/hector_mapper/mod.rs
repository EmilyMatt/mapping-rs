use crate::Sum;
use builder::HectorMapperBuilder;
use grid_map::GridMap;
use mapping_algorithms_rs::icp::types::ICPSuccess;
use mapping_algorithms_rs::icp::{icp, types::ICPConfiguration};
use mapping_algorithms_rs::types::IsometryAbstraction;
use nalgebra::{ComplexField, Const, RealField};
use num_traits::{AsPrimitive, Bounded};

mod builder;
mod grid_map;

pub struct HectorMapper<T, const N: usize>
where
    T: ComplexField,
{
    grid_map: GridMap<T, N>,
    resolution: T,
    with_odometry: bool,

    current_position: nalgebra::Point<T, N>,
    current_angle: T,
    last_point_cloud: Vec<nalgebra::Point<T, N>>,
}

impl<T, const N: usize> HectorMapper<T, N>
where
    T: Bounded + Copy + Default + RealField + Sum,
    f32: AsPrimitive<T>,
    usize: AsPrimitive<T>,
    Const<N>: IsometryAbstraction<T, N>,
{
    pub fn builder() -> HectorMapperBuilder<
        T,
        N,
        builder::odometry::WantsOdometryConfig,
        builder::dimensions::WantsDimensions,
        builder::resolution::WantsResolution,
    > {
        HectorMapperBuilder::default()
    }

    ///
    pub fn push_point_cloud(&mut self, point_cloud: Vec<nalgebra::Point<T, N>>) {
        if self.with_odometry {
            let res: Result<ICPSuccess<T, N, Const<N>>, &'static str> = icp(
                &self.last_point_cloud,
                &point_cloud,
                ICPConfiguration::builder()
                    .with_kd_tree(true)
                    .with_max_iterations(20)
                    .with_mse_interval_threshold(0.001.as_())
                    .build(),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::hector_mapper::HectorMapper;

    #[test]
    fn test_something() {
        let hector_mapper = HectorMapper::builder()
            .with_resolution(0.1)
            .with_odometry_calculation(true)
            .with_dimensions([1024; 3]);
    }
}
