mod haversine;

pub use haversine::{calculate_haversine_distance, calculate_sphere_bearing};

#[cfg(feature = "pregenerated")]
#[doc = "Contains pregenerated functions for single precision geographical algorithms."]
pub mod single_precision {
    pub use super::haversine::single_precision::{
        calculate_haversine_distance, calculate_sphere_bearing,
    };
}

#[cfg(feature = "pregenerated")]
#[doc = "Contains pregenerated functions for double precision geographical algorithms."]
pub mod double_precision {
    pub use super::haversine::double_precision::{
        calculate_haversine_distance, calculate_sphere_bearing,
    };
}
