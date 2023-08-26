#![deny(missing_docs)]
#![deny(rustdoc::missing_crate_level_docs)]
#![deny(rustdoc::broken_intra_doc_links)]
#![deny(rustdoc::private_intra_doc_links)]

//! A collection of mapping algorithms, for use either independently, or using the mapping-suites-rs crate.

#[doc = "An Iterative Closest Point algorithm, useful in matching Point Clouds."]
#[doc = "Contains a 2D implementation when using the `2d` feature, and a 3D implementation when using the `3d` feature."]
pub mod icp;

#[doc = "A module containing common and interfacing structs and types."]
pub mod types;

#[doc = "Various utility functions that are commonly used by these algorithms."]
pub mod utils;
