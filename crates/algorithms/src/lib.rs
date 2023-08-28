#![deny(missing_docs)]
#![deny(rustdoc::missing_crate_level_docs)]
#![deny(rustdoc::broken_intra_doc_links)]
#![deny(rustdoc::private_intra_doc_links)]

//! A collection of mapping algorithms, for use either independently, or using the mapping-suites-rs crate.

/// An Iterative Closest Point algorithm, useful in matching Point Clouds.
/// Contains a 2D implementation when using the `2d` feature, and a 3D implementation when using the `3d` feature.
pub mod icp;

///A module containing common and interfacing structs and types.
pub mod types;

/// Implementation of an Ackerman vehicle steering model.
pub mod ackerman;

/// Implementations of Bresenham line algorithms.
pub mod bresenham;

/// Implementations of a Point-In-Polygon algorithm for both the singular and plural cases.
pub mod point_in_polygon;

/// Implementation of a Point-In-Convex-Hull algorithm, for both the singular and plural cases.
pub mod point_in_convex_hull;

/// Various utility functions that are commonly used by these algorithms.
pub mod utils;
