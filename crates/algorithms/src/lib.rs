#![deny(missing_docs)]
#![deny(rustdoc::missing_crate_level_docs)]
#![deny(rustdoc::broken_intra_doc_links)]
#![deny(rustdoc::private_intra_doc_links)]
#![cfg_attr(not(feature = "std"), no_std)]

//! A collection of mapping algorithms, for use either independently, or using the mapping-suites-rs crate.

#[cfg(not(any(feature = "std", feature = "libm")))]
compile_error!("Either `std` or `libm` features must be enabled");

#[cfg(not(feature = "std"))]
extern crate alloc;
#[cfg(not(feature = "std"))]
extern crate core;

#[cfg(not(feature = "std"))]
use {
    alloc::{boxed::Box, string::String, vec::Vec},
    core::{array, fmt::Debug, iter::Sum, mem, ops},
    utils::distance_squared,
};
#[cfg(feature = "std")]
use {
    nalgebra::distance_squared,
    std::{array, boxed::Box, fmt::Debug, iter::Sum, mem, ops, string::String, vec::Vec},
};

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

/// A K-Dimensional Tree data structure, useful for various geo-spatial computations.
pub mod kd_tree;

/// A Collection of pathfinding algorithms
mod pathfinding;
/// Various utility functions that are commonly used by these algorithms.
pub mod utils;
