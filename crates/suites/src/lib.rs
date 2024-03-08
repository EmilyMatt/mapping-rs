#![deny(missing_docs)]
#![deny(rustdoc::missing_crate_level_docs)]
#![deny(rustdoc::broken_intra_doc_links)]
#![deny(rustdoc::private_intra_doc_links)]
#![cfg_attr(not(feature = "std"), no_std)]
//! A collection of mapping algorithms, for use either independently, or using the mapping-suites-rs crate.

#[cfg(not(feature = "std"))]
extern crate alloc;
#[cfg(not(feature = "std"))]
extern crate core;

#[cfg(feature = "std")]
use std::{marker::PhantomData, vec::Vec};
#[cfg(not(feature = "std"))]
use {alloc::vec::Vec, core::marker::PhantomData};

/// A SLAM suite that uses LIDAR point clouds to determine the vehicle's velocity and map its surroundings.
pub mod hector_mapper;
