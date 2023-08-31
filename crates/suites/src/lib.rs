#![deny(missing_docs)]
#![deny(rustdoc::missing_crate_level_docs)]
#![deny(rustdoc::broken_intra_doc_links)]
#![deny(rustdoc::private_intra_doc_links)]
#![cfg_attr(not(feature = "std"), no_std)]

//! A collection of mapping suites that implement the mapping-algorithms-rs crate.

#![cfg(not(feature = "std"))]
extern crate alloc;

/// A Hector Mapper, Utilizing Lidar And Wheel Odometry
pub mod hector_mapper;
