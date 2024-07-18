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

#![deny(missing_docs)]
#![deny(rustdoc::missing_crate_level_docs)]
#![deny(rustdoc::broken_intra_doc_links)]
#![deny(rustdoc::private_intra_doc_links)]
#![cfg_attr(not(feature = "std"), no_std)]
#![doc = include_str!("../README.md")]

#[cfg(not(feature = "std"))]
extern crate alloc;
#[cfg(not(feature = "std"))]
extern crate core;

#[cfg(feature = "std")]
use std::{
    array,
    boxed::Box,
    cmp::Ordering,
    collections::{HashMap, VecDeque},
    fmt::Debug,
    iter::Sum,
    marker, ops,
    vec::Vec,
};

#[cfg(not(feature = "std"))]
use {
    alloc::{
        boxed::Box,
        collections::{BTreeMap as HashMap, VecDeque},
        vec::Vec,
    },
    core::{array, cmp::Ordering, fmt::Debug, iter::Sum, marker, ops},
};

///A module containing common and interfacing structs and types.
pub mod types;

/// A K-Dimensional Tree data structure, useful for various geo-spatial computations.
pub mod kd_tree;

/// A module containing various algorithms for convex hulls.
pub mod convex_hulls;

/// A module containing various algorithms for point clouds.
pub mod point_clouds;

/// A module containing various algorithms for polygons.
pub mod polygons;

/// A module containing various geospatial algorithms.
pub mod geo;

/// A module containing various line algorithms.
pub mod lines;

/// Various utility functions that are commonly used by these algorithms.
pub mod utils;
