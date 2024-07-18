[![Build Pipeline](https://github.com/EmilyMatt/mapping-rs/actions/workflows/main.yml/badge.svg)](https://github.com/EmilyMatt/mapping-rs/actions/workflows/main.yml)
[![codecov](https://codecov.io/gh/EmilyMatt/mapping-rs/graph/badge.svg)](https://codecov.io/gh/EmilyMatt/mapping-rs)
[![GitHub Issues](https://img.shields.io/github/issues/EmilyMatt/mapping-rs)](https://github.com/EmilyMatt/mapping-rs/issues)
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/EmilyMatt/mapping-rs?tab=License-1-ov-file)

![GitHub Stars](https://img.shields.io/github/stars/EmilyMatt/mapping-rs)
![GitHub Watchers](https://img.shields.io/github/watchers/EmilyMatt/mapping-rs)
[![GitHub Forks](https://img.shields.io/github/forks/EmilyMatt/mapping-rs)](https://github.com/EmilyMatt/mapping-rs/fork)

[![Discord Channel](https://dcbadge.vercel.app/api/server/hKFKTaMKkq/)](https://discord.gg/j4z4WM3ZNV)

# mapping-rs

### A SLAM ecosystem for Rust

## ⚠️ Unstable API ⚠️

### Warning: this crate is in early development, breaking API changes are to be expected.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
mapping-algorithms = { "0.0.1", .. }
mapping-suites = { version = "0.0.1", .. }
```

# Features

## no_std support

While the `std` feature is enabled by default,
these crates were designed with `no_std` support in mind,\
provided that a memory allocator is configured
(these crate __do__ use the `alloc` crate).

It can be easily achieved like so:

```toml
[dependencies.mapping-algorithms]
default-features = false

[dependencies.mapping-suites]
default-features = false
```

## tracing

These crates provides profiling and instrumentation insight
via the [tracing](https://github.com/tokio-rs/tracing) crate.

To use it, simply enable the `tracing` feature in your Cargo.toml,
and use your choice of a subscriber.
Note that different functions have different tracing levels.

Since each and every function is instrumented, be sure to remember the overhead for enabling tracing.

## pregenerated

These crates heavily rely on generics, and suffers severe performance penalties in `debug`\
(We strive to be _very_ fast in `release` though).

For this purpose, a `pregenerated` feature exists, which provides access to public pre-generated functions for most use
cases and types.\
This is recommended for most users, and allows bypassing the generics overhead.

Usage:

In your Cargo.toml:

```toml
# Enables the pregenerated feature (This is enabled by default)
[dependencies.mapping-algorithms]
features = ["pregenerated"]

# Compiles these crates with max optimizations
[profile.dev.package.mapping-algorithms]
opt-level = 3

[profile.dev.package.mapping-suites]
opt-level = 3
```

Code example:

```rust,ignore
use mapping_algorithms::point_clouds;

// Instead of doing this:
let res = point_clouds::icp::<f32, 2 > (...); // Uses generics, uses your project's optimization level

// Do this(Runs much faster):
let res = point_clouds::single_precision::icp_2d(...); // Is a regular function, uses the crate's optimization level
```

The `pregenerated` macro is enabled by default.

## Contributing

If you would like to contribute, we welcome your contributions.
Please be sure to check out our [CONTRIBUTING.md](https://github.com/EmilyMatt/mapping-rs/blob/main/CONTRIBUTING.md)
