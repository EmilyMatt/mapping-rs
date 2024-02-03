[![Windows Build](https://github.com/EmilyMatt/mapping-rs/actions/workflows/build-win.yml/badge.svg)](https://github.com/EmilyMatt/mapping-rs/actions/workflows/build-win.yml)
[![Linux Build](https://github.com/EmilyMatt/mapping-rs/actions/workflows/build-linux.yml/badge.svg)](https://github.com/EmilyMatt/mapping-rs/actions/workflows/build-linux.yml)
[![MacOS Build](https://github.com/EmilyMatt/mapping-rs/actions/workflows/build-macos.yml/badge.svg)](https://github.com/EmilyMatt/mapping-rs/actions/workflows/build-macos.yml)
[![Documentation](https://github.com/EmilyMatt/mapping-rs/actions/workflows/doc.yml/badge.svg)](https://github.com/EmilyMatt/mapping-rs/actions/workflows/doc.yml)
[![codecov](https://codecov.io/gh/EmilyMatt/mapping-rs/graph/badge.svg?token=GSPWQVRCV8)](https://codecov.io/gh/EmilyMatt/mapping-rs)

![GitHub Stars](https://img.shields.io/github/stars/EmilyMatt/mapping-rs)
![GitHub Watchers](https://img.shields.io/github/watchers/EmilyMatt/mapping-rs)
[![GitHub Forks](https://img.shields.io/github/forks/EmilyMatt/mapping-rs)](https://github.com/EmilyMatt/mapping-rs/fork)
[![GitHub Issues](https://img.shields.io/github/issues/EmilyMatt/mapping-rs)](https://github.com/EmilyMatt/mapping-rs/issues)
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

[![Discord Channel](https://dcbadge.vercel.app/api/server/hKFKTaMKkq/)](https://discord.gg/j4z4WM3ZNV)

## Unstable API
Warning: this crate is in early development, breaking API changes are to be expected.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
mapping-algorithms-rs = { git = "https://github.com/EmilyMatt/mapping-rs.git" }
```

# Features

## std
While the `std` feature is enabled by default,
this crate can be used without the standard library, provided that a memory allocator is configured (this crate __does__ use the `alloc` crate).

It can be easily achieved like so:

```toml
[dependencies.mapping-algorithms-rs]
default-features = false
```

## tracing
This crate provides profiling and instrumentation insight 
via the [tracing](https://github.com/tokio-rs/tracing) crate.

To use it, simply enable the `tracing` feature in your Cargo.toml, 
and use your choice of a subscriber.

# pregenerated
Note that this crate heavily relies on generics, and therefore suffers performance penalties in `debug`, (but is _very_ fast in `release`).
A `pregenerated` feature exists, which provides access to public pre-generated functions for most use cases and types.

I recommend adding the following to your Cargo.toml, and using the macro pre-generated functions whenever possible, 
bypassing the generics overhead:
```toml
[dependencies.mapping-algorithms-rs]
features = ["pregenerated"]
```

```toml
[profile.dev.package.mapping-rs]
opt-level = 3
```

Example:
```rust
// Instead of doing this:
let res = icp::icp::<f32, 2, Const<2>>(...);

// Do this(Runs much faster):
let res = icp::f32::icp_2d(...);
```

The `pregenerated` macro is enabled by default.

## CUDA (Future-Feature)
This crate is designed to take advantage of CUDA for parallel processing; 
this can greatly improve algorithm performance but requires an NVIDIA graphics card and drivers.

To enable CUDA, use the `cuda` feature:
```toml
[dependencies.mapping-algorithms-rs]
features = ["cuda"]
```

## Contributing
If you would like to contribute, we welcome your contributions.
Please be sure to check out our [CONTRIBUTING.md](CONTRIBUTING.md)