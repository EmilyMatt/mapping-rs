![Windows Build](https://github.com/EmilyMatt/mapping-rs/actions/workflows/build-win.yml/badge.svg)
![Linux Build](https://github.com/EmilyMatt/mapping-rs/actions/workflows/build-linux.yml/badge.svg)
![MacOS Build](https://github.com/EmilyMatt/mapping-rs/actions/workflows/build-macos.yml/badge.svg)
![Documentation](https://github.com/EmilyMatt/mapping-rs/actions/workflows/doc.yml/badge.svg)
![GitHub Stars](https://img.shields.io/github/stars/EmilyMatt/mapping-rs)
![GitHub Watchers](https://img.shields.io/github/watchers/EmilyMatt/mapping-rs)
![GitHub Forks](https://img.shields.io/github/forks/EmilyMatt/mapping-rs)
[![GitHub Issues](https://img.shields.io/github/issues/EmilyMatt/mapping-rs)](https://github.com/EmilyMatt/mapping-rs/issues)
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/EmilyMatt/mapping-rs/blob/master/LICENSE)

![Discord Channel](https://dcbadge.vercel.app/api/server/hKFKTaMKkq/)

## Unstable API
Note that this crate is in early development, breaking API changes are to be expected.

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
mapping-algorithms-rs = "0.0.1"
```

## Features

This crate can be used without the standard library, given that a memory allocator is configured(this crate __does__ use the `alloc` crate).

this can be easily achieved by disabling the default `std` feature. Like so:

```toml
[dependencies.mapping-algorithms-rs]
version = "0.0.1"
default-features = false
```

This crate is designed to be able to take advantage of CUDA for parallel processing, 
this can greatly improve algorithm performance, but requires an NVIDIA graphics card and drivers.
To enable CUDA, use the `cuda` feature:
```toml
[dependencies]
mapping-algorithms-rs = { version = "0.0.1", features = ["cuda"] }
```