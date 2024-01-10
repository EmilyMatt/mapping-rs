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
mapping-algorithms-rs = { git = "https://github.com/EmilyMatt/mapping-rs.git" }
```

Note that in this crate heavily uses generics, and is therefor suffering penalties in `debug` profile, (but _very_ fast in `release`).

I'd recommend adding the following in your Cargo.toml, and using the macro pre-generated functions whenever possible:
```toml
[profile.dev.package.mapping-rs]
opt-level = 3
```

# Features

## no_std
This crate can be used without the standard library, given that:

a. memory allocator is configured(this crate __does__ use the `alloc` crate).

b. the `libm` feature is enabled.

this can be easily achieved like so:

```toml
[dependencies.mapping-algorithms-rs]
default-features = false
features = ["libm"]
```

## tracing
This crate provides profiling and instrumentation insight 
via the [tracing](https://github.com/tokio-rs/tracing) crate, in order to use that, 
simply enable the `tracing` feature in your Cargo.toml, and use your choice of a subscriber.

## CUDA (Future-Feature)
This crate is designed to be able to take advantage of CUDA for parallel processing, 
this can greatly improve algorithm performance, but requires an NVIDIA graphics card and drivers.
To enable CUDA, use the `cuda` feature:
```toml
[dependencies.mapping-algorithms-rs]
features = ["cuda"]
```

## Contributing
If you'd like to contribute, we'll gladly have you, but be sure to check out our [CONTRIBUTING.md](CONTRIBUTING.md)