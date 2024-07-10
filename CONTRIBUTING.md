<!-- omit in toc -->
# Contributing to mapping-rs!

First off, thanks for taking the time to contribute! ❤️

See the [Table of Contents](#table-of-contents) for different ways to help and details about how this project handles them. Please make sure to read the relevant section before making your contribution. It will make it a lot easier for us maintainers and smooth out the experience for all involved. The community looks forward to your contributions. 🎉

> And if you like the project, but just don't have time to contribute, that's fine. There are other easy ways to support the project and show your appreciation, which we would also be very happy about:
> - Star the project
> - Tweet about it
> - Refer this project in your project's readme
> - Mention the project at local meetups and tell your friends/colleagues

<!-- omit in toc -->
## Table of Contents

- [I Want To Contribute](#i-want-to-contribute)
  - [Reporting Bugs](#reporting-bugs)
  - [Contributing Code](#contributing-code)


## I Want To Contribute

> ### Legal Notice <!-- omit in toc -->
> When contributing to this project, you must agree that you have authored 100% of the content, that you have the necessary rights to the content and that the content you contribute may be provided under the project license.

### Reporting Bugs

<!-- omit in toc -->
#### Before Submitting a Bug Report

A good bug report shouldn't leave others needing to chase you up for more information. Therefore, we ask you to investigate carefully, collect information and describe the issue in detail in your report. Please complete the following steps in advance to help us fix any potential bug as fast as possible.

- Make sure that you are using the latest version.
- Determine if your bug is really a bug and not an error on your side e.g. using incompatible environment components/versions.
- To see if other users have experienced (and potentially already solved) the same issue you are having, check if there is not already a bug report existing for your bug or error in the [bug tracker](https://github.com/EmilyMatt/mapping-rs/issues?q=label%3Abug).
- Collect information about the bug:
  - Stack trace (Traceback)
  - OS, Platform and Version (Windows, Linux, macOS, x86, ARM)
  - Possibly your input and the output
  - Can you reliably reproduce the issue? And can you also reproduce it with older versions?

<!-- omit in toc -->
#### How Do I Submit a Good Bug Report?

> NOTE: You must never report security related issues, vulnerabilities or bugs including sensitive information to the issue tracker, or elsewhere in public. Instead sensitive bugs must be sent by email to Emily Matheys, directly at `emilymatt96@gmail.com`.
<!-- You may add a PGP key to allow the messages to be sent encrypted as well. -->

We use GitHub issues to track bugs and errors. If you run into an issue with the project:

- Open an [Issue](https://github.com/EmilyMatt/mapping-rs/issues/new). (Since we can't be sure at this point whether it is a bug or not, we ask you not to talk about a bug yet and not to label the issue.)
- Explain the behavior you would expect and the actual behavior.
- Please provide as much context as possible and describe the *reproduction steps* that someone else can follow to recreate the issue on their own. This usually includes your code. For good bug reports you should isolate the problem and create a reduced test case.
- Provide the information you collected in the previous section.


### Contributing Code
#### Settings up your environment
> NOTE: We recommend installing the Rust toolchain etc. using [Rustup](https://rustup.rs), but you are free to install it anyway your like, this is not a requirement as long as you use the *STABLE* MSRV or newer (Minimum Supported Rust Version)

- Before beginning the process, set up your environment, with your favorite IDE/Code Editor, and have the relevant Rust extensions/plugins installed.
- In order to save your precious time, and shorten the process for your PRs to be acknowledged, reviewed, and approved, please add the following jobs to your IDE.
  * Compile the project: `cargo build`
  * Run the 'Clippy' linter on the project: `cargo clippy`
  * Generate the project documentation: `cargo doc`
  * Run the project tests: `cargo <test/nextest>`
- Please ensure these jobs pass on your local machine with both:
  - The standard library and all required features (by adding `--all-features` after the job)
  - Without the standard library (by adding `--no-default-features --features=pregenerated` after the job)

#### Coding Guidelines
- Project layout: This crate adheres to the [Cargo project layout](https://doc.rust-lang.org/cargo/guide/project-layout.html), but uses the old module system, in which each module has its own directory containing a "mod.rs" file, which is the entrypoint to that module.


- This library is meant to be performant and robust, when implementing a feature or enhancement, you should ensure that your code is as optimal as you can, and that it is generic enough to be used in various ways. Points to consider:
  - Is my code using a specific type that will require the user to perform a data type conversion? i.e., using an `f32` in an algorithm that should not be constricted to that precision, using a specific `nalgebra::Matrix` storage that limits usage.
  - Is my code doing its job with as little operations as it can? consider that when using gigantic datasets, a simple iteration can become a serious problem.
  - Is my code written in a clear manner, that allows a developer to follow it quickly and understand its purpose?


- We do not accept any user-facing functions or structs that are not *fully* documented, please ensure that your documentation encompasses all the information required for the user. Points to verify:
  - What does this function do, what does this struct represent/contain?
  - Arguments/members, are they all explained and documented? 
  - What does this function return(both type and purpose).
  - Could this function panic under some conditions? please specify all these conditions.
  - All structs/functions/traits mentioned in docs *must* be intra-linked.


- All functions must be instrumented using the `tracing` crate(using `cfg_attr` to ensure instrumentation is only compiled when the `tracing` feature is enabled). For example: `#[cfg_attr(feature = "tracing", tracing::instrument("Insert New Point", skip_all))]`. Note that we do not pass function parameters to instrumentation.


- **Testing, testing, testing**, When implementing any feature, we are aiming for *100% code coverage*, this means that every possible branch of your code should be tested and accounted for:
  - If you are implementing an algorithm, you should have a main test verifying the algorithm functions as intended.
  - *all helper functions* should also be tested and accounted for, each one should have its own test verifying that it does what you think it does, this is critical, and allows us to a) identify bugs much easier, and b) ensure that no one pushes changes that will break something undetected.
  - If you are implementing a function that might work in a different way when given a specific configuration, you should have a test for each of these configuration options, even if the test's outputs are the same.
  - If your function may return an error in certain cases, write a test *for each of those cases*.
  - This does not mean that this crate requires you to work in a test-driven-development paradigm, but you are free to do so if you wish.

#### Writing Examples
When implementing a new algorithm, suite, or feature, it is nice to add an [example](https://doc.rust-lang.org/cargo/reference/cargo-targets.html#examples), but it is not mandatory for that PR (as established, the function will already be fully documented), you can, instead, [open a new issue](https://github.com/EmilyMatt/mapping-rs/issues/new), label it with the "Enhancement" label, and someone else will request assignment and create that example.
An example, should be very clear in showing *how* to use the crate's API to perform that specific functionality, but should also have much more focus on UI/UX, and clearly and most importantly *visually* present the feature.
Generally, examples in this crate use [egui](https://github.com/emilk/egui), which is a fantastic ecosystem for Immediate-style GUI purposes, we use it with the `glow` feature enabled, for when `OpenGL` interfacing is required (mostly for 3D examples).

<!-- omit in toc -->
## Attribution
This guide is based on the **contributing-gen**. [Make your own](https://github.com/bttger/contributing-gen)!
