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

/// This trait is used to check if a number is NaN.
/// For integer types, this will always return false.
pub trait IsNan: Copy {
    /// Returns true if self is NaN.
    fn is_nan(self) -> bool;
}

macro_rules! impl_is_nan {
    ($(($t:ty)),*) => {
        $(
            impl IsNan for $t {
                #[inline]
                fn is_nan(self) -> bool {
                    false
                }
            }
        )*
    };
    ($t:ty) => {
        impl IsNan for $t {
            #[inline]
            fn is_nan(self) -> bool {
                self.is_nan()
            }
        }
    }
}

impl_is_nan!(
    (u8),
    (u16),
    (u32),
    (u64),
    (u128),
    (usize),
    (i8),
    (i16),
    (i32),
    (i64),
    (i128),
    (isize)
);
impl_is_nan!(f32);
impl_is_nan!(f64);

#[cfg(test)]
mod tests {
    use super::*;

    macro_rules! impl_test {
        ($(fl $t:expr),*) => {
            $(
                ::paste::paste! {
                    #[test]
                    fn [<test_is_nan_ $t>]() {
                        assert!(!<$t as IsNan>::is_nan(0.0));
                        assert!(!<$t as IsNan>::is_nan(1.0));
                        assert!(!<$t as IsNan>::is_nan($t::INFINITY));
                        assert!(!<$t as IsNan>::is_nan($t::NEG_INFINITY));

                        assert!(<$t as IsNan>::is_nan($t::NAN));
                        assert!(<$t as IsNan>::is_nan(-$t::NAN));
                    }
                }
            )*
        };
        ($($t:expr),*) => {
            $(
                ::paste::paste! {
                    #[test]
                    fn [<test_is_nan_ $t>]() {
                        assert!(!<$t as IsNan>::is_nan(0));
                        assert!(!<$t as IsNan>::is_nan(1));
                    }
                }
            )*
        }
    }

    impl_test!(fl f32, fl f64);
    impl_test!(u8, u16, u32, u64, usize, i8, i16, i32, i64, isize);
}
