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

    #[test]
    fn test_is_nan_f32() {
        assert!(!<f32 as IsNan>::is_nan(0.0));
        assert!(!<f32 as IsNan>::is_nan(1.0));
        assert!(!<f32 as IsNan>::is_nan(f32::INFINITY));
        assert!(!<f32 as IsNan>::is_nan(f32::NEG_INFINITY));

        assert!(<f32 as IsNan>::is_nan(f32::NAN));
    }

    #[test]
    fn test_is_nan_f64() {
        assert!(!<f64 as IsNan>::is_nan(0.0));
        assert!(!<f64 as IsNan>::is_nan(1.0));
        assert!(!<f64 as IsNan>::is_nan(f64::INFINITY));
        assert!(!<f64 as IsNan>::is_nan(f64::NEG_INFINITY));

        assert!(<f64 as IsNan>::is_nan(f64::NAN));
    }

    #[test]
    fn test_is_nan_u8() {
        assert!(!0u8.is_nan());
        assert!(!1u8.is_nan());
    }

    #[test]
    fn test_is_nan_u16() {
        assert!(!0u16.is_nan());
        assert!(!1u16.is_nan());
    }

    #[test]
    fn test_is_nan_u32() {
        assert!(!0u32.is_nan());
        assert!(!1u32.is_nan());
    }

    #[test]
    fn test_is_nan_u64() {
        assert!(!0u64.is_nan());
        assert!(!1u64.is_nan());
    }

    #[test]
    fn test_is_nan_usize() {
        assert!(!0usize.is_nan());
        assert!(!1usize.is_nan());
    }

    #[test]
    fn test_is_nan_i8() {
        assert!(!0i8.is_nan());
        assert!(!1i8.is_nan());
    }

    #[test]
    fn test_is_nan_i16() {
        assert!(!0i16.is_nan());
        assert!(!1i16.is_nan());
    }

    #[test]
    fn test_is_nan_i32() {
        assert!(!0i32.is_nan());
        assert!(!1i32.is_nan());
    }

    #[test]
    fn test_is_nan_i64() {
        assert!(!0i64.is_nan());
        assert!(!1i64.is_nan());
    }

    #[test]
    fn test_is_nan_isize() {
        assert!(!0isize.is_nan());
        assert!(!1isize.is_nan());
    }
}
