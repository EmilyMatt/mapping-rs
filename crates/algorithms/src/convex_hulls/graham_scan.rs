use std::ops;
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
use crate::types::IsNan;
use nalgebra::{Point, Scalar};

pub fn graham_scan<
    T: Copy + Scalar + PartialOrd + IsNan + num_traits::NumOps + num_traits::NumAssignOps,
    const N: usize,
>(
    points: &[Point<T, N>],
) -> Option<()> {
    let sorted = crate::point_clouds::lex_sort(points)?;

    let upper_hull = sorted
        .iter()
        .fold(vec![], |mut acc: Vec<&Point<T, N>>, it| {
            loop {
                if acc.len() < 2 {
                    break;
                }

                let a = acc[acc.len() - 2];
                let b = acc[acc.len() - 1];
                let c = it;

                // pop if cross product is to the left
            }

            // Push the new point
            acc.push(it);
            acc
        });

    // Do the same thing for the lower hull but in reverse, kinda

    Some(())
}
