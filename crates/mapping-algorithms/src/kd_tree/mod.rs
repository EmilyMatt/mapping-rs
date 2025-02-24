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

use nalgebra::{Point, Scalar};
use num_traits::{NumOps, Zero};

use crate::{utils::distance_squared, Box, Ordering};

#[derive(Clone, Debug, Default)]
struct KDNode<T, const N: usize>
where
    T: Copy + Default + NumOps + PartialOrd + Scalar + Zero,
{
    internal_data: Point<T, N>,
    right: Option<Box<KDNode<T, N>>>,
    left: Option<Box<KDNode<T, N>>>,
}

impl<T, const N: usize> KDNode<T, N>
where
    T: Copy + Default + NumOps + PartialOrd + Scalar + Zero,
{
    fn new(root: Point<T, N>) -> Self {
        Self {
            internal_data: root,
            left: None,
            right: None,
        }
    }

    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Insert New Point", skip_all, level = "trace")
    )]
    fn insert(&mut self, data: Point<T, N>, depth: usize) -> bool {
        let dimension_to_check = depth % N;

        let (branch_to_use, verify_equals) =
            // Note that this is a &mut Option, not an Option<&mut>!
            match data.coords[dimension_to_check].partial_cmp(&self.internal_data.coords[dimension_to_check]).unwrap() {
                Ordering::Less => (&mut self.left, false),
                Ordering::Equal => (&mut self.right, true),
                Ordering::Greater => (&mut self.right, false)
            };

        if let Some(branch_exists) = branch_to_use.as_mut() {
            return branch_exists.insert(data, depth + 1);
        } else if verify_equals && self.internal_data == data {
            return false;
        }

        *branch_to_use = Some(Box::new(KDNode::new(data)));
        true
    }

    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Branch Nearest Neighbour", skip_all, level = "trace")
    )]
    fn nearest(&self, target: &Point<T, N>, depth: usize) -> Option<Point<T, N>> {
        let dimension_to_check = depth % N;
        let (next_branch, opposite_branch) =
            if target.coords[dimension_to_check] < self.internal_data.coords[dimension_to_check] {
                (self.left.as_ref(), self.right.as_ref())
            } else {
                (self.right.as_ref(), self.left.as_ref())
            };

        // Start with the nearer branch, default to this branch's point
        let mut best = next_branch
            .and_then(|branch| branch.nearest(target, depth + 1))
            .unwrap_or(self.internal_data);

        let axis_distance =
            target.coords[dimension_to_check] - self.internal_data.coords[dimension_to_check];

        if distance_squared(&self.internal_data, target) < distance_squared(&best, target) {
            best = self.internal_data;
        }

        if (axis_distance * axis_distance) < distance_squared(&best, target) {
            if let Some(opposite_best) =
                opposite_branch.and_then(|branch| branch.nearest(target, depth + 1))
            {
                if distance_squared(&opposite_best, target) < distance_squared(&best, target) {
                    return Some(opposite_best);
                }
            }
        }

        Some(best)
    }

    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Traverse Branch With Function", skip_all, level = "debug")
    )]
    fn traverse_branch<F: FnMut(&Point<T, N>)>(&self, func: &mut F) {
        if let Some(left) = self.left.as_ref() {
            left.traverse_branch(func);
        }
        func(&self.internal_data);
        if let Some(right) = self.right.as_ref() {
            right.traverse_branch(func);
        }
    }

    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Traverse Branch With Mutable Function)", skip_all, level = "debug")
    )]
    fn traverse_branch_mut<F: FnMut(&mut Point<T, N>)>(&mut self, func: &mut F) {
        if let Some(left) = self.left.as_mut() {
            left.traverse_branch_mut(func);
        }
        func(&mut self.internal_data);
        if let Some(right) = self.right.as_mut() {
            right.traverse_branch_mut(func);
        }
    }
}

/// The Actual K-Dimensional Tree struct, contains it's first node.
///
/// # Generics
/// `T`: Either an [`f32`] or [`f64`]
/// `N`: a const usize specifying how many dimensions should each point have.
#[derive(Clone, Debug, Default)]
pub struct KDTree<T, const N: usize>
where
    T: Copy + Default + NumOps + PartialOrd + Scalar + Zero,
{
    root: Option<KDNode<T, N>>,
    element_count: usize,
}

impl<T, const N: usize> KDTree<T, N>
where
    T: Copy + Default + NumOps + PartialOrd + Scalar + Zero,
{
    /// Inserts a new data points into the tree, taking into consideration it's position.
    ///
    /// # Arguments
    /// * `data`: a [`Point`], to be inserted into the tree.
    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Insert To Tree", skip_all, level = "debug")
    )]
    pub fn insert(&mut self, data: Point<T, N>) {
        if let Some(root) = self.root.as_mut() {
            if root.insert(data, 0) {
                self.element_count += 1;
            }
        } else {
            self.root = Some(KDNode::new(data));
            self.element_count = 1;
        }
    }

    /// Returns the number of elements in the tree.
    ///
    /// # Returns
    /// A [`usize`] representing the number of elements in the tree.
    pub fn len(&self) -> usize {
        self.element_count
    }

    /// Returns whether the tree is empty or not.
    ///
    /// # Returns
    /// A [`bool`] representing whether the tree is empty or not.
    pub fn is_empty(&self) -> bool {
        self.element_count == 0
    }

    /// Attempts to find the nearest point in the tree for the specified target point.
    /// # Arguments
    /// * `target`: a [`Point`], to search the closest point for.
    ///
    /// # Returns
    /// [`None`] if the tree is empty, otherwise returns the closest [`Point`].
    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Find Nearest Neighbour", skip_all, level = "debug")
    )]
    pub fn nearest(&self, target: &Point<T, N>) -> Option<Point<T, N>> {
        self.root.as_ref().and_then(|root| root.nearest(target, 0))
    }

    /// Allows traversal of the entire tree structure, calling the `func` closure on each branch's data.
    ///
    /// # Arguments
    /// * `func`: a closure of type [`Fn`], it's only parameter is a reference of the branch's [`Point`].
    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Traverse Tree With Function", skip_all, level = "info")
    )]
    pub fn traverse_tree<F: FnMut(&Point<T, N>)>(&self, mut func: F) {
        if let Some(root) = self.root.as_ref() {
            root.traverse_branch(&mut func);
        }
    }

    /// Allows traversal of the entire tree structure, calling the `func` closure on each branch's data, possible mutating the data.
    ///
    /// # Arguments
    /// * func: a closure of type [`FnMut`], it's only parameter is a reference of the branch's [`Point`].
    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Traverse Tree With Mutable Function", skip_all, level = "info")
    )]
    pub fn traverse_tree_mut<F: FnMut(&mut Point<T, N>)>(&mut self, mut func: F) {
        if let Some(root) = self.root.as_mut() {
            root.traverse_branch_mut(&mut func);
        }
    }
}

impl<T, const N: usize> From<&[Point<T, N>]> for KDTree<T, N>
where
    T: Copy + Default + NumOps + PartialOrd + Scalar + Zero,
{
    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Generate Tree From Point Cloud", skip_all, level = "info")
    )]
    fn from(point_cloud: &[Point<T, N>]) -> Self {
        point_cloud
            .iter()
            .copied()
            .fold(Self::default(), |mut tree, current_point| {
                tree.insert(current_point);
                tree
            })
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point2, Point3};

    use crate::{point_clouds::find_nearest_neighbour_naive, Vec};

    use super::*;

    fn generate_tree() -> KDTree<f32, 3> {
        let points = Vec::from([
            Point3::new(0.0, 2.0, 1.0),
            Point3::new(-1.0, 4.0, 2.5),
            Point3::new(1.3, 2.5, 0.5),
            Point3::new(-2.1, 0.2, -0.2),
        ]);
        KDTree::from(points.as_slice())
    }

    #[test]
    fn test_insert() {
        // Test an empty tree
        let mut tree = KDTree::default();
        tree.insert(Point2::new(0.0f32, 0.0f32));

        match tree.root.as_ref() {
            None => {
                panic!("Error, tree root should be Some()")
            }
            Some(root) => {
                assert_eq!(root.internal_data, Point2::new(0.0f32, 0.0f32));
            }
        }

        // Inserting new element
        // Since x is less than root's x, first divergence should be to the left branch.
        tree.insert(Point2::new(-1.0f32, 0.4f32));
        match tree.root.as_ref().unwrap().left.as_ref() {
            None => {
                panic!("Error, first left branch should be Some()");
            }
            Some(left_branch) => {
                assert_eq!(left_branch.internal_data, Point2::new(-1.0f32, 0.4f32));
            }
        }

        // Since second element's x is still less than root's x, right branch should be unchanged.
        tree.insert(Point2::new(-2.0f32, -3.0f32));
        assert!(tree.root.as_ref().unwrap().right.is_none());

        // Third element's x is larger than root's x, so it should be in right branch.
        tree.insert(Point2::new(1.4f32, 5.0f32));
        match tree.root.as_ref().unwrap().right.as_ref() {
            None => {
                panic!("Error, first right branch should be Some()");
            }
            Some(right_branch) => {
                assert_eq!(right_branch.internal_data, Point2::new(1.4f32, 5.0f32));
            }
        }
    }

    #[test]
    fn test_insert_duplicate() {
        let mut tree = KDTree::default();
        assert!(tree.is_empty());

        tree.insert(Point2::new(0.0f32, 0.0f32));
        assert_eq!(tree.len(), 1);
        assert!(!tree.is_empty());

        // Insert duplicate
        tree.insert(Point2::new(0.0f32, 0.0f32));
        assert_eq!(tree.len(), 1);
    }

    #[test]
    fn test_nearest() {
        // Test an empty tree
        {
            let tree = KDTree::<f32, 2>::default();
            assert!(tree.nearest(&Point2::new(0.0, 0.0)).is_none())
        }

        let tree = generate_tree();
        let nearest = tree.nearest(&Point3::new(1.32, 2.7, 0.2));
        assert!(nearest.is_some());
        assert_eq!(nearest.unwrap(), Point3::new(1.3, 2.5, 0.5));
    }

    #[test]
    fn compare_nearest_with_naive_version() {
        let points_a = [
            [8.037338, -10.512266, 5.3038273],
            [-13.573973, 5.2957783, -5.7758245],
            [5.399618, 14.216839, 13.042112],
            [10.134924, -3.9498444, 12.201418],
            [-3.7965546, -4.1447372, 3.7468758],
            [2.494978, -5.231186, 10.918207],
            [10.469978, 2.231762, 12.076345],
            [-11.764912, 14.629526, -14.80231],
            [-8.693936, 5.038475, -0.32558632],
            [7.616955, -3.7277327, 2.344328],
            [-11.924471, -11.668331, -1.2298765],
            [-14.369208, -7.1591473, -9.843174],
        ]
        .into_iter()
        .map(Point3::from)
        .collect::<Vec<_>>();

        let points_b = [
            [6.196747, -11.11811, 0.470586],
            [-13.9269495, 9.677899, 1.9754279],
            [13.07056, 12.289567, 9.591913],
            [12.668911, -6.104495, 5.763672],
            [-3.2386777, -2.61825, 5.1327395],
            [5.2409143, -5.826359, 8.294433],
            [14.281796, -0.12630486, 5.762767],
            [-2.7135608, 15.505872, 16.110285],
            [5.980031, -4.006213, -1.6124942],
            [-14.19904, -7.7923203, 4.401306],
            [-19.287233, -1.7146804, -1.7363598],
        ]
        .into_iter()
        .map(Point3::from)
        .collect::<Vec<_>>();

        let kd_tree = KDTree::from(points_b.as_slice());

        let closest_points_naive = points_a
            .iter()
            .map(|point_a| find_nearest_neighbour_naive(point_a, points_b.as_slice()))
            .collect::<Vec<_>>();
        let closest_point_kd = points_a
            .iter()
            .map(|point_a| kd_tree.nearest(point_a))
            .collect::<Vec<_>>();
        assert_eq!(closest_points_naive, closest_point_kd);
    }

    #[test]
    fn test_traverse_tree() {
        let tree = generate_tree();
        let mut sum = 0.0;
        tree.traverse_tree(|point| {
            sum += point.x + point.y;
        });

        assert_eq!(sum, 6.9); // Nice
    }

    #[test]
    fn test_traverse_tree_mut() {
        let mut tree = generate_tree();
        tree.traverse_tree_mut(|point| {
            *point = Point3::new(1.0, 1.0, 1.0);
        });

        tree.traverse_tree(|point| {
            assert_eq!(point.x, 1.0);
            assert_eq!(point.y, 1.0);
            assert_eq!(point.z, 1.0);
        });
    }

    #[test]
    fn test_multiple_elements_structure() {
        let mut tree = KDTree::default();
        let points = Vec::from([
            Point2::new(3.0, 6.0),
            Point2::new(17.0, 15.0),
            Point2::new(13.0, 15.0),
            Point2::new(6.0, 12.0),
            Point2::new(9.0, 1.0),
            Point2::new(2.0, 7.0),
            Point2::new(10.0, 19.0),
        ]);

        for point in points.iter() {
            tree.insert(*point);
        }

        assert_eq!(tree.len(), 7);
    }
}
