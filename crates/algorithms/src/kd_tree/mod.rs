use nalgebra::{distance_squared, ComplexField, Point, RealField};
use std::iter::Sum;

struct KDNode<T, const N: usize>
where
    T: ComplexField + Copy + Default + RealField + Sum,
{
    internal_data: Point<T, N>,
    right: Option<Box<KDNode<T, N>>>,
    left: Option<Box<KDNode<T, N>>>,
}

impl<T, const N: usize> KDNode<T, N>
where
    T: ComplexField + Copy + Default + RealField + Sum,
{
    fn new(data: Point<T, N>) -> Self {
        Self {
            internal_data: data,
            left: None,
            right: None,
        }
    }

    fn insert(&mut self, data: Point<T, N>, depth: usize) {
        let dimension_to_check = depth % N;

        let branch_to_use =
            // Note that this is an &mut Option, not an Option<&mut>!
            if data.coords[dimension_to_check] < self.internal_data.coords[dimension_to_check] {
                &mut self.left
            } else {
                &mut self.right
            };

        if let Some(branch_exists) = branch_to_use.as_mut() {
            branch_exists.insert(data, depth + 1);
        } else {
            *branch_to_use = Some(Box::new(KDNode::new(data)))
        }
    }

    fn nearest(&self, target: &Point<T, N>, depth: usize) -> Option<Point<T, N>> {
        let dimension_to_check = depth % N;
        let (next_branch, opposite_branch) =
            if target.coords[dimension_to_check] < self.internal_data.coords[dimension_to_check] {
                (self.left.as_ref(), self.right.as_ref())
            } else {
                (self.right.as_ref(), self.left.as_ref())
            };

        // Start with the nearer branch, default to this branch's point
        let best = next_branch
            .and_then(|branch| branch.nearest(target, depth + 1))
            .unwrap_or(self.internal_data);

        if distance_squared(&best, target) > distance_squared(&best, target) {
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

    fn traverse_branch<F: Fn(&Point<T, N>)>(&self, func: &F) {
        if let Some(left) = self.left.as_ref() {
            left.traverse_branch(func);
        }
        func(&self.internal_data);
        if let Some(right) = self.right.as_ref() {
            right.traverse_branch(func);
        }
    }

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
/// `N`: a constant usize specifying how many dimensions should each point have.
#[derive(Default)]
pub struct KDTree<T, const N: usize>
where
    T: ComplexField + Copy + Default + RealField + Sum,
{
    root: Option<KDNode<T, N>>,
}

impl<T, const N: usize> KDTree<T, N>
where
    T: ComplexField + Copy + Default + RealField + Sum,
{
    /// Returns an empty instance of this tree structure
    pub fn new() -> Self {
        Default::default()
    }

    /// Inserts a new data points into the tree, taking into consideration it's position.
    ///
    /// # Arguments
    /// * `data`: a [`Point`], to be inserted into the tree.
    pub fn insert(&mut self, data: Point<T, N>) {
        if let Some(root) = self.root.as_mut() {
            root.insert(data, 0);
        } else {
            self.root = Some(KDNode::new(data));
        }
    }

    /// Attempts to find the nearest point in the tree for the specified target point.
    /// # Arguments
    /// * `target`: a [`Point`], to search the closest point for.
    ///
    /// # Returns
    /// [`None`] if the tree is empty, otherwise returns the closest [`Point`].
    pub fn nearest(&mut self, target: &Point<T, N>) -> Option<Point<T, N>> {
        self.root.as_ref().and_then(|root| root.nearest(target, 0))
    }

    /// Allows traversal of the entire tree structure, calling [`func`] on each branch's data.
    ///
    /// # Arguments
    /// * `func`: a closure of type [`Fn`], it's only parameter is a reference of the branch's [`Point`].
    pub fn traverse_tree<F: Fn(&Point<T, N>)>(&self, func: F) {
        if let Some(root) = self.root.as_ref() {
            root.traverse_branch(&func);
        }
    }

    /// Allows traversal of the entire tree structure, calling [`func`] on each branch's data, possible mutating the data.
    ///
    /// # Arguments
    /// * func: a closure of type [`FnMut`], it's only parameter is a reference of the branch's [`Point`].
    pub fn traverse_tree_mut<F: FnMut(&mut Point<T, N>)>(&mut self, mut func: F) {
        if let Some(root) = self.root.as_mut() {
            root.traverse_branch_mut(&mut func);
        }
    }
}

impl<T, const N: usize> From<&[Point<T, N>]> for KDTree<T, N>
where
    T: ComplexField + Copy + Default + RealField + Sum,
{
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
