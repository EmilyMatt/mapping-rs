use crate::{utils::distance_squared, Box};
use nalgebra::{Point, Scalar};
use num_traits::NumOps;

struct KDNode<T, const N: usize>
where
    T: Copy + Default + NumOps + PartialOrd + Scalar,
{
    internal_data: Point<T, N>,
    right: Option<Box<KDNode<T, N>>>,
    left: Option<Box<KDNode<T, N>>>,
}

impl<T, const N: usize> KDNode<T, N>
where
    T: Copy + Default + NumOps + PartialOrd + Scalar,
{
    fn new(data: Point<T, N>) -> Self {
        Self {
            internal_data: data,
            left: None,
            right: None,
        }
    }

    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Insert New Point", skip_all, level = "trace")
    )]
    fn insert(&mut self, data: Point<T, N>, depth: usize) {
        let dimension_to_check = depth % N;

        let branch_to_use =
            // Note that this is a &mut Option, not an Option<&mut>!
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
#[derive(Default)]
pub struct KDTree<T, const N: usize>
where
    T: Copy + Default + NumOps + PartialOrd + Scalar,
{
    root: Option<KDNode<T, N>>,
}

impl<T, const N: usize> KDTree<T, N>
where
    T: Copy + Default + NumOps + PartialOrd + Scalar,
{
    /// Returns an empty instance of this tree structure
    pub fn new() -> Self {
        Self { root: None }
    }

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
    T: Copy + Default + NumOps + PartialOrd + Scalar,
{
    #[cfg_attr(
        feature = "tracing",
        tracing::instrument("Generate Tree From Point Cloud", skip_all, level = "info")
    )]
    fn from(point_cloud: &[Point<T, N>]) -> Self {
        point_cloud
            .iter()
            .copied()
            .fold(Self::new(), |mut tree, current_point| {
                tree.insert(current_point);
                tree
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{utils::point_cloud::find_closest_point, Vec};
    use nalgebra::{Point2, Point3};

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
        let mut tree = KDTree::new();
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
    fn test_nearest() {
        // Test an empty tree
        {
            let tree = KDTree::<f32, 2>::new();
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
            .map(|point_a| find_closest_point(point_a, points_b.as_slice()))
            .collect::<Vec<_>>();
        let closest_point_kd = points_a
            .iter()
            .map(|point_a| kd_tree.nearest(point_a).unwrap())
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
}
