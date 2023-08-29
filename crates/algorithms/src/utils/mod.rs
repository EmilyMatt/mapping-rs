use nalgebra::{ComplexField, Point, RealField};

#[cfg(feature = "tracing")]
use tracing::instrument;

/// Finds the closest matching target point to the passed source point.
///
/// # Arguments
/// * `transformed_point`: a [`Point`], for which to find the closest point.
/// * `target_points`: a slice of [`Point`], representing the target point cloud.
///
/// # Returns
/// A [`Point`], representing said closest point.
///
/// # Panics
/// In debug builds, this function will panic if the `target_points` is an empty slice.
#[inline]
#[cfg_attr(feature = "tracing", instrument("Find Closest Points", skip_all))]
pub(crate) fn find_closest_point<'a, T, const N: usize>(
    transformed_point: &'a Point<T, N>,
    target_points: &'a [Point<T, N>],
) -> Point<T, N>
where
    T: ComplexField + RealField + Copy,
{
    debug_assert!(!target_points.is_empty());

    let mut current_distance = T::max_value().expect("Number Must Have a MAX value");
    let mut current_idx = 0;
    for (idx, target_point) in target_points.iter().enumerate() {
        let distance = (transformed_point - target_point).norm();

        if distance < current_distance {
            current_distance = distance;
            current_idx = idx;
        }
    }

    // Guaranteed to exist, so direct access is valid
    target_points[current_idx]
}

#[cfg(test)]
pub(crate) mod tests {
    use nalgebra::{Isometry, Point};

    /// Generates a points cloud, and a corresponding points cloud, transformed by `isometry_matrix`
    /// # Arguments
    /// * `isometry_matrix`: an [`Isometry`], with [`R`] being [`UnitQuaternion`](nalgebra::UnitQuaternion) for [`N = 3`] and [`UnitComplex`](nalgebra::UnitComplex) for [`N = 2`]
    ///
    /// # Returns
    /// A tuple of two [`Vec<Point<f32, N>>`]'s, the first one being the source set of points, and the second the transformed set of points.
    pub fn generate_points<const N: usize, R>(
        num_points: usize,
        isometry_matrix: Isometry<f32, R, N>,
    ) -> (Vec<Point<f32, N>>, Vec<Point<f32, N>>)
    where
        R: nalgebra::AbstractRotation<f32, N>,
    {
        use rand::Rng;
        let mut rng = rand::thread_rng();

        (0..num_points)
            .map(|_| {
                let storage: [f32; N] = std::array::from_fn(|_| rng.gen_range(-15.0f32..=15.0f32));
                let orig = nalgebra::Point::from(storage);
                let transformed = isometry_matrix.transform_point(&orig);

                (orig, transformed)
            })
            .unzip()
    }
}
