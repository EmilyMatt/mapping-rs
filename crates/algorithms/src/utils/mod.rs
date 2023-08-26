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
        isometry_matrix: Isometry<f32, R, N>,
    ) -> (Vec<Point<f32, N>>, Vec<Point<f32, N>>)
    where
        R: nalgebra::AbstractRotation<f32, N>,
    {
        use rand::Rng;
        let mut rng = rand::thread_rng();

        (0..100)
            .map(|_| {
                let storage: [f32; N] = std::array::from_fn(|_| rng.gen_range(-15.0f32..=15.0f32));
                let orig = nalgebra::Point::from(storage);
                let transformed = isometry_matrix.transform_point(&orig);

                (orig, transformed)
            })
            .unzip()
    }
}
