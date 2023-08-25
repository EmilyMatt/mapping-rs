#[cfg(test)]
pub fn generate_points<const N: usize, R>(
    isometry_matrix: nalgebra::Isometry<f32, R, N>,
) -> (Vec<nalgebra::Point<f32, N>>, Vec<nalgebra::Point<f32, N>>)
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
