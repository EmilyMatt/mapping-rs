use crate::types::LongDMatrix;
use nalgebra::{
    AbstractRotation, ArrayStorage, Const, Dyn, Isometry, Matrix, Point, VecStorage, Vector,
};

pub fn get_mean_point_nd<const N: usize>(
    point_cloud: &Matrix<f32, Const<N>, Dyn, VecStorage<f32, Const<N>, Dyn>>,
) -> Matrix<f32, Const<N>, Const<1>, ArrayStorage<f32, N, 1>> {
    point_cloud.column_iter().fold(
        Matrix::from_data(ArrayStorage([[0f32; N]; 1])),
        |acc, it| acc + it,
    ) / point_cloud.len() as f32
}

pub fn get_point_cloud_centeroid_nd<const N: usize>(
    point_cloud: &LongDMatrix<N>,
    mean_point: Vector<f32, Const<N>, ArrayStorage<f32, N, 1>>,
) -> LongDMatrix<N> {
    Matrix::from_data(VecStorage::new(
        Const,
        Dyn(point_cloud.len()),
        point_cloud
            .column_iter()
            .flat_map(|current_point| (current_point - mean_point).data.0[0])
            .collect::<Vec<_>>(),
    ))
}

#[inline]
pub fn transform_points_nd<const N: usize, R: AbstractRotation<f32, N>>(
    points_a: &Matrix<f32, Const<N>, Dyn, VecStorage<f32, Const<N>, Dyn>>,
    transform: Isometry<f32, R, N>,
) -> Matrix<f32, Const<N>, Dyn, VecStorage<f32, Const<N>, Dyn>> {
    Matrix::from_data(VecStorage::new(
        Const,
        Dyn(points_a.ncols()),
        points_a
            .column_iter()
            .flat_map(|point_as_v| {
                transform
                    .transform_point(&Point::from_slice(point_as_v.as_slice()))
                    .coords
                    .data
                    .to_owned()
                    .0[0]
            })
            .collect::<Vec<_>>(),
    ))
}

#[cfg(test)]
pub mod test_utils {
    use crate::types::LongDMatrix;
    use nalgebra::{AbstractRotation, Const, Dyn, Isometry, Matrix, Point, VecStorage};
    use rand::Rng;

    pub fn generate_points<const N: usize, R>(
        isometry_matrix: Isometry<f32, R, N>,
    ) -> (LongDMatrix<N>, LongDMatrix<N>)
    where
        R: AbstractRotation<f32, N>,
    {
        let mut rng = rand::thread_rng();

        let points = Matrix::from_data(VecStorage::new(
            Const,
            Dyn(100),
            (0..100 * N)
                .map(|_| rng.gen_range(-15.0f32..=15.0f32))
                .collect::<Vec<_>>(),
        ));

        let points_transformed = Matrix::from_data(VecStorage::new(
            Const,
            Dyn(100),
            points
                .column_iter()
                .flat_map(|original_point| {
                    isometry_matrix
                        .transform_point(&Point::from_slice(original_point.data.into_slice()))
                        .coords
                        .data
                        .0[0]
                })
                .collect::<Vec<_>>(),
        ));

        (points, points_transformed)
    }
}
