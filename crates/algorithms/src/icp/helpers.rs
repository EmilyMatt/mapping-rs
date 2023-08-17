use crate::types::LongDMatrix;
use nalgebra::{Const, Dyn, Matrix, VecStorage};

#[inline]
pub fn get_closest_points<const N: usize>(
    points_a: &LongDMatrix<N>,
    points_b: &LongDMatrix<N>,
) -> LongDMatrix<N> {
    Matrix::from_data(VecStorage::new(
        Const,
        Dyn(points_a.ncols()),
        points_a
            .column_iter()
            .flat_map(|point_a| {
                let mut min_distance = f32::INFINITY;
                let mut closest_point_index = 0;
                for (idx, point_b) in points_b.column_iter().enumerate() {
                    let distance = point_a.metric_distance(&point_b);
                    if distance < min_distance {
                        min_distance = distance;
                        closest_point_index = idx;
                    }
                }
                points_a.column(closest_point_index).as_slice().to_owned()
            })
            .collect::<Vec<_>>(),
    ))
}

#[inline]
pub fn calculate_mse<const N: usize>(points_a: &LongDMatrix<N>, points_b: &LongDMatrix<N>) -> f32 {
    points_a
        .column_iter()
        .zip(points_b.column_iter())
        .map(|(point_a, point_b)| point_a.metric_distance(&point_b).powi(2))
        .sum::<f32>()
        / points_b.ncols() as f32
}
