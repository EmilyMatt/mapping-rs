use nalgebra::{ArrayStorage, Const, Isometry, Matrix};

pub type SameSizeMat<const N: usize> = Matrix<f32, Const<N>, Const<N>, ArrayStorage<f32, N, N>>;

#[derive(Debug)]
pub struct ICPSuccess<const N: usize, R> {
    pub(crate) transform: Isometry<f32, R, N>,
    pub(crate) mse: f32,
    pub(crate) iteration_num: usize,
}
