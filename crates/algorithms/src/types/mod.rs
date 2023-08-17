use nalgebra::{ArrayStorage, Const, Dyn, Isometry, Matrix, VecStorage};

pub type LongDMatrix<const N: usize> = Matrix<f32, Const<N>, Dyn, VecStorage<f32, Const<N>, Dyn>>;

#[derive(Debug)]
pub struct ConvergenceIntermediateStep<const N: usize> {
    pub translation: Matrix<f32, Const<N>, Const<1>, ArrayStorage<f32, N, 1>>,
    pub rotation: Matrix<f32, Const<N>, Const<N>, ArrayStorage<f32, N, N>>,
}

#[derive(Debug)]
pub enum ConvergenceResult<const N: usize> {
    Converged,
    NotConverged(ConvergenceIntermediateStep<N>),
}

#[derive(Debug)]
pub struct ICPSuccess<const N: usize, R> {
    pub(crate) transform: Isometry<f32, R, N>,
    pub(crate) mse: f32,
    pub(crate) iteration_num: usize,
}

#[derive(Debug)]
pub enum ICPResult<const N: usize, R> {
    Ok(ICPSuccess<N, R>),
    Err(String),
}
