use crate::Vec;

pub struct GridMap<T, const N: usize> {
    data: Vec<T>,
}

impl<T, const N: usize> GridMap<T, N> {
    pub fn create(dimensions: [usize; N]) -> Self {
        Self {
            data: vec![T::zero(); dimensions.into_iter().fold(0, |acc, it| acc * it)],
        }
    }
}
