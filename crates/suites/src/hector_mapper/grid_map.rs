use crate::{array, Vec};
use nalgebra::{ComplexField, RealField};

#[derive(Clone, Debug, Default)]
struct Cell<T> {
    odds: T,
    update_frame_idx: u8,
}

pub struct GridMap<T, const N: usize> {
    data: Vec<Cell<T>>,
    strides: [usize; N],
    confidence_pos_factor: T,
    confidence_neg_factor: T,
    max_confidence: T,
}

impl<T: Copy + Default + RealField, const N: usize> GridMap<T, N> {
    pub fn create(
        dimensions: &[usize; N],
        occupied_factor: T,
        free_factor: T,
        max_confidence: T,
    ) -> Self {
        let strides: [usize; N] = array::from_fn(|idx| dimensions.iter().take(idx).product());
        Self {
            data: vec![Cell::default(); dimensions.iter().product()],
            strides,
            confidence_pos_factor: (occupied_factor - (T::one() / occupied_factor)).ln(),
            confidence_neg_factor: (free_factor - (T::one() / free_factor)).ln(),
            max_confidence,
        }
    }

    #[inline]
    fn get_cell_coord(&self, point: &nalgebra::Point<usize, N>) -> usize {
        point.coords.data.0[0]
            .iter()
            .enumerate()
            .map(|(idx, coord)| coord * self.strides[idx])
            .sum()
    }

    #[inline]
    pub fn update_taken(&mut self, point: &nalgebra::Point<usize, N>, update_frame_idx: u8) {
        let cell_idx = self.get_cell_coord(point);
        if let Some(cell) = self.data.get_mut(cell_idx) {
            if cell.odds < self.max_confidence {
                // Meaning this cell was already updated as
                if cell.update_frame_idx == update_frame_idx {
                    // Add the neg factor as well to revoke the reduction in confidence made by a bresenham plot
                    cell.odds += self.confidence_pos_factor + self.confidence_neg_factor;
                    return;
                }

                cell.update_frame_idx = update_frame_idx;
                cell.odds += self.confidence_pos_factor;
            }
        }
    }

    #[inline]
    pub fn update_free(&mut self, point: &nalgebra::Point<usize, N>, update_frame_idx: u8) {
        let cell_idx = self.get_cell_coord(point);
        if let Some(cell) = self.data.get_mut(cell_idx) {
            if cell.update_frame_idx != update_frame_idx {
                cell.odds -= self.confidence_neg_factor;
                cell.update_frame_idx = update_frame_idx;
            }
        }
    }

    pub fn get_cell_probability(&mut self, point: &nalgebra::Point<usize, N>) -> Option<T> {
        let cell_idx = self.get_cell_coord(point);
        self.data.get_mut(cell_idx).map(|cell| {
            let odds = <T as ComplexField>::exp(cell.odds);
            odds / (odds + T::one())
        })
    }
}
