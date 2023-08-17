// TODO: impl for f64, not as simple as it seems without duplicating, WIll prob convert to a trait and a macro

use nalgebra::{Matrix2xX, Vector2};

// This is like the little-sister algorithm for a full ICP algo
pub fn csm_2d(points_a: &Matrix2xX<f32>, points_b: &Matrix2xX<f32>) -> (Vector2<f32>, f32) {
    let mean_a = points_a.column_mean();
    let mean_b = points_b.column_mean();

    let mut covariance = 0.0;
    let mut variance = 0.0;
    for (a, b) in points_a.column_iter().zip(points_b.column_iter()) {
        let delta_a = a - mean_a;
        let delta_b = b - mean_b;

        covariance += delta_a.x * delta_b.y - delta_a.y * delta_b.x;
        variance += delta_a.dot(&delta_b);
    }

    let rotation = covariance.atan2(variance);
    let translation = mean_b - mean_a;

    (translation, rotation)
}

#[cfg(test)]
mod tests {
    use crate::{csm_2d, utils::test_utils::generate_points};
    use nalgebra::{Isometry2, Vector2};
    use std::f32;

    const TRANSLATION_ERROR_MARGIN: f32 = 0.15;
    const ROTATION_ERROR_MARGIN: f32 = 0.01;

    #[test]
    fn test_csm_2d() {
        let translation = Vector2::new(-0.8, 1.3);
        let rotation = f32::consts::PI / 1.5;
        let isom = Isometry2::new(translation, rotation);
        let (points, points_transformed) = generate_points(isom);

        let (translation_result, rotation_result) = csm_2d(&points, &points_transformed);

        let translation_error = translation_result.metric_distance(&translation);
        if translation_error > TRANSLATION_ERROR_MARGIN {
            panic!("Translation Result ({translation_result}) - ({translation}) bigger than error margin ({TRANSLATION_ERROR_MARGIN})");
        }

        let rotation_error = (rotation_result - rotation).abs();
        if rotation_error > ROTATION_ERROR_MARGIN {
            panic!("Rotation Result ({rotation_error})  - ({rotation}) bigger than error margin ({ROTATION_ERROR_MARGIN})");
        }
    }
}
