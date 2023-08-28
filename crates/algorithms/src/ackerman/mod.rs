use nalgebra::{ComplexField, Isometry2, RealField, Translation2, UnitComplex};
use num_traits::AsPrimitive;
use std::iter::Sum;

/// This function uses an Ackerman steering model, and returns the expected change in translation and rotation for the timelapse
///
/// # Generics
/// [`T`]: One of [`f32`] or [`f64`]
///
/// # Arguments
/// * `velocity`: a [`T`] representing the current linear velocity of the vehicle.
/// * `steering_angle_in_rad`: a [`T`] representing the current angle(in radians!) of the wheels, measured to the relative positive X axis of the vehicle.
/// * `wheelbase`: a [`T`] representing the distance between the front and rear axles of the cars, must be in the same units as the `velocity`.
/// * `timelapse`: a [`T`] representing the time frame for which to calculate the transformation, usually measured since the last call to this function,
/// for better accuracy, call for more and smaller time frames.
///
/// # Returns
/// an [`Isometry2`], containing both the translation and rotation of the vehicle for the given time.
///
/// # Panics
/// In case [`wheelbase`] is smaller than [`T::default_epsilon`], to avoid division by 0.
pub fn get_transformation_from_wheels<T>(
    velocity: T,
    steering_angle_in_rad: T,
    wheelbase: T,
    timelapse: T,
) -> Result<Isometry2<T>, String>
where
    T: ComplexField + Copy + Default + RealField + Sum,
    usize: AsPrimitive<T>,
{
    if wheelbase.abs() < T::default_epsilon() {
        return Err("Wheelbase cannot be 0".to_string());
    } else if timelapse.abs() < T::default_epsilon() {
        return Err("Timelapse cannot be 0".to_string());
    } else if !(-T::pi()..=T::pi()).contains(&steering_angle_in_rad) {
        return Err("Steering angle should be between -PI and PI".to_string());
    }

    let rotation_at_t = (velocity * steering_angle_in_rad.sin() / wheelbase) * timelapse;
    let velocity_delta = (rotation_at_t + steering_angle_in_rad).sin_cos();

    Ok(Isometry2::from_parts(
        Translation2::new(velocity_delta.0 * timelapse, velocity_delta.1 * timelapse),
        UnitComplex::new(rotation_at_t),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_ackerman_steering() {
        assert!(get_transformation_from_wheels(2.0f32, 0.1, 0.0, 0.33).is_err());
        assert!(get_transformation_from_wheels(1.5f32, 0.7, 2.6, 0.0).is_err());
        assert!(get_transformation_from_wheels(1.5f32, 5.3, 2.6, 0.66).is_err());
    }
}
