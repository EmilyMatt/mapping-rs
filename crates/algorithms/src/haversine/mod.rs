// SPDX-License-Identifier: MIT
/*
 * Copyright (c) [2023 - Present] Emily Matheys <emilymatt96@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

use nalgebra::{Point2, RealField, Scalar};
use num_traits::Float;

#[inline]
fn half_angle_sine_squared<T>(input: T) -> T
where
    T: Float,
{
    (input.to_radians() / (T::one() + T::one())).sin().powi(2)
}

/// Calculates the Haversine distance between two points on a sphere using floating-point arithmetic.
///
/// # Arguments
/// * `point_a`: A [`Point2'] representing the first geographical point.
/// * `point_b`: A [`Point2`] representing the second geographical point.
/// * `sphere_radius`: A `T` representing the radius of the sphere, typically the Earth's radius in kilometers or miles.
///
/// # Generics
/// `T`: Either an [`prim@f32`] or [`prim@f64`]
///
/// # Returns
/// A 'T', the distance between `point_a` and `point_b` along the surface of the sphere, using the Haversine formula.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Haversine Distance", skip_all)
)]
pub fn calculate_haversine_distance<T>(
    point_a: Point2<T>,
    point_b: Point2<T>,
    sphere_radius: T,
) -> T
where
    T: Scalar + Float,
{
    let delta_lat = point_b.x - point_a.x;
    let delta_lon = point_b.y - point_a.y;

    let lat1_radians = point_a.x.to_radians();
    let lat2_radians = point_b.x.to_radians();

    let basic_haversine = half_angle_sine_squared(delta_lat)
        + half_angle_sine_squared(delta_lon) * lat1_radians.cos() * lat2_radians.cos();

    let inverse_haversine = (T::one() + T::one())
        * basic_haversine
            .sqrt()
            .atan2((T::one() - basic_haversine).sqrt());

    sphere_radius * inverse_haversine
}

/// Calculates the initial bearing (forward azimuth) from the first point to the second point.
///
/// This function computes the initial bearing, or forward azimuth, between two points on the surface
/// of a sphere, assuming a spherical model. The bearing is the direction one must travel
/// from the first point to reach the second point, expressed as an angle from North (0 radians)
/// in a clockwise direction.
///
/// # Arguments
/// * `point_a`: A [`Point2`] representing the starting geographical point (latitude and longitude).
/// * `point_b`: A [`Point2`] representing the destination geographical point (latitude and longitude).
///
/// # Generics
/// `T`: Either an [`prim@f32`] or [`prim@f64`]
///
/// # Returns
/// * A value that representing the initial bearing from `point_a` to `point_b`, in radians.
/// The result is normalized to a range of 0 to 2 PI radians.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Bearing Between Points", skip_all)
)]
pub fn calculate_sphere_bearing<T>(point_a: Point2<T>, point_b: Point2<T>) -> T
where
    T: Scalar + Float + RealField,
{
    let lat1_rad = point_a.x.to_radians();
    let lat2_rad = point_b.x.to_radians();

    let lon_delta_radians = (point_b.y - point_a.y).to_radians();

    let x = <T as Float>::sin(lon_delta_radians) * <T as Float>::cos(lat2_rad);
    let y = (<T as Float>::cos(lat1_rad) * <T as Float>::sin(lat2_rad))
        - (<T as Float>::sin(lat1_rad)
            * <T as Float>::cos(lat2_rad)
            * <T as Float>::cos(lon_delta_radians));

    (<T as Float>::atan2(x, y) + T::two_pi()) % T::two_pi()
}

#[cfg(feature = "pregenerated")]
macro_rules! impl_haversine_formula {
    ($prec:expr, doc $doc:tt) => {
        ::paste::paste! {
            #[doc = "A " $doc "-precision implementation of the Haversine formula and adjacent utilities"]
            pub mod [<$doc _precision>] {
                #[doc = "Calculates the Haversine distance between two points on a sphere using " $doc "-precision floating-point arithmetic."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `point_a`: A [`Point2'](nalgebra::Point2) representing the first geographical point."]
                #[doc = "* `point_b`: A [`Point2`](nalgebra::Point2) representing the second geographical point."]
                #[doc = "* `sphere_radius`: The radius of the sphere, typically the Earth's radius in kilometers or miles."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "A '" $prec "', the distance between `point_a` and `point_b` along the surface of the sphere, using the Haversine formula."]
                pub fn calculate_haversine_distance(point_a: nalgebra::Point2<$prec>, point_b: nalgebra::Point2<$prec>, sphere_radius: $prec) -> $prec {
                    super::calculate_haversine_distance(point_a,point_b,sphere_radius)
                }

                #[doc = "Calculates the initial bearing (forward azimuth) from the first point to the second point."]
                #[doc = ""]
                #[doc = "This function computes the initial bearing, or forward azimuth, between two points on the surface"]
                #[doc = "of a sphere, assuming a spherical model. The bearing is the direction one must travel"]
                #[doc = "from the first point to reach the second point, expressed as an angle from North (0 radians)"]
                #[doc = "in a clockwise direction."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `point_a`: A [`Point2`](nalgebra::Point2) representing the starting geographical point (latitude and longitude)."]
                #[doc = "* `point_b`: A [`Point2`](nalgebra::Point2) representing the destination geographical point (latitude and longitude)."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "* A value that representing the initial bearing from `point_a` to `point_b`, in radians. The result is normalized"]
                #[doc = "  to a range of 0 to 2 PI radians."]
                pub fn calculate_sphere_bearing(point_a: nalgebra::Point2<$prec>, point_b: nalgebra::Point2<$prec>) -> $prec {
                    super::calculate_sphere_bearing(point_a,point_b)}
                }
            }
        }
}

#[cfg(feature = "pregenerated")]
impl_haversine_formula!(f32, doc single);
#[cfg(feature = "pregenerated")]
impl_haversine_formula!(f64, doc double);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distance() {
        let point_a = Point2::new(52.5200, 13.4050); // Berlin, Germany
        let point_b = Point2::new(48.8566, 2.3522); // Paris, France

        let earth_radius_km = 6371.0;
        let distance =
            double_precision::calculate_haversine_distance(point_a, point_b, earth_radius_km);
        let expected_distance = 877.46; // Approximate distance in km
        assert!(
            (distance - expected_distance).abs() < 0.01,
            "Distance between Berlin and Paris should be roughly 877.46 km, found {}",
            distance
        );
    }

    #[test]
    fn test_bearing() {
        let point_a = Point2::new(39.099_91, -94.581213); // Kansas City
        let point_b = Point2::new(38.627_09, -90.200_2); // St Louis

        let bearing = single_precision::calculate_sphere_bearing(point_a, point_b);
        let expected_bearing = 96.51; // Approximate bearing in degrees
        assert!(
            (bearing - expected_bearing.to_radians()).abs() < 0.01,
            "Bearing from Kansas City to St Louis should be roughly 96.51 degrees, found {}",
            bearing.to_degrees()
        );
    }
}
