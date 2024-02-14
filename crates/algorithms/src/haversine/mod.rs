use nalgebra::{Point2, RealField, Scalar};
use num_traits::Float;

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Haversine Distance", skip_all)
)]
fn calculate_haversine_distance<T>(point_a: Point2<T>, point_b: Point2<T>, sphere_radius: T) -> T
where
    T: Scalar + Float,
{
    let delta_lat = point_b.x - point_a.x;
    let delta_lon = point_b.y - point_a.y;
    let lat1_radians = point_a.x.to_radians();
    let lat2_radians = point_b.x.to_radians();
    let basic_haversine =
        haversine(delta_lat) + haversine(delta_lon) * lat1_radians.cos() * lat2_radians.cos();
    let inverse_haversine = (T::one() + T::one())
        * basic_haversine
            .sqrt()
            .atan2((T::one() - basic_haversine).sqrt());

    sphere_radius * inverse_haversine
}
#[inline]
fn haversine<T>(input: T) -> T
where
    T: Float,
{
    (input.to_radians() / (T::one() + T::one())).sin().powi(2)
}

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Bearing Between Points", skip_all)
)]
fn calculate_bearing_on_sphere<T>(point_a: Point2<T>, point_b: Point2<T>) -> T
where
    T: Scalar + Float + RealField,
{
    let lat1_rad = point_a.x.to_radians();
    let lat2_rad = point_b.x.to_radians();
    let lon_diff_rad = (point_b.y - point_a.y).to_radians();
    let x = <T as Float>::sin(lon_diff_rad) * <T as Float>::cos(lat2_rad);
    let y = (<T as Float>::cos(lat1_rad) * <T as Float>::sin(lat2_rad))
        - (<T as Float>::sin(lat1_rad)
            * <T as Float>::cos(lat2_rad)
            * <T as Float>::cos(lon_diff_rad));
    let bearing_rad = <T as Float>::atan2(x, y);
    ((bearing_rad + T::two_pi()) % T::two_pi()).to_degrees()
}

#[cfg(feature = "pregenerated")]

macro_rules! impl_haversine_formula {
    ($prec:expr, $prec_str:tt) => {
        ::paste::paste! {
            #[doc = "A " $prec_str "-precision implementation of a Haversine formula and calculate bearing between two points"]
            pub mod $prec {
                use super::*;
                #[doc = "Calculates the Haversine distance between two points on a sphere using " $prec_str "-precision floating-point arithmetic."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `point_a`: A [`Point2'] representing the first geographical point."]
                #[doc = "* `point_b`: A [`Point2`] representing the second geographical point."]
                #[doc = "* `sphere_radius`: The radius of the sphere, typically the Earth's radius in kilometers or miles."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "A 'float', the distance between `point_a` and `point_b` along the surface of the sphere, using the Haversine formula."]
                #[doc = "distance will return as kilometer"]
                pub fn [<calculate_haversine_distance_ $prec>](point_a: nalgebra::Point2<$prec>, point_b: nalgebra::Point2<$prec>, sphere_radius: $prec) -> $prec {

                    calculate_haversine_distance(point_a,point_b,sphere_radius)
                }
                #[doc = "Calculates the initial bearing (forward azimuth) from the first point to the second point."]
                #[doc = ""]
                #[doc = "This function computes the initial bearing, or forward azimuth, between two points on the surface"]
                #[doc = "of a sphere, assuming a spherical Earth model. The bearing is the direction one must travel"]
                #[doc = "from the first point to reach the second point, expressed as an angle from North (0 degrees)"]
                #[doc = "in a clockwise direction."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `point_a`: A [`Point2`] representing the starting geographical point (latitude and longitude)."]
                #[doc = "* `point_b`: A [`Point2`] representing the destination geographical point (latitude and longitude)."]
                #[doc = ""]
                #[doc = "# Type Parameters"]
                #[doc = "* `T`: The floating point type used for the calculation, which must implement `Copy`, `Float`, and `Debug`."]
                #[doc = "  This allows for a generic implementation that can work with different numeric types, such as `f32` or `f64`."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "* A value that representing the initial bearing from `point_a` to `point_b`, in degrees. The result is normalized"]
                #[doc = "  to a range of 0 to 360 degrees."]
                pub fn [<calculate_bearing_on_sphere_ $prec>](point_a: nalgebra::Point2<$prec>, point_b: nalgebra::Point2<$prec>) -> $prec {

                    calculate_bearing_on_sphere(point_a,point_b)}
                }


            }
        };}
#[cfg(feature = "pregenerated")]
impl_haversine_formula!(f32, single);
#[cfg(feature = "pregenerated")]
impl_haversine_formula!(f64, double);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distance() {
        let point_a = Point2::new(52.5200, 13.4050); // Berlin, Germany
        let point_b = Point2::new(48.8566, 2.3522); // Paris, France
        let earth_radius_km = 6371.0;
        let distance = calculate_haversine_distance(point_a, point_b, earth_radius_km);
        let expected_distance = 877.46; // Approximate distance in km
        assert!(
            (distance - expected_distance).abs() < 0.01,
            "Distance between Berlin and Paris should be roughly 877.46 km, found {}",
            distance
        );
    }

    #[test]
    fn test_bearing() {
        let point_a = Point2::new(39.099912, -94.581213); // Kansas City
        let point_b = Point2::new(38.627089, -90.200203); // St Louis
        let bearing = calculate_bearing_on_sphere(point_a, point_b);
        let expected_bearing = 96.51; // Approximate bearing in degrees
        assert!(
            (bearing - expected_bearing).abs() < 0.01,
            "Bearing from Kansas City to St Louis should be roughly 96.51 degrees, found {}",
            bearing
        );
    }
}
