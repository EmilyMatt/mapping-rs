use core::fmt::Debug;
use nalgebra::Point2;
use num_traits::Float;

///calculate distance between point with use haversine formula
/// return distance in km
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Haversine Data", skip_all)
)]
fn calculate_haversine_distance<T>(p1: Point2<T>, p2: Point2<T>, sphere_radius: T) -> T
where
    T: Copy + Float + Debug + 'static,
{
    //define two = 2 to support generic
    let two = T::from(2.0).unwrap();
    //calc delta
    let dlat = (p2.x - p1.x).to_radians();
    let dlon = (p2.y - p1.y).to_radians();
    //convert coordinate to radians
    let lat1_radians = p1.x.to_radians();
    let lat2_radians = p2.x.to_radians();
    //apply haversine formula
    let a = (dlat / two).sin().powi(2)
        + (dlon / two).sin().powi(2) * lat1_radians.cos() * lat2_radians.cos();
    //use atan2 instead of arcsin(\sqrt a) due to numerical stability and precision
    let c = two * a.sqrt().atan2((T::one() - a).sqrt());

    sphere_radius * c
}
/// Returns the bearing from the first point to the second point.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Calculate Bearing Between Points", skip_all)
)]
fn return_bearing_between_points<T>(p1: Point2<T>, p2: Point2<T>) -> T
where
    T: Copy + Float + Debug + 'static,
{
    let lat1_rad = p1.x.to_radians();
    let lat2_rad = p2.x.to_radians();
    let lon_diff_rad = (p2.y - p1.y).to_radians();
    let y = lon_diff_rad.sin() * lat2_rad.cos();
    let x = lat1_rad.cos() * lat2_rad.sin() - lat1_rad.sin() * lat2_rad.cos() * lon_diff_rad.cos();

    let bearing_rad = y.atan2(x);
    //convert to degree because navigation work with degree and return it
    let circle_angle = T::from(360.0).unwrap();
    (bearing_rad.to_degrees() + circle_angle) % circle_angle
}
#[cfg(feature = "pregenerated")]
macro_rules! impl_p_i_p_algorithm {
    ($prec:expr, $prec_str:tt) => {
        ::paste::paste! {
            #[doc = "A " $prec_str "-precision implementation of a Haversine formula and calculate bearing between two points"]
            pub mod $prec {
                use super::*;
                #[doc = "Calculates the Haversine distance between two points on a sphere using " $prec_str "-precision floating-point arithmetic."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `p1`: A [`Point2'] representing the first geographical point."]
                #[doc = "* `p2`: A [`Point2`] representing the second geographical point."]
                #[doc = "* `sphere_radius`: The radius of the sphere, typically the Earth's radius in kilometers or miles."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "A 'float', the distance between `p1` and `p2` along the surface of the sphere, using the Haversine formula."]
                pub fn [<calculate_haversine_distance_ $prec>](p1: nalgebra::Point2<$prec>, p2: nalgebra::Point2<$prec>, sphere_radius: $prec) -> $prec {

                    calculate_haversine_distance(p1,p2,sphere_radius)
                }
                #[doc = "Calculates the initial bearing (forward azimuth) from the first point to the second point."]
                #[doc = ""]
                #[doc = "This function computes the initial bearing, or forward azimuth, between two points on the surface"]
                #[doc = "of a sphere, assuming a spherical Earth model. The bearing is the direction one must travel"]
                #[doc = "from the first point to reach the second point, expressed as an angle from North (0 degrees)"]
                #[doc = "in a clockwise direction."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `p1`: A [`Point2`] representing the starting geographical point (latitude and longitude)."]
                #[doc = "* `p2`: A [`Point2`] representing the destination geographical point (latitude and longitude)."]
                #[doc = ""]
                #[doc = "# Type Parameters"]
                #[doc = "* `T`: The floating point type used for the calculation, which must implement `Copy`, `Float`, and `Debug`."]
                #[doc = "  This allows for a generic implementation that can work with different numeric types, such as `f32` or `f64`."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "* A value that representing the initial bearing from `p1` to `p2`, in degrees. The result is normalized"]
                #[doc = "  to a range of 0 to 360 degrees."]
                pub fn [<return_bearing_between_points_ $prec>](p1: nalgebra::Point2<$prec>, p2: nalgebra::Point2<$prec>) -> $prec {

                    return_bearing_between_points(p1,p2)}
                }


            }
        };}
#[cfg(feature = "pregenerated")]
impl_p_i_p_algorithm!(f32, single);
#[cfg(feature = "pregenerated")]
impl_p_i_p_algorithm!(f64, double);
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distance() {
        let p1 = Point2::new(52.5200, 13.4050); // Berlin, Germany
        let p2 = Point2::new(48.8566, 2.3522); // Paris, France
        let earth_radius_km = 6371.0;
        let distance = calculate_haversine_distance(p1, p2, earth_radius_km);
        let expected_distance = 878.0; // Approximate distance in km
        assert!(
            (distance - expected_distance).abs() < 10.0,
            "Distance between Berlin and Paris should be roughly 878 km, found {}",
            distance
        );
    }

    #[test]
    fn test_bearing() {
        let p1 = Point2::new(52.5200, 13.4050); // Berlin, Germany
        let p2 = Point2::new(48.8566, 2.3522); // Paris, France
        let bearing = return_bearing_between_points(p1, p2);
        let expected_bearing = 240.0; // Approximate bearing in degrees
        assert!(
            (bearing - expected_bearing).abs() < 10.0,
            "Bearing from Berlin to Paris should be roughly 240 degrees, found {}",
            bearing
        );
    }
}
