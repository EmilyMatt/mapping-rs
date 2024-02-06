use crate::PI;
use nalgebra::{Point2};
///function to convert degree to radians
/// we use this func because coordinate is in degree, and we need to use trigonometric function
fn to_radians(degrees: f64) -> f64 {
    degrees * PI / 180.0
}
/// function to convert radians to degree
fn to_degrees(radians: f64) -> f64 {
    radians * 180.0 / PI
}
///calculate distance between point with use haversine formula
/// return distance in km
fn return_distance_between_points(p1: Point2<f64>, p2: Point2<f64>) -> f64 {
    //we need radius of earth
    let earth_radius_km = 6371.0;
    //extract coordinate from point
    let lat1 = p1.x;
    let lon1 = p1.y;
    let lat2 = p2.x;
    let lon2 = p2.y;
    //calc delta
    let dlat = to_radians(lat2 - lat1);
    let dlon = to_radians(lon2 - lon1);
    //convert coordinate to radians
    let lat1_radians = to_radians(lat1);
    let lat2_radians = to_radians(lat2);
    //apply haversine formula
    let a = (dlat / 2.0).sin().powi(2)
        + (dlon / 2.0).sin().powi(2) * lat1_radians.cos() * lat2_radians.cos();
    //use atan2 instead of arcsin(\sqrt a) due to numerical stability and precision
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    earth_radius_km * c
}
/// Returns the bearing from the first point to the second point.
fn return_bearing_between_points(p1: Point2<f64>, p2: Point2<f64>) -> f64 {
    let lat1 = p1.x;
    let lon1 = p1.y;
    let lat2 = p2.x;
    let lon2 = p2.y;
    let lat1_rad = to_radians(lat1);
    let lat2_rad = to_radians(lat2);
    let lon_diff_rad = to_radians(lon2 - lon1);
    let y = lon_diff_rad.sin() * lat2_rad.cos();
    let x = lat1_rad.cos() * lat2_rad.sin() - lat1_rad.sin() * lat2_rad.cos() * lon_diff_rad.cos();

    let bearing_rad = y.atan2(x);
    //convert to degree because navigation work with degree and return it
    (to_degrees(bearing_rad) + 360.0) % 360.0
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_distance() {
        let p1 = Point2::new(52.5200, 13.4050); // Berlin, Germany
        let p2 = Point2::new(48.8566, 2.3522); // Paris, France
        let distance = return_distance_between_points(p1, p2);
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
