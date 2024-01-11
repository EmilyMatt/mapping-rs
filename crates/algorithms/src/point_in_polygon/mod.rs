use crate::{mem, utils::calculate_polygon_extents, Vec};
use nalgebra::{ComplexField, Point, Point2, RealField, SimdComplexField, SimdRealField, Vector2};

#[inline]
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Does Ray Intersect Polygon", skip_all)
)]
fn does_ray_intersect<T>(point: &Vector2<T>, mut vertex1: Point2<T>, mut vertex2: Point2<T>) -> bool
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    // Reverse direction, we assume a rising line function, the simplest solution to avoid a different one is simply to reverse the order of vertices.
    if vertex1.y > vertex2.y {
        mem::swap(&mut vertex1, &mut vertex2);
    }

    // Handle case where difference is too small and will cause issues, by simply adding an epsilon ;)
    if point.y == vertex1.y || point.y == vertex2.y {
        return does_ray_intersect(
            &Vector2::from([point.x, point.y + T::default_epsilon()]),
            vertex1,
            vertex2,
        );
    }

    // Check if out of extents, no need to continue checking then.
    if point.y > vertex2.y || point.y < vertex1.y || point.x > vertex1.x.max(vertex2.x) {
        return false;
    }

    if point.x < vertex1.x.min(vertex2.x) {
        return true;
    }

    (((vertex2.x - vertex1.x) * (point.y - vertex1.y))
        - ((vertex2.y - vertex1.y) * (point.x - vertex1.x)))
        < T::zero()
}

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Get Point's Number Of Intersections With Polygon", skip_all)
)]
fn get_point_intersections_with_polygon<T>(
    point: &Point2<T>,
    polygon: &[Point2<T>],
) -> Vec<Point2<T>>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    let polygon_len = polygon.len();
    (0..polygon_len)
        .filter_map(|current_vertex_idx| {
            let current_vertex = polygon[current_vertex_idx];
            let next_vertex = polygon[(current_vertex_idx + 1) % polygon_len];

            does_ray_intersect(&point.coords, current_vertex, next_vertex).then_some(*point)
        })
        .collect() // Only returns the intersections as Some()
}

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Is Point In Polygon", skip_all)
)]
fn is_single_point_in_polygon<T>(point: &Point2<T>, polygon: &[Point2<T>]) -> bool
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    let len: usize = get_point_intersections_with_polygon(point, polygon).len();
    len % 2 == 1 // If the number of intersections is odd - we didn't exit the polygon, and are therefor in it.
}

#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Are Points In Polygon", skip_all)
)]
fn are_multiple_points_in_polygon<T>(points: &[Point<T, 2>], polygon: &[Point<T, 2>]) -> Vec<bool>
where
    T: ComplexField + SimdComplexField + RealField + SimdRealField + Default + Copy,
{
    let polygon_extents = calculate_polygon_extents(polygon);

    points
        .iter()
        .map(|current_point| {
            // Verify that each coordinate is within the bounds of the polygon, will save a lot of computational load for large polygons
            polygon_extents
                .iter()
                .zip(current_point.coords.iter())
                .fold(
                    true,
                    |is_in_extents, (extent_for_dimension, vertex_coord)| {
                        is_in_extents && extent_for_dimension.contains(vertex_coord)
                    },
                )
                && is_single_point_in_polygon(current_point, polygon)
        })
        .collect()
}

macro_rules! impl_p_i_p_algorithm {
    ($prec:expr, $prec_str:tt) => {
        ::paste::paste! {
            #[doc = "A " $prec_str "-precision implementation of a point-in-polygon algorithm."]
            pub mod $prec {
                use super::*;

                #[doc = "Check whether a specified ray(with origin 0) collides with another ray."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `ray`: A reference to a [`Vector2`], representing a ray, whose origin is [0.0, 0.0]."]
                #[doc = "* `vertex1`: A [`Point2`] representing the first vertex of the other ray."]
                #[doc = "* `vertex2`: A [`Point2`] representing the second vertex of the other ray."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "A `bool`, specifying whether the rays intersect"]
                pub fn [<does_ray_intersect_$prec>](ray: &Vector2<$prec>, vertex1: Point2<$prec>, vertex2: Point2<$prec>) -> bool {
                    does_ray_intersect(ray, vertex1, vertex2)
                }

                #[doc = "Get the number of intersections of this point(representing a vector), with this polygon."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `point`: A reference to a [`Point2`]"]
                #[doc = "* `polygon`: A slice of [`Point2`]s representing the vertices."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "A usize, representing the number of intersections."]
                pub fn [<get_point_intersections_with_polygon_$prec>](
                        point: &Point2<$prec>,
                        polygon: &[Point2<$prec>],
                    ) -> Vec<Point2<$prec>> {
                    get_point_intersections_with_polygon(point, polygon)
                }

                #[doc = "Check if the provided point is within the provided polygon."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `point`: A reference to a [`Point2`]."]
                #[doc = "* `polygon`: A slice of [`Point2`]s representing the vertices."]
                #[doc = "# Returns"]
                #[doc = "A boolean value, specifying if the point is within the polygon."]
                pub fn [<is_single_point_in_polygon_$prec>](point: &Point2<$prec>, polygon: &[Point2<$prec>]) -> bool {
                    is_single_point_in_polygon(point, polygon)
                }

                #[doc = "This function will run the [`is_single_point_in_polygon_" $prec "`] for each on of the points given, and the provided polygon,"]
                #[doc = "But pre-calculates the polygon extents to reduce workloads for larger datasets, please profile this for you specific use-case."]
                #[doc = ""]
                #[doc = "# Arguments"]
                #[doc = "* `points`: A slice of [`Point`]."]
                #[doc = "* `polygon`: A slice of [`Point`]s, representing the vertices."]
                #[doc = ""]
                #[doc = "# Returns"]
                #[doc = "A [`Vec`] of booleans, with the same size as `points`, containing the result for each point."]
                pub fn [<are_multiple_points_in_polygon_$prec>](
                    points: &[Point2<$prec>],
                    polygon: &[Point2<$prec>],
                ) -> Vec<bool> {
                    are_multiple_points_in_polygon(points, polygon)
                }
            }
        }
    };
}

impl_p_i_p_algorithm!(f32, single);
impl_p_i_p_algorithm!(f64, double);

#[cfg(test)]
mod tests {
    use crate::Vec;
    use nalgebra::{Point2, Vector2};

    #[test]
    fn test_does_ray_intersect() {
        let point_a = Vector2::new(4.0, -3.0);
        let vertex_a1 = Point2::new(5.0, 0.0);
        let vertex_a2 = Point2::new(1.0, -4.0);
        assert!(super::f32::does_ray_intersect_f32(
            &point_a, vertex_a1, vertex_a2
        ));

        let point_b = Vector2::new(-4.0, 4.0);
        let vertex_b1 = Point2::new(0.0, 0.0);
        let vertex_b2 = Point2::new(1.0, 5.0);
        assert!(super::f32::does_ray_intersect_f32(
            &point_b, vertex_b1, vertex_b2
        ));
    }

    // These following tests pretty much test all the functions:
    #[test]
    fn test_multiple_points_in_polygon_clockwise() {
        // A simple square polygon with vertices (0,0), (0,1), (1,1), (1,0)
        let polygon = &[
            Point2::from([0.0, 0.0]),
            Point2::from([0.0, 1.0]),
            Point2::from([1.0, 1.0]),
            Point2::from([1.0, 0.0]),
        ];

        // One point inside the polygon and one outside.
        let points = &[
            Point2::from([0.5, 0.5]), // Inside
            Point2::from([1.5, 1.5]), // Outside
        ];

        let result = super::f32::are_multiple_points_in_polygon_f32(points, polygon);

        // Expecting [true, false] since the first point is inside and the second is outside.
        assert_eq!(result, Vec::from([true, false]));
    }

    #[test]
    fn test_multiple_points_in_polygon_counter_clockwise() {
        // A simple square polygon with vertices (0,0), (0,1), (1,1), (1,0)
        let polygon = &[
            Point2::from([0.0, 0.0]),
            Point2::from([0.0, 1.0]),
            Point2::from([1.0, 1.0]),
            Point2::from([1.0, 0.0]),
        ];

        // One point inside the polygon and one outside.
        let points = &[
            Point2::from([0.5, 0.5]), // Inside
            Point2::from([1.5, 1.5]), // Outside
        ];

        let result = super::f32::are_multiple_points_in_polygon_f32(points, polygon);

        // Expecting [true, false] since the first point is inside and the second is outside.
        assert_eq!(result, Vec::from([true, false]));
    }
}
