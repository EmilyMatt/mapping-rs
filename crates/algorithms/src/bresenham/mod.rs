use crate::{array, Vec};
use nalgebra::{ComplexField, Point, Scalar};
use num_traits::{AsPrimitive, Float};

fn bresenham_implementation<F, T, const N: usize>(
    start_point: Point<F, N>,
    end_point: Point<F, N>,
) -> Vec<Point<T, N>>
where
    F: ComplexField + Float + AsPrimitive<usize> + AsPrimitive<T>,
    T: Scalar + Copy,
{
    let deltas: [F; N] = array::from_fn(|idx| <F as Float>::abs(end_point[idx] - start_point[idx]));
    let steps: [F; N] = array::from_fn(|idx| {
        if end_point[idx] > start_point[idx] {
            F::one()
        } else {
            -F::one()
        }
    });
    let primary_axis = deltas
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .unwrap()
        .0;

    let mut current_point = start_point;
    let mut errors = Vec::from([F::zero(); N]);
    let mut points = Vec::with_capacity(<F as AsPrimitive<usize>>::as_(
        deltas[primary_axis] + F::one(),
    ));
    while <F as Float>::abs(current_point[primary_axis] - end_point[primary_axis])
        > Float::epsilon()
    {
        points.push(current_point.map(|element| element.as_()));

        for axis in 0..N {
            if axis == primary_axis {
                continue;
            }

            errors[axis] += deltas[axis] / deltas[primary_axis];

            if errors[axis] >= F::one() - (F::one() / F::from_usize(N + 1).unwrap()) {
                current_point[axis] += steps[axis];
                errors[axis] -= F::one();
            }
        }

        current_point[primary_axis] += steps[primary_axis];
    }

    points.push(end_point.map(|element| element.as_()));
    points
}

/// This is a free-form version of the bresenham line-drawing algorithm, allowing for any input, any output,
/// and N dimensions, under the constraints of the function.
///
/// # Arguments
/// * `start_point`: A [`Point`] of `I` type and `N` dimensions, representing the starting point of the line.
/// * `end_point`: A [`Point`] of `I` type and `N` dimensions, representing the ending point of the line.
///
/// # Returns
/// A [`Vec`] of [`Point`]s, representing the drawn line, including the starting point and ending point.
///
/// NOTE: The returned [`Vec`] will always go from the starting point to the ending point, regardless of direction in axis.
#[cfg_attr(
    feature = "tracing",
    tracing::instrument("Plot Bresenham Line", skip_all)
)]
pub fn plot_bresenham_line<I, F, T, const N: usize>(
    start_point: Point<I, N>,
    end_point: Point<I, N>,
) -> Vec<Point<T, N>>
where
    I: AsPrimitive<F> + Scalar,
    F: ComplexField + Float + AsPrimitive<usize> + AsPrimitive<T>,
    T: Scalar + Copy,
{
    bresenham_implementation(
        start_point.map(|element| <F as Float>::floor(element.as_())),
        end_point.map(|element| <F as Float>::floor(element.as_())),
    )
}

macro_rules! impl_bresenham_algorithm {
    ($precision:expr, $nd:expr, $out:expr) => {
        ::paste::paste! {
            #[doc = "Bresenham line drawing algorithm in " $nd "D space."]
            #[doc = "# Arguments"]
            #[doc = "* `start_point`: A [`Point<" $precision ", " $nd ">`], representing the starting point of the line."]
            #[doc = "* `end_point`: A [`Point<" $precision ", " $nd ">`], representing the ending point of the line."]
            #[doc = ""]
            #[doc = "# Returns"]
            #[doc = "A [`Vec`] of [`Point`]s, representing the drawn line, including the starting point and ending point."]
            #[doc = ""]
            #[doc = "NOTE: The returned [`Vec`] will always go from the starting point to the ending point, regardless of direction in axis."]
            pub fn [<plot_bresenham_line_$nd d_returns_$out>](start_point: Point<$precision, $nd>, end_point: Point<$precision, $nd>) -> Vec<Point<$out, $nd>> {
                    plot_bresenham_line::<$precision, $precision, $out, $nd>(start_point, end_point)
            }
        }
    };
}

macro_rules! blanket_impl_bresenham_for_d {
    ($prec:expr, $nd:expr) => {
        impl_bresenham_algorithm!($prec, $nd, i32);
        impl_bresenham_algorithm!($prec, $nd, i64);
        impl_bresenham_algorithm!($prec, $nd, isize);
        impl_bresenham_algorithm!($prec, $nd, u32);
        impl_bresenham_algorithm!($prec, $nd, u64);
        impl_bresenham_algorithm!($prec, $nd, usize);
        impl_bresenham_algorithm!($prec, $nd, $prec);
    };
}

#[cfg(feature = "pregenerated")]
/// A single-precision implementation of a bresenham line-drawing algorithm.
pub mod f32 {
    use super::*;
    blanket_impl_bresenham_for_d!(f32, 2);
    blanket_impl_bresenham_for_d!(f32, 3);
}

#[cfg(feature = "pregenerated")]
/// A single-precision implementation of a bresenham line-drawing algorithm.
pub mod f64 {
    use super::*;
    blanket_impl_bresenham_for_d!(f64, 2);
    blanket_impl_bresenham_for_d!(f64, 3);
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point2, Point3};

    #[test]
    fn test_plot_bresenham_line_2d_nonsteep_pos() {
        let res = f32::plot_bresenham_line_2d_returns_isize(
            Point2::new(0.0f32, 0.0f32),
            Point2::new(10.0f32, 3.0f32),
        );
        assert_eq!(
            res,
            Vec::<Point2<isize>>::from([
                Point2::new(0, 0),
                Point2::new(1, 0),
                Point2::new(2, 0),
                Point2::new(3, 1),
                Point2::new(4, 1),
                Point2::new(5, 1),
                Point2::new(6, 2),
                Point2::new(7, 2),
                Point2::new(8, 2),
                Point2::new(9, 3),
                Point2::new(10, 3),
            ])
        );
    }

    #[test]
    fn test_plot_bresenham_line_2d_steep_pos() {
        let res = f32::plot_bresenham_line_2d_returns_isize(
            Point2::new(0.0f32, 0.0f32),
            Point2::new(3.0f32, 10.0f32),
        );
        assert_eq!(
            res,
            Vec::<Point2<isize>>::from([
                Point2::new(0, 0),
                Point2::new(0, 1),
                Point2::new(0, 2),
                Point2::new(1, 3),
                Point2::new(1, 4),
                Point2::new(1, 5),
                Point2::new(2, 6),
                Point2::new(2, 7),
                Point2::new(2, 8),
                Point2::new(3, 9),
                Point2::new(3, 10),
            ])
        );
    }

    #[test]
    fn test_plot_bresenham_line_2d_nonsteep_neg() {
        let res = f32::plot_bresenham_line_2d_returns_isize(
            Point2::new(0.0f32, 0.0f32),
            Point2::new(-10.0f32, -3.0f32),
        );
        assert_eq!(
            res,
            Vec::<Point2<isize>>::from([
                Point2::new(0, 0),
                Point2::new(-1, 0),
                Point2::new(-2, 0),
                Point2::new(-3, -1),
                Point2::new(-4, -1),
                Point2::new(-5, -1),
                Point2::new(-6, -2),
                Point2::new(-7, -2),
                Point2::new(-8, -2),
                Point2::new(-9, -3),
                Point2::new(-10, -3),
            ])
        );
    }

    #[test]
    fn test_plot_bresenham_line_2d_steep_neg() {
        let res = f32::plot_bresenham_line_2d_returns_isize(
            Point2::new(0.0f32, 0.0f32),
            Point2::new(-3.0f32, -10.0f32),
        );
        assert_eq!(
            res,
            Vec::<Point2<isize>>::from([
                Point2::new(0, 0),
                Point2::new(0, -1),
                Point2::new(0, -2),
                Point2::new(-1, -3),
                Point2::new(-1, -4),
                Point2::new(-1, -5),
                Point2::new(-2, -6),
                Point2::new(-2, -7),
                Point2::new(-2, -8),
                Point2::new(-3, -9),
                Point2::new(-3, -10),
            ])
        );
    }

    #[test]
    fn test_plot_bresenham_line_3d_x() {
        let res = f32::plot_bresenham_line_3d_returns_isize(
            Point3::new(0.0f32, 0.0f32, 0.0f32),
            Point3::new(-3.0f32, -10.0f32, 7.0f32),
        );
        assert_eq!(
            res,
            Vec::<Point3<isize>>::from([
                Point3::new(0, 0, 0),
                Point3::new(0, -1, 0),
                Point3::new(0, -2, 1),
                Point3::new(-1, -3, 2),
                Point3::new(-1, -4, 3),
                Point3::new(-1, -5, 3),
                Point3::new(-2, -6, 4),
                Point3::new(-2, -7, 5),
                Point3::new(-2, -8, 5),
                Point3::new(-2, -9, 6),
                Point3::new(-3, -10, 7),
            ])
        );
    }
}
