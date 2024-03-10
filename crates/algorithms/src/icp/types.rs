use crate::Debug;
use nalgebra::{AbstractRotation, Isometry, Scalar};
use num_traits::AsPrimitive;

/// Contains the resulting transform, the resulting Mean Squared Error, and the number of iterations taken for a successful ICP convergence.
#[derive(Debug)]
pub struct ICPSuccess<T: Scalar, R: AbstractRotation<T, N>, const N: usize> {
    /// An isometric matrix, containing the translation and rotation between the point sets.
    /// In 2D space, its rotation component would be a [`UnitComplex`](nalgebra::UnitComplex), in 3D space it would be a [`UnitQuaternion`](nalgebra::UnitQuaternion).
    pub transform: Isometry<T, R, N>,
    /// Mean Squared Error, this is the distances between each point in `points_a` and its corresponding point in `points_b`,
    /// This can be used to determine whether the ICP converged correctly, or simply on its local minimum.
    pub mse: T,
    /// The amount of iterations passed until convergence.
    pub iteration_num: usize,
}

/// A struct specifying configuration options for an ICP algorithm.
#[derive(Clone, Debug)]
pub struct ICPConfiguration<T> {
    /// Whether to use a KDTree structure to find nearest neighbours, becomes increasingly effective with point cloud growth.
    pub(crate) use_kd_tree: bool,
    /// The amount of iterations before giving up and exiting the algorithm.
    pub(crate) max_iterations: usize,
    /// When provided, the algorithm will consider itself converged when the MSE is smaller than the given value, without any more iterations.
    pub(crate) mse_absolute_threshold: Option<T>,
    /// This will specify the interval between iteration MSE's than when reached, will declare ICP convergence.
    pub(crate) mse_interval_threshold: T,
}

impl<T: 'static + Copy> ICPConfiguration<T>
where
    f32: AsPrimitive<T>,
{
    /// Returns a builder for the configuration struct.
    ///
    /// # Returns
    /// An [`ICPConfigurationBuilder`].
    pub fn builder() -> ICPConfigurationBuilder<T> {
        ICPConfigurationBuilder {
            _internal: ICPConfiguration {
                use_kd_tree: false,
                max_iterations: 20,
                mse_absolute_threshold: None,
                mse_interval_threshold: 0.01.as_(),
            },
        }
    }
}

/// A Builder-pattern struct for safely constructing an [`ICPConfiguration`] struct.
#[derive(Clone, Debug)]
pub struct ICPConfigurationBuilder<T> {
    _internal: ICPConfiguration<T>,
}

impl<T: Copy> ICPConfigurationBuilder<T> {
    /// Enables usage of a KD Tree structure to find nearest neighbours, or use a native On^2 search,
    /// a KD Tree becomes increasingly effective with point cloud growth.
    ///
    /// # Arguments
    /// * `use_kd_tree`: Whether to use a KD Tree search method.
    ///
    /// # Returns
    /// A copy of self, with the updated parameters
    pub fn with_kd_tree(&self, use_kd_tree: bool) -> Self {
        Self {
            _internal: ICPConfiguration {
                use_kd_tree,
                ..self._internal
            },
        }
    }

    /// The amount of iterations before giving up and exiting the algorithm.
    ///
    /// # Arguments
    /// * `max_iterations`: The maximum number of iterations to allow.
    ///
    /// # Returns
    /// A copy of self, with the updated parameters
    pub fn with_max_iterations(&self, max_iterations: usize) -> Self {
        Self {
            _internal: ICPConfiguration {
                max_iterations,
                ..self._internal
            },
        }
    }

    /// When provided, the algorithm will consider itself converged when the MSE is smaller than the given value, without any more iterations.
    ///
    /// # Arguments
    /// * `mse_absolute_threshold`: If is [`Some`], sets the minimum accepted MSE difference, that will return a convergence.
    ///
    /// # Returns
    /// A copy of self, with the updated parameters
    pub fn with_absolute_mse_threshold(&self, mse_absolute_threshold: Option<T>) -> Self {
        Self {
            _internal: ICPConfiguration {
                mse_absolute_threshold,
                ..self._internal
            },
        }
    }

    /// This will specify the interval between iteration MSE's than when reached, will declare ICP convergence.
    ///
    /// # Arguments
    /// * `mse_interval_threshold`: The minimum threshold for an MSE, anything below will return a convergence.
    ///
    /// # Returns
    /// A copy of self, with the updated parameters
    pub fn with_mse_interval_threshold(&self, mse_interval_threshold: T) -> Self {
        Self {
            _internal: ICPConfiguration {
                mse_interval_threshold,
                ..self._internal
            },
        }
    }

    /// Generates an [`ICPConfiguration`] from the struct currently contained by the builder
    ///
    /// # Returns
    /// An [`ICPConfiguration`], note that this does not consume the builder, leaving it intact for another use.
    pub fn build(&self) -> ICPConfiguration<T> {
        self._internal.clone()
    }
}
