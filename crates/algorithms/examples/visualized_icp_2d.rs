use eframe::{egui, epaint};
use mapping_algorithms_rs::{
    icp::{icp_iteration, types::ICPConfiguration},
    kd_tree::KDTree,
    utils::point_cloud::{
        calculate_point_cloud_center, generate_point_cloud, transform_point_cloud,
    },
};
use nalgebra::{Isometry2, Point2, UnitComplex, Vector2};
use rand::Rng;

#[derive(Copy, Clone)]
struct RunConfiguration {
    num_points: usize,
    offset_x: f32,
    offset_y: f32,
    offset_angle: f32,
    max_iterations: usize,
    mse_threshold: Option<f32>,
    mse_interval_threshold: f32,
    with_kd: bool,
}

impl From<RunConfiguration> for ICPConfiguration<f32> {
    fn from(value: RunConfiguration) -> Self {
        ICPConfiguration::builder()
            .with_kd_tree(value.with_kd)
            .with_max_iterations(value.max_iterations)
            .with_absolute_mse_threshold(value.mse_threshold)
            .with_mse_interval_threshold(value.mse_interval_threshold)
            .build()
    }
}

struct VisualizerApp {
    points_a: Vec<Point2<f32>>,
    points_b: Vec<Point2<f32>>,
    kd_tree: Option<KDTree<f32, 2>>,
    transformed_points: Vec<Point2<f32>>,
    current_transform: Isometry2<f32>,
    current_mse: f32,
    current_iteration: usize,
    original_mean: Point2<f32>,
    current_means: (Point2<f32>, Point2<f32>),
    config: RunConfiguration,
    run_next_iteration: bool,
    converged: bool,
}

impl VisualizerApp {
    fn new(config: RunConfiguration) -> Self {
        let points_a = generate_point_cloud(config.num_points, -1.0..=1.0);
        let points_b = transform_point_cloud(
            &points_a,
            Isometry2::from_parts(
                Vector2::new(config.offset_x, config.offset_y).into(),
                UnitComplex::from_angle(config.offset_angle.to_radians()),
            ),
        );

        Self {
            current_transform: Isometry2::identity(),
            current_mse: f32::MAX,
            current_iteration: 0,
            current_means: (Point2::default(), Point2::default()),
            transformed_points: points_a.to_vec(),
            kd_tree: config.with_kd.then_some(KDTree::from(points_b.as_slice())),
            original_mean: calculate_point_cloud_center(&points_a),
            points_a,
            points_b,
            config,
            run_next_iteration: false,
            converged: false,
        }
    }
}

impl eframe::App for VisualizerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ctx.request_repaint();

            if self.run_next_iteration {
                log::info!("Running iteration");

                self.current_iteration += 1;
                match icp_iteration::<_, 2>(
                    &self.points_a,
                    &mut self.transformed_points,
                    &self.points_b,
                    self.kd_tree.as_ref(),
                    &mut self.current_transform,
                    &mut self.current_mse,
                    &self.config.into(),
                ) {
                    Ok(mse) => {
                        log::info!(
                            "Successfully converged after {} iterations! Last MSE: {mse}",
                            self.current_iteration
                        );
                        self.converged = true;
                    }
                    Err(means) => {
                        log::info!("MSE {}", self.current_mse);
                        self.current_means = means;
                    }
                }

                self.run_next_iteration = false;
            }

            let viewport_raw = ctx
                .input(|read| read.viewport().inner_rect)
                .expect("No outer rect found");
            let center = (viewport_raw.size() / 2.0).to_pos2();
            let scale = viewport_raw.size() / 4.0;

            let painter = ui.painter();

            // Draw centeroids
            {
                // Draw transformed source centeroid
                painter.add(egui::Shape::Rect(epaint::RectShape::filled(
                    egui::Rect::from_center_size(
                        center + egui::Vec2::from(self.current_means.0.coords.data.0[0]) * scale,
                        egui::Vec2::splat(4.0),
                    ),
                    0.0,
                    egui::Color32::from_rgb(0, 0, 255),
                )));

                // Draw source centeroid
                painter.add(egui::Shape::Rect(epaint::RectShape::filled(
                    egui::Rect::from_center_size(
                        center + egui::Vec2::from(self.original_mean.coords.data.0[0]) * scale,
                        egui::Vec2::splat(4.0),
                    ),
                    0.0,
                    egui::Color32::from_rgb(255, 0, 0),
                )));

                // Draw target centeroid
                painter.add(egui::Shape::Rect(epaint::RectShape::filled(
                    egui::Rect::from_center_size(
                        center + egui::Vec2::from(self.current_means.1.coords.data.0[0]) * scale,
                        egui::Vec2::splat(4.0),
                    ),
                    0.0,
                    egui::Color32::from_rgb(0, 255, 0),
                )));
            }

            for (point_a, (transformed_point_a, point_b)) in self
                .points_a
                .iter()
                .zip(self.transformed_points.iter().zip(self.points_b.iter()))
            {
                // Draw intermediate points first
                painter.add(egui::Shape::Circle(epaint::CircleShape::filled(
                    center + egui::Vec2::from(transformed_point_a.coords.data.0[0]) * scale,
                    1.0,
                    egui::Color32::from_rgb(0, 0, 255),
                )));

                // Draw source points
                painter.add(egui::Shape::Circle(epaint::CircleShape::filled(
                    center + egui::Vec2::from(point_a.coords.data.0[0]) * scale,
                    1.0,
                    egui::Color32::from_rgb(255, 0, 0),
                )));

                // Draw target points
                painter.add(egui::Shape::Circle(epaint::CircleShape::filled(
                    center + egui::Vec2::from(point_b.coords.data.0[0]) * scale,
                    1.0,
                    egui::Color32::from_rgb(0, 255, 0),
                )));
            }

            if !self.converged && ctx.input(|read| read.key_released(egui::Key::Space)) {
                if self.current_iteration < self.config.max_iterations {
                    self.run_next_iteration = true;
                } else {
                    log::warn!("No iterations left");
                }
            }
        });
    }
}

fn main() -> eframe::Result<()> {
    simple_logger::SimpleLogger::new()
        .with_utc_timestamps()
        .with_level(log::LevelFilter::Info)
        .with_threads(true)
        .init()
        .ok();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1280.0, 720.0]),
        multisampling: 4,
        renderer: eframe::Renderer::Glow,
        ..Default::default()
    };

    eframe::run_native(
        "mapping-rs 2D ICP Algorithm Visualization Tool",
        options,
        Box::new(|_cc| {
            let mut rng = rand::thread_rng();
            Box::new(VisualizerApp::new(RunConfiguration {
                num_points: 500,
                offset_x: rng.gen_range(-0.1..=0.1),
                offset_y: rng.gen_range(-0.1..=0.1),
                offset_angle: rng.gen_range(-10.0..=10.0),
                max_iterations: 50,
                mse_threshold: None,
                mse_interval_threshold: 0.01,
                with_kd: false,
            }))
        }),
    )
}
