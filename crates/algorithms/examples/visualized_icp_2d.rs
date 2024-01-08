use eframe::{egui, epaint};
use mapping_algorithms_rs::icp::icp_iteration;
use mapping_algorithms_rs::kd_tree::KDTree;
use mapping_algorithms_rs::utils::point_cloud::{
    calculate_point_cloud_center, generate_point_cloud, transform_point_cloud,
};
use nalgebra::{Const, Isometry2, Point2, UnitComplex, Vector2};

struct RunConfiguartion {
    max_iterations: usize,
    mse_threshold: f32,
    with_kd: bool,
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
    config: RunConfiguartion,
    run_next_iteration: bool,
}

impl VisualizerApp {
    fn new(config: RunConfiguartion) -> Self {
        let points_a = generate_point_cloud(500, -1.0..=1.0);
        let points_b = transform_point_cloud(
            &points_a,
            Isometry2::from_parts(
                Vector2::new(0.183, 0.712).into(),
                UnitComplex::from_angle(12.84f32.to_radians()),
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
        }
    }
}

impl eframe::App for VisualizerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ctx.request_repaint();

            if self.run_next_iteration {
                log::info!("Running iteration");

                if let Err((mean_a, mean_b)) = icp_iteration::<f32, 2, Const<2>>(
                    &self.points_a,
                    &mut self.transformed_points,
                    &self.points_b,
                    self.kd_tree.as_ref(),
                    &mut self.current_transform,
                    &mut self.current_mse,
                    self.config.mse_threshold,
                ) {
                    self.current_means = (mean_a, mean_b);
                }

                log::info!("MSE {}", self.current_mse);

                self.current_iteration += 1;
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

            if ctx.input(|read| read.key_released(egui::Key::Space)) {
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
            Box::new(VisualizerApp::new(RunConfiguartion {
                max_iterations: 10,
                mse_threshold: 0.001,
                with_kd: false,
            }))
        }),
    )
}
