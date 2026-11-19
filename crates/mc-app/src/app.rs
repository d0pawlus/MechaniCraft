//! Viewer state and layout: controls on the left, plot at the bottom,
//! mechanism in the centre.

use eframe::egui;
use egui_plot::{Line, Plot, VLine};
use mc_view::{Channel, Document, Playback, load};

use crate::canvas;
use crate::files::{self, Inbox};

/// Bundled example, shown at start so the web build has something to play.
const DEMO_NAME: &str = "slider-crank (demo)";
const DEMO: &str = include_str!("../../../fixtures/demo/slider-crank.sam");
/// Default height of the plot panel in points.
const PLOT_HEIGHT: f32 = 260.0;
/// Width of the controls panel in points.
const CONTROLS_WIDTH: f32 = 240.0;
/// Playback speed range (model seconds per second).
const SPEED_RANGE: std::ops::RangeInclusive<f64> = 0.05..=4.0;

/// The viewer application.
pub struct ViewerApp {
    doc: Option<Document>,
    playback: Playback,
    channel: Option<Channel>,
    path_node: Option<u32>,
    error: Option<String>,
    inbox: Inbox,
    /// Bumped on every opened document, so the plot forgets its zoom.
    generation: u64,
}

impl Default for ViewerApp {
    fn default() -> Self {
        Self::new()
    }
}

impl ViewerApp {
    /// Viewer with the bundled demo loaded and playing.
    pub fn new() -> Self {
        let mut app = Self {
            doc: None,
            playback: Playback::new(Vec::new()),
            channel: None,
            path_node: None,
            error: None,
            inbox: Inbox::default(),
            generation: 0,
        };
        app.open(DEMO_NAME, DEMO.as_bytes());
        app
    }

    /// Loads a project. On error the current document stays and the message
    /// is shown.
    pub fn open(&mut self, name: &str, bytes: &[u8]) {
        match load(name, bytes) {
            Ok(doc) => {
                self.playback = Playback::new(doc.times());
                self.playback.playing = true;
                self.channel = doc.default_channel();
                self.path_node = doc.default_path_node();
                self.error = None;
                self.doc = Some(doc);
                self.generation += 1;
            }
            Err(err) => self.error = Some(format!("{name}: {err}")),
        }
    }

    fn receive_files(&mut self, ctx: &egui::Context) {
        let dropped = ctx.input(|i| i.raw.dropped_files.clone());
        if !dropped.is_empty() {
            files::receive_dropped(&self.inbox, ctx, dropped);
        }
        for item in self.inbox.take() {
            match item {
                Ok((name, bytes)) => self.open(&name, &bytes),
                Err(err) => self.error = Some(err),
            }
        }
    }

    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.heading("MechaniCraft");
        ui.horizontal(|ui| {
            if ui.button("Open .SAM…").clicked() {
                files::pick_file(&self.inbox, ui.ctx());
            }
            if ui.button("Demo").clicked() {
                self.open(DEMO_NAME, DEMO.as_bytes());
            }
        });
        ui.label("…or drop a file on the window.");
        if let Some(err) = &self.error {
            ui.colored_label(ui.visuals().error_fg_color, err);
        }
        let Some(doc) = &self.doc else {
            return;
        };
        ui.separator();
        ui.strong(&doc.name);
        for w in &doc.warnings {
            ui.colored_label(ui.visuals().warn_fg_color, w);
        }
        ui.separator();
        playback_controls(ui, &mut self.playback);
        ui.separator();
        channel_picker(ui, doc, &mut self.channel);
        path_picker(ui, doc, &mut self.path_node);
    }

    fn plot(&mut self, ui: &mut egui::Ui) {
        let (Some(doc), Some(channel)) = (&self.doc, self.channel) else {
            ui.label("No channel selected.");
            return;
        };
        let label = doc.label(channel);
        let samples = doc.samples(channel);
        let time = self.playback.time;
        // A new id per document and channel starts with fitted bounds.
        let response = Plot::new(("channel_plot", self.generation, channel))
            .x_axis_label("t [s]")
            .y_axis_label(label.clone())
            .show(ui, |plot| {
                plot.line(Line::new(label, samples));
                plot.vline(VLine::new("t", time));
                plot.pointer_coordinate()
            });
        if response.response.clicked()
            && let Some(p) = response.inner
        {
            self.playback.playing = false;
            self.playback.time = p.x;
        }
    }
}

fn playback_controls(ui: &mut egui::Ui, playback: &mut Playback) {
    let Some((start, end)) = playback.range() else {
        return;
    };
    ui.horizontal(|ui| {
        let text = if playback.playing {
            "⏸ Pause"
        } else {
            "▶ Play"
        };
        if ui.button(text).clicked() {
            playback.playing = !playback.playing;
        }
        ui.checkbox(&mut playback.looping, "Loop");
    });
    ui.add(egui::Slider::new(&mut playback.time, start..=end).text("t [s]"));
    ui.add(
        egui::Slider::new(&mut playback.speed, SPEED_RANGE)
            .logarithmic(true)
            .text("speed"),
    );
    if let Some(frame) = playback.frame() {
        ui.label(format!("frame {} / {}", frame + 1, playback.frame_count()));
    }
}

fn channel_picker(ui: &mut egui::Ui, doc: &Document, channel: &mut Option<Channel>) {
    let selected = channel.map_or_else(|| "—".to_owned(), |c| doc.label(c));
    egui::ComboBox::from_label("plot")
        .selected_text(selected)
        .show_ui(ui, |ui| {
            for c in doc.channels() {
                ui.selectable_value(channel, Some(c), doc.label(c));
            }
        });
}

fn path_picker(ui: &mut egui::Ui, doc: &Document, node: &mut Option<u32>) {
    let selected = node.map_or_else(|| "none".to_owned(), |nr| format!("node {nr}"));
    egui::ComboBox::from_label("path")
        .selected_text(selected)
        .show_ui(ui, |ui| {
            ui.selectable_value(node, None, "none");
            for n in &doc.scene.nodes {
                ui.selectable_value(node, Some(n.nr), format!("node {}", n.nr));
            }
        });
}

impl eframe::App for ViewerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.receive_files(&ctx);
        self.playback.advance(f64::from(ctx.input(|i| i.stable_dt)));
        if self.playback.playing {
            ctx.request_repaint();
        }

        egui::Panel::left("controls")
            .default_size(CONTROLS_WIDTH)
            .show(ui, |ui| self.controls(ui));
        egui::Panel::bottom("plot")
            .resizable(true)
            .default_size(PLOT_HEIGHT)
            .show(ui, |ui| self.plot(ui));
        egui::CentralPanel::default_margins().show(ui, |ui| match &self.doc {
            Some(doc) => canvas::paint(ui, doc, &self.playback, self.path_node),
            None => {
                ui.centered_and_justified(|ui| ui.label("Open a .SAM project."));
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_with_demo_playing_and_defaults_chosen() {
        let app = ViewerApp::new();
        let doc = app.doc.as_ref().expect("demo loaded");
        assert_eq!(doc.name, DEMO_NAME);
        assert!(app.playback.playing);
        assert_eq!(app.channel.map(|c| doc.label(c)).as_deref(), Some("x3"));
        assert_eq!(app.path_node, Some(2));
        assert!(app.error.is_none());
    }

    #[test]
    fn failed_open_keeps_document_and_reports_name() {
        let mut app = ViewerApp::new();
        app.open("broken.sam", b"garbage");
        assert_eq!(app.doc.as_ref().map(|d| d.name.as_str()), Some(DEMO_NAME));
        let err = app.error.as_deref().unwrap();
        assert!(err.starts_with("broken.sam: import failed"), "{err}");
    }

    #[test]
    fn successful_open_clears_previous_error() {
        let mut app = ViewerApp::new();
        app.open("broken.sam", b"garbage");
        app.open("again.sam", DEMO.as_bytes());
        assert!(app.error.is_none());
        assert_eq!(app.doc.as_ref().map(|d| d.name.as_str()), Some("again.sam"));
    }

    #[test]
    fn inbox_items_are_opened_or_reported() {
        let mut app = ViewerApp::new();
        app.inbox.push(Err("x.sam: denied".into()));
        app.receive_files(&egui::Context::default());
        assert_eq!(app.error.as_deref(), Some("x.sam: denied"));
        app.inbox
            .push(Ok(("y.sam".into(), DEMO.as_bytes().to_vec())));
        app.receive_files(&egui::Context::default());
        assert_eq!(app.doc.as_ref().map(|d| d.name.as_str()), Some("y.sam"));
    }
}
