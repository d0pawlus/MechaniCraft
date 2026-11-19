//! Painting the mechanism in its current frame.

use eframe::egui::{self, Color32, Pos2, Sense, Shape, Stroke};
use mc_view::{Document, LinkKind, Playback, SceneNode, ViewTransform};

/// Empty border around the mechanism, in points.
const MARGIN: f64 = 32.0;
const LINK_WIDTH: f32 = 4.0;
const SLIDER_WIDTH: f32 = 8.0;
const NODE_RADIUS: f32 = 4.0;
const SUPPORT_SIZE: f32 = 10.0;
const PATH_WIDTH: f32 = 1.5;

/// Paints `doc` at the playback frame, with the path of `path_node`.
pub fn paint(ui: &mut egui::Ui, doc: &Document, playback: &Playback, path_node: Option<u32>) {
    let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::hover());
    let rect = response.rect;
    let (Some(bounds), Some(frame)) = (doc.bounds(), playback.frame()) else {
        return;
    };
    let vt = ViewTransform::fit(
        bounds,
        [f64::from(rect.min.x), f64::from(rect.min.y)],
        [f64::from(rect.width()), f64::from(rect.height())],
        MARGIN,
    );
    let to_pos = |p: [f64; 2]| {
        let s = vt.to_screen(p);
        // Screen coordinates fit comfortably in f32.
        Pos2::new(s[0] as f32, s[1] as f32)
    };
    let visuals = ui.visuals();
    let fg = visuals.strong_text_color();
    let accent = visuals.selection.bg_fill;
    let faint = visuals.weak_text_color();

    if let Some(nr) = path_node {
        let points: Vec<Pos2> = doc.node_path(nr).into_iter().map(to_pos).collect();
        painter.add(Shape::line(points, Stroke::new(PATH_WIDTH, accent)));
    }
    for seg in doc.link_segments(frame) {
        let width = match seg.kind {
            LinkKind::Beam => LINK_WIDTH,
            LinkKind::Slider => SLIDER_WIDTH,
        };
        painter.line_segment([to_pos(seg.p), to_pos(seg.q)], Stroke::new(width, fg));
    }
    for node in &doc.scene.nodes {
        let Some(p) = doc.node_position(frame, node.nr) else {
            continue;
        };
        let center = to_pos(p);
        paint_support(&painter, node, center, faint);
        painter.circle_filled(center, NODE_RADIUS, visuals.panel_fill);
        painter.circle_stroke(center, NODE_RADIUS, Stroke::new(1.5, fg));
    }
}

/// Triangle for a fixed node, a line under or beside a node fixed in one axis.
fn paint_support(painter: &egui::Painter, node: &SceneNode, c: Pos2, color: Color32) {
    let s = SUPPORT_SIZE;
    let stroke = Stroke::new(1.5, color);
    match (node.fixed_x, node.fixed_y) {
        (true, true) => {
            let pts = vec![
                c,
                Pos2::new(c.x - s, c.y + 1.6 * s),
                Pos2::new(c.x + s, c.y + 1.6 * s),
            ];
            painter.add(Shape::convex_polygon(pts, Color32::TRANSPARENT, stroke));
        }
        // Fixed y: guide below the node, along x.
        (false, true) => {
            painter.line_segment(
                [
                    Pos2::new(c.x - 2.0 * s, c.y + s),
                    Pos2::new(c.x + 2.0 * s, c.y + s),
                ],
                stroke,
            );
        }
        // Fixed x: guide beside the node, along y.
        (true, false) => {
            painter.line_segment(
                [
                    Pos2::new(c.x - s, c.y - 2.0 * s),
                    Pos2::new(c.x - s, c.y + 2.0 * s),
                ],
                stroke,
            );
        }
        (false, false) => {}
    }
}
