//! `mcraft-view`: desktop entry point, and the browser entry point when built
//! for `wasm32` (see `index.html`).

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_title("MechaniCraft")
            .with_inner_size([1100.0, 760.0])
            .with_drag_and_drop(true),
        ..Default::default()
    };
    eframe::run_native(
        "MechaniCraft",
        options,
        Box::new(|_cc| Ok(Box::new(mc_app::ViewerApp::new()))),
    )
}

/// Id of the canvas element in `index.html`.
#[cfg(target_arch = "wasm32")]
const CANVAS_ID: &str = "mc_canvas";

#[cfg(target_arch = "wasm32")]
fn main() {
    use eframe::wasm_bindgen::JsCast;

    wasm_bindgen_futures::spawn_local(async {
        let document = web_sys::window()
            .and_then(|w| w.document())
            .expect("running in a browser page");
        let canvas = document
            .get_element_by_id(CANVAS_ID)
            .and_then(|e| e.dyn_into::<web_sys::HtmlCanvasElement>().ok())
            .expect("index.html has a canvas with id mc_canvas");
        let started = eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|_cc| Ok(Box::new(mc_app::ViewerApp::new()))),
            )
            .await;
        if let Err(err) = started
            && let Some(body) = document.body()
        {
            body.set_inner_html(&format!("<p>MechaniCraft failed to start: {err:?}</p>"));
        }
    });
}
