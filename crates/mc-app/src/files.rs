//! Getting project bytes into the app: file dialog and drag and drop.
//!
//! On the desktop files are read synchronously. In the browser reading is
//! asynchronous, so results arrive later through the [`Inbox`].

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use eframe::egui;

/// A file read from disk or the browser: `(name, bytes)` or an error message.
pub type Loaded = Result<(String, Vec<u8>), String>;

/// Files waiting to be opened by the app.
#[derive(Debug, Clone, Default)]
pub struct Inbox(Arc<Mutex<Vec<Loaded>>>);

impl Inbox {
    fn lock(&self) -> MutexGuard<'_, Vec<Loaded>> {
        // A panic while pushing cannot leave the vector inconsistent.
        self.0.lock().unwrap_or_else(PoisonError::into_inner)
    }

    pub fn push(&self, item: Loaded) {
        self.lock().push(item);
    }

    /// Removes and returns everything received so far.
    pub fn take(&self) -> Vec<Loaded> {
        std::mem::take(&mut *self.lock())
    }
}

fn dialog_filters<T>(add: impl FnOnce(&'static str, &[&'static str]) -> T) -> T {
    add("SAM project", &["sam", "SAM"])
}

/// Shows the open-file dialog; the chosen file arrives in `inbox`.
#[cfg(not(target_arch = "wasm32"))]
pub fn pick_file(inbox: &Inbox, _ctx: &egui::Context) {
    let Some(path) =
        dialog_filters(|name, ext| rfd::FileDialog::new().add_filter(name, ext)).pick_file()
    else {
        return;
    };
    inbox.push(read_path(&path));
}

#[cfg(not(target_arch = "wasm32"))]
fn read_path(path: &std::path::Path) -> Loaded {
    let name = path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    );
    std::fs::read(path)
        .map(|bytes| (name.clone(), bytes))
        .map_err(|err| format!("{name}: {err}"))
}

/// Shows the open-file dialog; the chosen file arrives in `inbox`.
#[cfg(target_arch = "wasm32")]
pub fn pick_file(inbox: &Inbox, ctx: &egui::Context) {
    let dialog = dialog_filters(|name, ext| rfd::AsyncFileDialog::new().add_filter(name, ext));
    let inbox = inbox.clone();
    let ctx = ctx.clone();
    wasm_bindgen_futures::spawn_local(async move {
        if let Some(file) = dialog.pick_file().await {
            inbox.push(Ok((file.file_name(), file.read().await)));
            ctx.request_repaint();
        }
    });
}

/// Reads files dropped on the window into `inbox`.
#[cfg(not(target_arch = "wasm32"))]
pub fn receive_dropped(inbox: &Inbox, _ctx: &egui::Context, files: Vec<egui::DroppedFileHandle>) {
    for file in files {
        let name = file_name(&*file);
        inbox.push(
            file.bytes()
                .map(|bytes| (name.clone(), bytes))
                .map_err(|err| format!("{name}: {err}")),
        );
    }
}

/// Reads files dropped on the page into `inbox`.
#[cfg(target_arch = "wasm32")]
pub fn receive_dropped(inbox: &Inbox, ctx: &egui::Context, files: Vec<egui::DroppedFileHandle>) {
    for file in files {
        let inbox = inbox.clone();
        let ctx = ctx.clone();
        wasm_bindgen_futures::spawn_local(async move {
            let name = file_name(&*file);
            inbox.push(
                file.bytes_async()
                    .await
                    .map(|bytes| (name.clone(), bytes))
                    .map_err(|err| format!("{name}: {err}")),
            );
            ctx.request_repaint();
        });
    }
}

fn file_name(file: &dyn egui::DroppedFile) -> String {
    let path = file.path();
    path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inbox_take_drains_in_order() {
        let inbox = Inbox::default();
        inbox.push(Ok(("a.sam".into(), vec![1])));
        inbox.clone().push(Err("b.sam: missing".into()));
        let items = inbox.take();
        assert_eq!(items.len(), 2);
        assert_eq!(items[0], Ok(("a.sam".into(), vec![1])));
        assert!(inbox.take().is_empty());
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn read_path_reports_missing_file_with_its_name() {
        let err = read_path(std::path::Path::new("/definitely/missing/x.sam")).unwrap_err();
        assert!(err.starts_with("x.sam: "), "{err}");
    }
}
