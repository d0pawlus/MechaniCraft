//! MechaniCraft viewer: an eframe shell around [`mc_view`].
//!
//! All computation lives in `mc-view`; this crate only lays out widgets,
//! paints the mechanism and gets files in.

mod app;
mod canvas;
mod files;

pub use app::ViewerApp;
