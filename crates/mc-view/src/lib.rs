//! GUI-independent view model for MechaniCraft.
//!
//! Everything the viewer shows is computed here so it can be tested without a
//! window: loading a project ([`load`]), scene geometry per frame
//! ([`Document::link_segments`]), playback ([`Playback`]), plot channels
//! ([`Document::channels`], [`Document::samples`]) and node paths
//! ([`Document::node_path`]).

mod channels;
mod document;
mod playback;
mod scene;
mod transform;

pub use channels::{Channel, Derivative};
pub use document::{Document, LoadError, load};
pub use playback::Playback;
pub use scene::{Bounds, LinkKind, LinkSegment, Scene, SceneLink, SceneNode};
pub use transform::ViewTransform;
