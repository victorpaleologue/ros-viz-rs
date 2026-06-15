//! Browser entry point.
//!
//! Compiled only for wasm and exported through `wasm-bindgen`; the page
//! calls [`start`] with an optional rosbridge URL. Without one, the
//! embedded NAO demo plays — the GitHub Pages demo does exactly that.
//! Rendering goes into the `#ros-viz-canvas` element (see
//! [`crate::app::build_app`]).

use wasm_bindgen::prelude::*;

use crate::options::Options;

/// Start the visualizer in the page.
///
/// `rosbridge_url` connects to a rosbridge server (`ws://…`/`wss://…`);
/// `None`/`undefined` runs the built-in NAO waving demo.
#[wasm_bindgen]
pub fn start(rosbridge_url: Option<String>) {
    console_error_panic_hook();
    let options = Options {
        demo: rosbridge_url.is_none(),
        rosbridge: rosbridge_url,
        ..Options::default()
    };
    // `run` only returns on app exit, which never happens in the browser.
    let _ = crate::app::run(options);
}

/// Choose the robot the demo shows, by its URDF. Call before [`start`]; the
/// page fetches the URDF (and any meshes via [`add_mesh`]) and selects it.
#[wasm_bindgen]
pub fn set_demo_robot(urdf_xml: String) {
    crate::demo::set_demo_urdf(urdf_xml);
}

/// Supply a mesh file from the page (e.g. a user upload), keyed by its file
/// name or URI. The running app reloads the current robot so the mesh shows.
/// Lets users provide license-bound meshes (NAO) without hosting them.
#[wasm_bindgen]
pub fn add_mesh(name: String, bytes: Vec<u8>) {
    crate::scene::queue_mesh_blob(name, bytes);
}

/// Route panics to the browser console with a readable backtrace.
fn console_error_panic_hook() {
    static SET: std::sync::Once = std::sync::Once::new();
    SET.call_once(|| {
        std::panic::set_hook(Box::new(|info| {
            web_sys_error(&info.to_string());
        }));
    });
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console, js_name = error)]
    fn web_sys_error(message: &str);
}
