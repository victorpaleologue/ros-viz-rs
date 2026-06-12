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
