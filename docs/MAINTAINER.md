# Maintainer setup

One-time configuration only the repository owner can do. Everything else
(build, test, tag, release, package, deploy) is automated — see
[Architecture](wiki/Architecture.md) and the workflows in
[`.github/workflows/`](../.github/workflows/).

Registry names: the crate publishes to crates.io as **`ros-viz`** (so
`cargo install ros-viz` works), while the npm/browser package keeps the
**`ros-viz-rs`** name. Both are free today; the first successful publish on
each registry claims them. `cargo install ros-viz` installs two equivalent
commands, `ros-viz` and the `ros-viz-rs` alias; the distributed packages,
app bundles and binaries stay branded `ros-viz-rs`.

## crates.io publishing (Trusted Publishing, no stored token)

The `crates-io` job in `release.yml` uses [crates.io Trusted
Publishing](https://crates.io/docs/trusted-publishing): GitHub Actions
authenticates over OIDC (`rust-lang/crates-io-auth-action` + `id-token:
write`) and gets a short-lived token at publish time — **no
`CARGO_REGISTRY_TOKEN` secret to create, rotate, or leak.**

A trusted publisher can only be attached to a crate that already exists, so
there's a one-time bootstrap to claim the `ros-viz` name:

1. Sign in to <https://crates.io> with GitHub and verify your email
   (Account Settings).
2. **One-time first publish** to claim the name: create a short-lived API
   token (Account Settings → **API Tokens** → *New Token*, scope
   `publish-update`), run `cargo login` then `cargo publish` from a clean
   checkout of the tag, and **delete the token** afterwards. This is the only
   time a token is needed — CI can't publish a crate that doesn't exist yet.
3. On the new crate's page: **Settings → Trusted Publishing → Add** with
   - Repository owner: `victorpaleologue`
   - Repository name: `ros-viz-rs`
   - Workflow filename: `release.yml`
   - Environment: *(leave blank)*

From then on every tagged release publishes automatically over OIDC, with no
stored secret. Until step 3 is done the `crates-io` job is a harmless no-op
(`continue-on-error`); the GitHub release and platform binaries publish
regardless.

## npm publishing

The `npm` job needs an `NPM_TOKEN` secret (Settings → Secrets and variables →
Actions) — an npm Granular/Automation token with publish rights. npm has no
OIDC trusted-publishing equivalent here, so this one stays a stored secret.
Without it the job fails harmlessly (`continue-on-error`).

## GitHub Pages

Already enabled and deploying from the `pages.yml` workflow to
<https://victorpaleologue.github.io/ros-viz-rs/>. If it ever needs
re-enabling: **Settings → Pages → Source: GitHub Actions**.

## Release flow (no action needed, for reference)

Every PR must bump the `version` in `Cargo.toml` (enforced by the
`version-bump` CI job). On merge to `main`, `tag-release.yml` tags
`vX.Y.Z`, which triggers `release.yml` to build and attach:

- macOS `.dmg` (drag-to-install, see [`scripts/package_macos.sh`](../scripts/package_macos.sh))
- Linux `.deb` and `.rpm`
- Windows `.exe`
- crates.io + npm publishes (token-gated, above)

## Android (experimental)

The app builds for Android: a `#[bevy_main]` entry point
([`src/android.rs`](../src/android.rs)) into the shared `crate::app::run`, a
borderless-fullscreen window, MSAA off, mobile-tuned redraw, and mouse/touch
orbit controls ([`src/camera.rs`](../src/camera.rs)). Like the web build it is
**rosbridge-only** (DDS multicast is unreliable on mobile); with no on-device
connect screen yet, it launches the embedded NAO demo. Uses Bevy's default
**GameActivity** backend.

CI cross-compiles the arm64 library on every PR (the `android-build` job in
`ci.yml`, via `cargo-ndk` + the NDK), so the Android code is type-checked
continuously:

```bash
cargo ndk -t arm64-v8a build --release --lib --no-default-features --features rosbridge
```

What's **not** done yet (tracked in the Android issue): packaging that `.so`
into a signed, installable APK (GameActivity needs Gradle or `xbuild`, not
`cargo-apk`), a Play Store release (signing keystore would live in repo
secrets, like the other release secrets above), and on-device validation of
the touch controls. `[package.metadata.android]` in `Cargo.toml` already
carries the package id, label and `INTERNET` permission for whichever
packaging tool we settle on.

## Optional polish

- **macOS code signing / notarization** — the `.app` is currently unsigned,
  so Gatekeeper warns on first launch. To sign, add an Apple Developer ID
  cert + `codesign`/`notarytool` steps to the `macos-app` job (needs an
  Apple Developer account and signing secrets).
- **NAO meshes** — the demo NAO renders as a skeleton because its meshes are
  CC BY-NC-ND (not redistributable from this MIT repo). See issue #10 for
  the fetch options.

## Local development gotcha

`rustdds` does not loop back on a single host: local DDS tests need working
multicast on the default interface. On macOS, grant your terminal **Local
Network** access (System Settings → Privacy & Security → Local Network) and
check Wi-Fi/VPN. Tests skip with a clear message when multicast is absent
(`require_dds_multicast!`); the rosbridge path is unaffected.
