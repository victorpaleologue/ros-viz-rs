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
2. **One-time first publish** from your machine to claim the name. Either:
   - create a short-lived API token (Account Settings → **API Tokens** →
     *New Token*, scope `publish-update`), then
     `CARGO_REGISTRY_TOKEN=<token> cargo publish` (or `cargo login` then
     `cargo publish`) from a clean checkout of the tag — **then delete that
     token**; or
   - skip the token entirely by letting CI fail once, since CI can't publish
     a crate that doesn't exist yet — the manual publish above is the simpler
     path.
3. On the new crate's page: **Settings → Trusted Publishing → Add** with
   - Repository owner: `victorpaleologue`
   - Repository name: `ros-viz-rs`
   - Workflow filename: `release.yml`
   - Environment: *(leave blank)*

From then on every tagged release publishes automatically over OIDC, and you
can remove any `CARGO_REGISTRY_TOKEN` secret. Until step 3 is done the
`crates-io` job is a harmless no-op (`continue-on-error`); the GitHub release
and platform binaries publish regardless.

> If you'd rather not use Trusted Publishing, the classic path still works:
> add a `CARGO_REGISTRY_TOKEN` repo secret (Settings → Secrets and variables →
> Actions) and swap the auth step back to
> `cargo publish --token "${{ secrets.CARGO_REGISTRY_TOKEN }}"`.

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
