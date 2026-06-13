# Maintainer setup

One-time configuration only the repository owner can do. Everything else
(build, test, tag, release, package, deploy) is automated — see
[Architecture](wiki/Architecture.md) and the workflows in
[`.github/workflows/`](../.github/workflows/).

## Release secrets

Add under **Settings → Secrets and variables → Actions**. Without them the
matching release jobs fail harmlessly (`continue-on-error`); the GitHub
release and the platform binaries still publish.

| Secret | Used by | Get it from |
|---|---|---|
| `CARGO_REGISTRY_TOKEN` | `crates-io` job in `release.yml` | crates.io → Account Settings → API Tokens (scope: publish-update) |
| `NPM_TOKEN` | `npm` job in `release.yml` | npmjs.com → Access Tokens → Granular/Automation token with publish rights |

The package names `ros-viz-rs` are currently free on both registries; claim
them with the first successful publish.

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
