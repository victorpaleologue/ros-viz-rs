//! Pure-Rust visual-testing toolkit for visual regression tests.
//!
//! Replaces ImageMagick-based comparisons with deterministic, dependency-light
//! routines built on the `image` crate: RMSE comparison, visual diff images,
//! silhouette extraction (coverage + bounding box), coarse color histograms,
//! and a reference-image assertion helper.
//!
//! # Blessing references
//!
//! [`assert_matches_reference`] honors the `ROS_VIZ_BLESS` environment
//! variable: when set to `1`, the actual image overwrites the reference
//! instead of being compared against it.

use std::path::Path;

use anyhow::{Context, bail};
use image::{Rgba, RgbaImage};

/// Per-channel absolute difference above which a pixel is highlighted in red
/// by [`diff_image`].
const DIFF_HIGHLIGHT_THRESHOLD: u8 = 32;

/// Amplification factor applied to per-channel differences in [`diff_image`]
/// so that subtle differences become visible.
const DIFF_AMPLIFICATION: u16 = 4;

/// Quantization step used by [`dominant_colors`] (256 / 32 levels per channel).
const COLOR_QUANT_STEP: u32 = 8;

/// Environment variable that, when set to `1`, makes
/// [`assert_matches_reference`] overwrite the reference image with the actual
/// image instead of comparing.
pub const BLESS_ENV_VAR: &str = "ROS_VIZ_BLESS";

/// Normalized root-mean-square error between two images, in `0.0..=1.0`.
///
/// The error is computed across all four RGBA channels of every pixel and
/// normalized so that `0.0` means identical images and `1.0` means maximal
/// difference (e.g. pure black vs pure white, opaque vs transparent).
///
/// Returns an error if the two images have different dimensions.
pub fn rmse(a: &RgbaImage, b: &RgbaImage) -> anyhow::Result<f64> {
    if a.dimensions() != b.dimensions() {
        bail!(
            "image dimensions differ: {}x{} vs {}x{}",
            a.width(),
            a.height(),
            b.width(),
            b.height()
        );
    }
    let mut sum_sq = 0.0f64;
    for (pa, pb) in a.pixels().zip(b.pixels()) {
        for c in 0..4 {
            let d = f64::from(pa.0[c]) - f64::from(pb.0[c]);
            sum_sq += d * d;
        }
    }
    let n = f64::from(a.width()) * f64::from(a.height()) * 4.0;
    Ok((sum_sq / n).sqrt() / 255.0)
}

/// Per-pixel absolute difference between two images, amplified for visibility.
///
/// Channel differences are multiplied by a fixed gain and clamped, so subtle
/// differences become visible. Pixels whose raw difference exceeds a threshold
/// on any RGB channel are painted pure red to make regression areas obvious.
/// The output is fully opaque. Differing dimensions are tolerated: the diff
/// covers the intersection of the two images.
pub fn diff_image(a: &RgbaImage, b: &RgbaImage) -> RgbaImage {
    let width = a.width().min(b.width());
    let height = a.height().min(b.height());
    let mut out = RgbaImage::new(width, height);
    for y in 0..height {
        for x in 0..width {
            let pa = a.get_pixel(x, y);
            let pb = b.get_pixel(x, y);
            let raw: [u8; 4] = std::array::from_fn(|c| pa.0[c].abs_diff(pb.0[c]));
            let highlighted = raw[..3].iter().any(|&d| d > DIFF_HIGHLIGHT_THRESHOLD);
            let pixel = if highlighted {
                Rgba([255, 0, 0, 255])
            } else {
                let amplified: [u8; 4] = std::array::from_fn(|c| {
                    (u16::from(raw[c]) * DIFF_AMPLIFICATION).min(255) as u8
                });
                Rgba([amplified[0], amplified[1], amplified[2], 255])
            };
            out.put_pixel(x, y, pixel);
        }
    }
    out
}

/// Structural summary of the non-background content of an image.
///
/// Produced by [`silhouette`]; lets tests assert "something is visible,
/// roughly centered, roughly this size" without pixel-perfect references.
#[derive(Debug, Clone, PartialEq)]
pub struct Silhouette {
    /// Fraction of pixels differing from the background, in `0.0..=1.0`.
    pub coverage: f64,
    /// Bounding box of the non-background pixels as
    /// `(x_min, y_min, x_max, y_max)` (inclusive), or `None` if every pixel
    /// matches the background.
    pub bbox: Option<(u32, u32, u32, u32)>,
}

/// Computes the silhouette of `img` against a background color.
///
/// A pixel belongs to the silhouette when any of its RGBA channels differs
/// from `background` by more than `tolerance`.
pub fn silhouette(img: &RgbaImage, background: Rgba<u8>, tolerance: u8) -> Silhouette {
    let mut count = 0u64;
    let mut bbox: Option<(u32, u32, u32, u32)> = None;
    for (x, y, pixel) in img.enumerate_pixels() {
        let is_foreground = pixel
            .0
            .iter()
            .zip(background.0.iter())
            .any(|(&p, &b)| p.abs_diff(b) > tolerance);
        if is_foreground {
            count += 1;
            bbox = Some(match bbox {
                None => (x, y, x, y),
                Some((x0, y0, x1, y1)) => (x0.min(x), y0.min(y), x1.max(x), y1.max(y)),
            });
        }
    }
    let total = u64::from(img.width()) * u64::from(img.height());
    let coverage = if total == 0 {
        0.0
    } else {
        count as f64 / total as f64
    };
    Silhouette { coverage, bbox }
}

/// Top-`n` dominant colors of `img` with their pixel fractions.
///
/// Colors are quantized to 32 levels per channel (a coarse histogram), and the
/// returned colors are the centers of the most populated bins, sorted by
/// descending fraction (ties broken by bin order for determinism). Fractions
/// are relative to the total pixel count. Lets tests assert e.g. "there are
/// blue link pixels and red joint pixels" without exact color matching.
pub fn dominant_colors(img: &RgbaImage, n: usize) -> Vec<(Rgba<u8>, f64)> {
    let total = u64::from(img.width()) * u64::from(img.height());
    if total == 0 || n == 0 {
        return Vec::new();
    }
    let mut histogram: std::collections::BTreeMap<[u8; 4], u64> = std::collections::BTreeMap::new();
    for pixel in img.pixels() {
        let key: [u8; 4] = std::array::from_fn(|c| {
            // Quantize to the bin center so returned colors are representative.
            let bin = u32::from(pixel.0[c]) / COLOR_QUANT_STEP;
            (bin * COLOR_QUANT_STEP + COLOR_QUANT_STEP / 2) as u8
        });
        *histogram.entry(key).or_insert(0) += 1;
    }
    let mut bins: Vec<([u8; 4], u64)> = histogram.into_iter().collect();
    // Stable sort keeps the BTreeMap (bin order) tiebreak deterministic.
    bins.sort_by_key(|&(_, count)| std::cmp::Reverse(count));
    bins.into_iter()
        .take(n)
        .map(|(key, count)| (Rgba(key), count as f64 / total as f64))
        .collect()
}

/// Asserts that `actual` matches the reference image at `reference_path`
/// within `max_rmse`, writing artifacts to `artifacts_dir` on failure.
///
/// Behavior:
/// - If the `ROS_VIZ_BLESS` environment variable is set to `1`, the reference
///   is overwritten with `actual` and the check passes.
/// - If the reference is missing, `actual` is written to
///   `artifacts_dir/<name>.new.png` and an error explains how to bless it.
/// - If the RMSE (see [`rmse`]) exceeds `max_rmse` or the dimensions differ,
///   `actual` and an amplified diff (see [`diff_image`]) are written to
///   `artifacts_dir` and a rich error reports the RMSE and artifact paths.
///
/// `artifacts_dir` is created if needed; `<name>` is the file stem of
/// `reference_path`.
pub fn assert_matches_reference(
    actual: &RgbaImage,
    reference_path: &Path,
    max_rmse: f64,
    artifacts_dir: &Path,
) -> anyhow::Result<()> {
    let name = reference_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("reference");

    if std::env::var(BLESS_ENV_VAR).is_ok_and(|v| v == "1") {
        if let Some(parent) = reference_path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("creating reference dir {}", parent.display()))?;
        }
        actual
            .save(reference_path)
            .with_context(|| format!("blessing reference {}", reference_path.display()))?;
        return Ok(());
    }

    std::fs::create_dir_all(artifacts_dir)
        .with_context(|| format!("creating artifacts dir {}", artifacts_dir.display()))?;

    if !reference_path.exists() {
        let new_path = artifacts_dir.join(format!("{name}.new.png"));
        actual
            .save(&new_path)
            .with_context(|| format!("writing candidate {}", new_path.display()))?;
        bail!(
            "reference image {} is missing; candidate written to {}. \
             Inspect it and either copy it to the reference path or re-run \
             with {}=1 to bless it.",
            reference_path.display(),
            new_path.display(),
            BLESS_ENV_VAR
        );
    }

    let reference = image::open(reference_path)
        .with_context(|| format!("loading reference {}", reference_path.display()))?
        .to_rgba8();

    let write_failure_artifacts = || -> anyhow::Result<(std::path::PathBuf, std::path::PathBuf)> {
        let actual_path = artifacts_dir.join(format!("{name}.actual.png"));
        let diff_path = artifacts_dir.join(format!("{name}.diff.png"));
        actual
            .save(&actual_path)
            .with_context(|| format!("writing actual {}", actual_path.display()))?;
        diff_image(&reference, actual)
            .save(&diff_path)
            .with_context(|| format!("writing diff {}", diff_path.display()))?;
        Ok((actual_path, diff_path))
    };

    if reference.dimensions() != actual.dimensions() {
        let (actual_path, diff_path) = write_failure_artifacts()?;
        bail!(
            "image dimensions differ from reference {}: {}x{} vs {}x{}. \
             Artifacts: actual {}, diff {}. Re-run with {}=1 to bless.",
            reference_path.display(),
            reference.width(),
            reference.height(),
            actual.width(),
            actual.height(),
            actual_path.display(),
            diff_path.display(),
            BLESS_ENV_VAR
        );
    }

    let error = rmse(&reference, actual)?;
    if error > max_rmse {
        let (actual_path, diff_path) = write_failure_artifacts()?;
        bail!(
            "image differs from reference {}: RMSE {:.6} > max {:.6}. \
             Artifacts: actual {}, diff {}. Re-run with {}=1 to bless.",
            reference_path.display(),
            error,
            max_rmse,
            actual_path.display(),
            diff_path.display(),
            BLESS_ENV_VAR
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Builds a solid-color image.
    fn solid(width: u32, height: u32, color: Rgba<u8>) -> RgbaImage {
        RgbaImage::from_pixel(width, height, color)
    }

    /// Builds a horizontal gradient from black to white, fully opaque.
    fn gradient(width: u32, height: u32) -> RgbaImage {
        RgbaImage::from_fn(width, height, |x, _| {
            let v = (x * 255 / (width - 1).max(1)) as u8;
            Rgba([v, v, v, 255])
        })
    }

    /// Draws an axis-aligned filled rectangle into `img`.
    fn draw_rect(img: &mut RgbaImage, x0: u32, y0: u32, x1: u32, y1: u32, color: Rgba<u8>) {
        for y in y0..=y1 {
            for x in x0..=x1 {
                img.put_pixel(x, y, color);
            }
        }
    }

    /// Serializes tests calling [`assert_matches_reference`]: the bless test
    /// mutates the process-wide `ROS_VIZ_BLESS` variable, which the others
    /// read, and Rust runs tests in parallel threads of one process.
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    const WHITE: Rgba<u8> = Rgba([255, 255, 255, 255]);
    const BLACK: Rgba<u8> = Rgba([0, 0, 0, 255]);
    const RED: Rgba<u8> = Rgba([255, 0, 0, 255]);
    const BLUE: Rgba<u8> = Rgba([0, 0, 255, 255]);

    #[test]
    fn rmse_identical_images_is_zero() {
        let img = gradient(64, 32);
        assert_eq!(rmse(&img, &img).unwrap(), 0.0);
    }

    #[test]
    fn rmse_black_vs_white_is_expected_value() {
        let a = solid(16, 16, BLACK);
        let b = solid(16, 16, WHITE);
        // RGB channels differ by 255, alpha by 0: sqrt(3/4 * 255^2) / 255.
        let expected = (3.0f64 / 4.0).sqrt();
        let error = rmse(&a, &b).unwrap();
        assert!((error - expected).abs() < 1e-12, "got {error}");
    }

    #[test]
    fn rmse_known_single_channel_difference() {
        let a = solid(8, 8, Rgba([100, 50, 25, 255]));
        let b = solid(8, 8, Rgba([110, 50, 25, 255]));
        // Only red differs by 10 on every pixel: sqrt(10^2 / 4) / 255.
        let expected = (100.0f64 / 4.0).sqrt() / 255.0;
        let error = rmse(&a, &b).unwrap();
        assert!((error - expected).abs() < 1e-12, "got {error}");
    }

    #[test]
    fn rmse_rejects_dimension_mismatch() {
        let a = solid(8, 8, BLACK);
        let b = solid(8, 9, BLACK);
        let err = rmse(&a, &b).unwrap_err();
        assert!(err.to_string().contains("dimensions differ"), "{err}");
    }

    #[test]
    fn diff_image_is_black_for_identical_inputs() {
        let img = gradient(32, 16);
        let diff = diff_image(&img, &img);
        assert!(diff.pixels().all(|p| *p == Rgba([0, 0, 0, 255])));
    }

    #[test]
    fn diff_image_amplifies_small_and_highlights_large_differences() {
        let a = solid(4, 1, Rgba([100, 100, 100, 255]));
        let mut b = a.clone();
        b.put_pixel(0, 0, Rgba([105, 100, 100, 255])); // small: amplified
        b.put_pixel(1, 0, Rgba([200, 100, 100, 255])); // large: red highlight
        let diff = diff_image(&a, &b);
        assert_eq!(*diff.get_pixel(0, 0), Rgba([20, 0, 0, 255]));
        assert_eq!(*diff.get_pixel(1, 0), Rgba([255, 0, 0, 255]));
        assert_eq!(*diff.get_pixel(2, 0), Rgba([0, 0, 0, 255]));
    }

    #[test]
    fn diff_image_covers_intersection_of_sizes() {
        let a = solid(8, 8, BLACK);
        let b = solid(4, 6, BLACK);
        let diff = diff_image(&a, &b);
        assert_eq!(diff.dimensions(), (4, 6));
    }

    #[test]
    fn silhouette_of_background_only_is_empty() {
        let img = solid(10, 10, WHITE);
        let s = silhouette(&img, WHITE, 0);
        assert_eq!(s.coverage, 0.0);
        assert_eq!(s.bbox, None);
    }

    #[test]
    fn silhouette_of_centered_square_has_correct_bbox_and_coverage() {
        let mut img = solid(20, 20, WHITE);
        draw_rect(&mut img, 5, 5, 14, 14, RED);
        let s = silhouette(&img, WHITE, 0);
        assert_eq!(s.bbox, Some((5, 5, 14, 14)));
        let expected_coverage = (10.0 * 10.0) / (20.0 * 20.0);
        assert!((s.coverage - expected_coverage).abs() < 1e-12, "{s:?}");
    }

    #[test]
    fn silhouette_tolerance_ignores_near_background_pixels() {
        let mut img = solid(10, 10, Rgba([200, 200, 200, 255]));
        img.put_pixel(0, 0, Rgba([205, 200, 200, 255])); // within tolerance
        img.put_pixel(9, 9, Rgba([240, 200, 200, 255])); // beyond tolerance
        let s = silhouette(&img, Rgba([200, 200, 200, 255]), 10);
        assert_eq!(s.bbox, Some((9, 9, 9, 9)));
        assert!((s.coverage - 0.01).abs() < 1e-12, "{s:?}");
    }

    #[test]
    fn dominant_colors_finds_expected_colors_and_fractions() {
        let mut img = solid(10, 10, BLUE);
        draw_rect(&mut img, 0, 0, 9, 2, RED); // 30 red pixels, 70 blue
        let colors = dominant_colors(&img, 2);
        assert_eq!(colors.len(), 2);
        // Quantized bin centers: 255 -> 252, 0 -> 4.
        assert_eq!(colors[0].0, Rgba([4, 4, 252, 252]));
        assert!((colors[0].1 - 0.7).abs() < 1e-12, "{colors:?}");
        assert_eq!(colors[1].0, Rgba([252, 4, 4, 252]));
        assert!((colors[1].1 - 0.3).abs() < 1e-12, "{colors:?}");
    }

    #[test]
    fn dominant_colors_merges_nearby_shades() {
        let mut img = solid(4, 1, Rgba([100, 100, 100, 255]));
        img.put_pixel(0, 0, Rgba([101, 102, 103, 255])); // same 8-wide bins
        let colors = dominant_colors(&img, 4);
        assert_eq!(colors.len(), 1);
        assert_eq!(colors[0].1, 1.0);
    }

    #[test]
    fn dominant_colors_handles_empty_request() {
        let img = solid(4, 4, BLUE);
        assert!(dominant_colors(&img, 0).is_empty());
    }

    #[test]
    fn assert_matches_reference_passes_within_threshold() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let reference_path = dir.path().join("ref.png");
        let artifacts = dir.path().join("artifacts");
        let img = gradient(32, 16);
        img.save(&reference_path).unwrap();
        let mut near = img.clone();
        near.put_pixel(0, 0, Rgba([10, 10, 10, 255]));
        assert_matches_reference(&near, &reference_path, 0.1, &artifacts).unwrap();
    }

    #[test]
    fn assert_matches_reference_missing_reference_writes_candidate() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let reference_path = dir.path().join("snapshot.png");
        let artifacts = dir.path().join("artifacts");
        let img = solid(8, 8, RED);
        let err = assert_matches_reference(&img, &reference_path, 0.01, &artifacts).unwrap_err();
        let candidate = artifacts.join("snapshot.new.png");
        assert!(candidate.exists(), "candidate not written");
        assert!(err.to_string().contains("missing"), "{err}");
        assert!(err.to_string().contains(BLESS_ENV_VAR), "{err}");
    }

    #[test]
    fn assert_matches_reference_failure_writes_actual_and_diff() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let reference_path = dir.path().join("scene.png");
        let artifacts = dir.path().join("artifacts");
        solid(8, 8, BLACK).save(&reference_path).unwrap();
        let actual = solid(8, 8, WHITE);
        let err = assert_matches_reference(&actual, &reference_path, 0.1, &artifacts).unwrap_err();
        assert!(artifacts.join("scene.actual.png").exists());
        assert!(artifacts.join("scene.diff.png").exists());
        let message = err.to_string();
        assert!(message.contains("RMSE"), "{message}");
        assert!(message.contains("scene.diff.png"), "{message}");
    }

    #[test]
    fn assert_matches_reference_dimension_mismatch_fails_with_artifacts() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let reference_path = dir.path().join("size.png");
        let artifacts = dir.path().join("artifacts");
        solid(8, 8, BLACK).save(&reference_path).unwrap();
        let actual = solid(4, 4, BLACK);
        let err = assert_matches_reference(&actual, &reference_path, 1.0, &artifacts).unwrap_err();
        assert!(err.to_string().contains("dimensions differ"), "{err}");
        assert!(artifacts.join("size.actual.png").exists());
    }

    /// Bless flow, exercised via the documented env var. Holds [`ENV_LOCK`]
    /// like the other reference tests since the environment is process-wide.
    #[test]
    fn assert_matches_reference_bless_overwrites_reference() {
        let _guard = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let reference_path = dir.path().join("nested").join("bless.png");
        let artifacts = dir.path().join("artifacts");
        let img = solid(8, 8, BLUE);
        // SAFETY: tests in this binary run in threads of one process; this is
        // the only test mutating this variable and it restores it before exit.
        unsafe { std::env::set_var(BLESS_ENV_VAR, "1") };
        let result = assert_matches_reference(&img, &reference_path, 0.0, &artifacts);
        unsafe { std::env::remove_var(BLESS_ENV_VAR) };
        result.unwrap();
        let blessed = image::open(&reference_path).unwrap().to_rgba8();
        assert_eq!(blessed, img);
        // Once blessed, the same image matches the reference exactly.
        assert_matches_reference(&img, &reference_path, 0.0, &artifacts).unwrap();
    }
}
