use anyhow::Result;
use std::path::Path;
use std::process::Command;

/// Configuration for visual regression tests
struct VisualTest {
    name: &'static str,
    urdf_path: &'static str,
    reference_image: &'static str,
}

const VISUAL_TESTS: &[VisualTest] = &[
    VisualTest {
        name: "simple_arm",
        urdf_path: "test-data/urdf/simple_arm.urdf",
        reference_image: "test-data/reference/simple_arm.png",
    },
    VisualTest {
        name: "two_link_planar",
        urdf_path: "test-data/urdf/two_link_planar.urdf",
        reference_image: "test-data/reference/two_link_planar.png",
    },
    VisualTest {
        name: "triple_pendulum",
        urdf_path: "test-data/urdf/triple_pendulum.urdf",
        reference_image: "test-data/reference/triple_pendulum.png",
    },
];

/// Maximum acceptable difference (0.0 = identical, 1.0 = completely different)
const MAX_DIFF_THRESHOLD: f64 = 0.001; // 0.1% difference allowed

#[test]
fn visual_regression_tests() -> Result<()> {
    // Check if ImageMagick is available
    if !is_imagemagick_available() {
        eprintln!("⚠️  ImageMagick not found - skipping visual regression tests");
        eprintln!(
            "   Install with: brew install imagemagick (macOS) or apt install imagemagick (Linux)"
        );
        return Ok(());
    }

    let temp_dir = std::env::temp_dir().join("ros-viz-rs-visual-tests");
    std::fs::create_dir_all(&temp_dir)?;

    let mut all_passed = true;
    let mut results = Vec::new();

    for test in VISUAL_TESTS {
        println!("\n🔍 Testing: {}", test.name);

        let output_path = temp_dir.join(format!("{}.png", test.name));
        let diff_path = temp_dir.join(format!("{}_diff.png", test.name));

        // Generate fresh render
        println!("   Rendering URDF...");
        let render_result = render_urdf(test.urdf_path, &output_path)?;
        if !render_result.success {
            eprintln!("   ✗ Render failed");
            all_passed = false;
            results.push((test.name, false, None));
            continue;
        }

        // Compare with reference
        println!("   Comparing with reference...");
        let diff = compare_images(test.reference_image, &output_path, &diff_path)?;

        let passed = diff <= MAX_DIFF_THRESHOLD;
        results.push((test.name, passed, Some(diff)));

        if passed {
            println!("   ✓ PASS (diff: {:.4}%)", diff * 100.0);
        } else {
            println!(
                "   ✗ FAIL (diff: {:.4}% > {:.4}%)",
                diff * 100.0,
                MAX_DIFF_THRESHOLD * 100.0
            );
            println!("   Reference: {}", test.reference_image);
            println!("   Generated: {}", output_path.display());
            println!("   Diff image: {}", diff_path.display());
            all_passed = false;
        }
    }

    // Summary
    println!("\n{}", "=".repeat(60));
    println!("Visual Regression Test Summary:");
    println!("{}", "=".repeat(60));
    for (name, passed, diff) in &results {
        let status = if *passed { "✓ PASS" } else { "✗ FAIL" };
        let diff_str = diff
            .map(|d| format!("{:.4}%", d * 100.0))
            .unwrap_or_else(|| "N/A".to_string());
        println!("{:20} {} (diff: {})", name, status, diff_str);
    }
    println!("{}", "=".repeat(60));

    assert!(
        all_passed,
        "Visual regression tests failed - see diff images in {}",
        temp_dir.display()
    );

    Ok(())
}

fn render_urdf(urdf_path: &str, output_path: &Path) -> Result<RenderResult> {
    let output = Command::new("cargo")
        .args(&[
            "run",
            "--example",
            "test_urdf",
            "--",
            urdf_path,
            output_path.to_str().unwrap(),
        ])
        .output()?;

    Ok(RenderResult {
        success: output.status.success(),
    })
}

struct RenderResult {
    success: bool,
}

fn compare_images(reference: &str, generated: &Path, diff_output: &Path) -> Result<f64> {
    let output = Command::new("magick")
        .args(&[
            "compare",
            "-metric",
            "RMSE",
            reference,
            generated.to_str().unwrap(),
            diff_output.to_str().unwrap(),
        ])
        .output()?;

    // ImageMagick compare writes metrics to stderr
    let stderr = String::from_utf8_lossy(&output.stderr);

    // Parse RMSE output: "1234.56 (0.0123)"
    // We want the normalized value in parentheses
    let diff = parse_rmse(&stderr)?;

    Ok(diff)
}

fn parse_rmse(stderr: &str) -> Result<f64> {
    // Example output: "1234.56 (0.0123)"
    // We want the value in parentheses (normalized difference)
    if let Some(start) = stderr.find('(') {
        if let Some(end) = stderr.find(')') {
            let diff_str = &stderr[start + 1..end];
            return diff_str
                .parse::<f64>()
                .map_err(|e| anyhow::anyhow!("Failed to parse RMSE: {}", e));
        }
    }

    Err(anyhow::anyhow!("Could not parse RMSE from: {}", stderr))
}

fn is_imagemagick_available() -> bool {
    Command::new("magick")
        .arg("--version")
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false)
}
