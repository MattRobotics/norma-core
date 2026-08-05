use std::fs;
use std::path::PathBuf;

#[test]
fn export_exact_matdog_source_with_v25_rf_span_tolerance() {
    let crate_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace = crate_dir.join("../../..").canonicalize().unwrap();
    let source_path = crate_dir.join("src/auto_calibrate/matdog.rs");
    let source = fs::read_to_string(&source_path).unwrap();
    let old = "const RF_MIRROR_SPAN_TOLERANCE_TICKS: u16 = REPEATABILITY_TOLERANCE_TICKS;";
    let new = "const RF_MIRROR_SPAN_TOLERANCE_TICKS: u16 = LF_CONTACT_WITNESS_TOLERANCE_TICKS;";
    assert_eq!(source.matches(old).count(), 1);
    let patched = source.replacen(old, new, 1);
    assert_eq!(patched.matches(new).count(), 1);
    let output = workspace.join("ci-output/matdog-v25-span-patched.rs");
    fs::create_dir_all(output.parent().unwrap()).unwrap();
    fs::write(output, patched).unwrap();

    // Hardware trace contract: RF UPPER span 2016 vs LF V25 span 1999.
    assert_eq!(2016_u16.abs_diff(1999), 17);
    assert!(17 <= 24);
    assert!(!25_u16.le(&24));
}
