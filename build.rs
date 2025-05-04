#![allow(
    clippy::single_call_fn,
    reason = "Many functions are meant for single use"
)]
#![allow(clippy::shadow_reuse, reason = "This allows better naming")]
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    reason = "This is compile time so panic is fine"
)]
#![allow(clippy::use_debug, reason = "Allows for easier printing")]

use micropb_gen::config::OptionalRepr;
use micropb_gen::Config;
use std::fs::read_dir;
use std::path::{Path, PathBuf};

fn find_proto_files(dir: &Path) -> Vec<PathBuf> {
    let mut proto_files = Vec::new();
    if dir.is_dir() {
        for entry in read_dir(dir).expect("Failed to read proto directory") {
            let entry = entry.expect("Failed to read directory entry");
            let path = entry.path();
            if path.is_dir() {
                // Recursively search in subdirectories
                proto_files.extend(find_proto_files(&path));
            } else if path.extension().and_then(|os_str| os_str.to_str()) == Some("proto") {
                proto_files.push(path);
            }
        }
    }
    proto_files
}

fn main() {
    const PROTO_DIR: &str = "./proto";

    let mut gen = micropb_gen::Generator::new();

    let proto_files = find_proto_files(Path::new(&PROTO_DIR))
        .iter()
        .map(|path| path.display().to_string())
        .collect::<Vec<String>>();
    println!("{proto_files:?}");
    gen.use_container_heapless();
    gen.configure(".", Config::new().optional_repr(OptionalRepr::Option));
    gen.add_protoc_arg(format!("--proto_path={PROTO_DIR}"));
    gen.configure(".", micropb_gen::Config::new().max_len(8).max_bytes(16));
    gen.format(true);
    gen.compile_protos(&proto_files, "src/serial/proto.rs")
        .unwrap();
}
