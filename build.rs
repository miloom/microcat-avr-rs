use std::fs::read_dir;
use std::path::{Path, PathBuf};

fn main() {
    let mut gen = micropb_gen::Generator::new();

    const PROTO_DIR: &str = "./proto";

    let proto_files = find_proto_files(&Path::new(&PROTO_DIR))
        .iter()
        .map(|p| p.display().to_string())
        .collect::<Vec<String>>();
    eprintln!("{:?}", proto_files);
    gen.use_container_heapless();
    gen.add_protoc_arg(format!("--proto_path={PROTO_DIR}"));
    gen.configure(".", micropb_gen::Config::new().max_len(8).max_bytes(16));

    gen.compile_protos(
        &proto_files,
        std::env::var("OUT_DIR").unwrap() + "/proto.rs",
    )
    .unwrap();
}

fn find_proto_files(dir: &Path) -> Vec<PathBuf> {
    let mut proto_files = Vec::new();
    if dir.is_dir() {
        for entry in read_dir(dir).expect("Failed to read proto directory") {
            let entry = entry.expect("Failed to read directory entry");
            let path = entry.path();
            if path.is_dir() {
                // Recursively search in subdirectories
                proto_files.extend(find_proto_files(&path));
            } else if path.extension().and_then(|s| s.to_str()) == Some("proto") {
                proto_files.push(path);
            }
        }
    }
    proto_files
}
