use std::{env, fs};

fn main() {
    println!("cargo:rustc-check-cfg=cfg(rosidl_buffer_abi)");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-changed=build.rs");
    let buffers = env::var_os("CARGO_FEATURE_ROSIDL_BUFFER").is_some();
    if env::var_os("CARGO_FEATURE_USE_ROS_SHIM").is_some() {
        if buffers {
            println!("cargo:rustc-cfg=rosidl_buffer_abi");
        }
        return;
    }
    let prefixes = env::var_os("AMENT_PREFIX_PATH")
        .expect("AMENT_PREFIX_PATH is not set; source the ROS installation first");
    let prefixes: Vec<_> = env::split_paths(&prefixes).collect();
    let header = prefixes
        .iter()
        .flat_map(|prefix| {
            ["include/rosidl_runtime_c", "include"].map(|include| {
                prefix
                    .join(include)
                    .join("rosidl_runtime_c/primitives_sequence.h")
            })
        })
        .find(|path| path.is_file())
        .expect("rosidl_runtime_c headers are missing");
    println!("cargo:rerun-if-changed={}", header.display());
    let buffer_abi = fs::read_to_string(&header)
        .unwrap()
        .contains("is_rosidl_buffer");
    if buffer_abi {
        println!("cargo:rustc-cfg=rosidl_buffer_abi");
    }
    assert!(
        !buffers || buffer_abi,
        "rosidl-buffer requires buffer-enabled rosidl_runtime_c headers"
    );
    for prefix in prefixes {
        println!(
            "cargo:rustc-link-search=native={}",
            prefix.join("lib").display()
        );
    }
}
