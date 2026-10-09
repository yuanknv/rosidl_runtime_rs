cfg_if::cfg_if! {
    if #[cfg(not(feature="use_ros_shim"))] {
        use std::env;
        use std::path::Path;

        const AMENT_PREFIX_PATH: &str = "AMENT_PREFIX_PATH";

        fn get_env_var_or_abort(env_var: &'static str) -> String {
            if let Ok(value) = env::var(env_var) {
                value
            } else {
                panic!(
                    "{} environment variable not set - please source ROS 2 installation first.",
                    env_var
                );
            }
        }
    }
}

fn main() {
    println!("cargo:rustc-check-cfg=cfg(rosidl_buffer_abi)");
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    #[cfg(all(feature = "use_ros_shim", feature = "rosidl-buffer"))]
    println!("cargo:rustc-cfg=rosidl_buffer_abi");

    #[cfg(not(feature = "use_ros_shim"))]
    {
        let ament_prefix_path_list = get_env_var_or_abort(AMENT_PREFIX_PATH);
        let header = ament_prefix_path_list
            .split(':')
            .flat_map(|prefix| {
                ["include/rosidl_runtime_c", "include"].map(|include| {
                    Path::new(prefix)
                        .join(include)
                        .join("rosidl_runtime_c/primitives_sequence.h")
                })
            })
            .find(|path| path.is_file())
            .expect("rosidl_runtime_c headers are missing");
        println!("cargo:rerun-if-changed={}", header.display());
        let buffer_abi = std::fs::read_to_string(&header)
            .unwrap()
            .contains("is_rosidl_buffer");
        if buffer_abi {
            println!("cargo:rustc-cfg=rosidl_buffer_abi");
        }
        #[cfg(feature = "rosidl-buffer")]
        assert!(
            buffer_abi,
            "rosidl-buffer requires buffer-enabled rosidl_runtime_c headers"
        );

        for ament_prefix_path in ament_prefix_path_list.split(':') {
            let library_path = Path::new(ament_prefix_path).join("lib");
            println!("cargo:rustc-link-search=native={}", library_path.display());
        }

        #[cfg(feature = "rosidl-buffer")]
        build_buffer_bridge(&ament_prefix_path_list);
    }

    // Invalidate the built crate whenever this script changes
    println!("cargo:rerun-if-changed=build.rs");
}

#[cfg(all(feature = "rosidl-buffer", not(feature = "use_ros_shim")))]
fn build_buffer_bridge(ament_prefix_path: &str) {
    let prefixes: Vec<_> = std::env::split_paths(ament_prefix_path).collect();
    let buffer_include = prefixes
        .iter()
        .flat_map(|prefix| [prefix.join("include/rosidl_buffer"), prefix.join("include")])
        .find(|directory| directory.join("rosidl_buffer/buffer.hpp").is_file())
        .expect("rosidl_buffer headers are missing from AMENT_PREFIX_PATH");
    cxx_build::CFG.exported_header_dirs.push(&buffer_include);
    cxx_build::bridge("src/native.rs")
        .file("src/buffer_bridge.cpp")
        .include(&buffer_include)
        .std("c++20")
        .compile("rosidl_runtime_rs_buffer_bridge");

    for library in ["rosidl_buffer", "rosidl_runtime_c"] {
        let library_path = prefixes
            .iter()
            .map(|prefix| prefix.join("lib"))
            .find(|directory| {
                [
                    format!("lib{library}.so"),
                    format!("lib{library}.dylib"),
                    format!("{library}.lib"),
                ]
                .iter()
                .any(|name| directory.join(name).is_file())
            })
            .unwrap_or_else(|| panic!("{library} is missing from AMENT_PREFIX_PATH"));
        println!("cargo:rustc-link-search=native={}", library_path.display());
        println!("cargo:rustc-link-lib={library}");
    }
    println!("cargo:rerun-if-changed={}", buffer_include.display());
    for file in [
        "src/native.rs",
        "src/buffer_bridge.cpp",
        "src/buffer_bridge.hpp",
    ] {
        println!("cargo:rerun-if-changed={file}");
    }
}
