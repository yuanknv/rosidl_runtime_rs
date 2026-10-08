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
        assert!(
            !cfg!(feature = "rosidl-buffer") || buffer_abi,
            "rosidl-buffer requires buffer-enabled rosidl_runtime_c headers"
        );

        for ament_prefix_path in ament_prefix_path_list.split(':') {
            let library_path = Path::new(ament_prefix_path).join("lib");
            println!("cargo:rustc-link-search=native={}", library_path.display());
        }
    }

    // Invalidate the built crate whenever this script changes
    println!("cargo:rerun-if-changed=build.rs");
}
