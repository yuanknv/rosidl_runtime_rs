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
    #[cfg(not(feature = "use_ros_shim"))]
    {
        let ament_prefix_path_list = get_env_var_or_abort(AMENT_PREFIX_PATH);
        for library in ["rosidl_runtime_c", "rosidl_buffer"] {
            let library_path = env::split_paths(&ament_prefix_path_list)
                .map(|prefix| Path::new(&prefix).join("lib"))
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
        }
    }

    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-changed=build.rs");
}
