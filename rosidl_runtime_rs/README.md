# Common types and traits for ROS 2 messages in Rust

ROS 2 is a popular open source robotics framework, used in a variety of fields (self-driving cars, drones, humanoid robots, etc.). `rosidl_runtime_rs` is a library that is mainly used by generated code for ROS 2 messages.

Please see the docs in the [`ros2_rust` repo](https://github.com/ros2-rust/ros2_rust).

## Buffer-backed messages

Generated `msg::buffer`, `srv::buffer`, and `action::buffer` representations
use storage from `rosidl_buffer_rs`, re-exported by this crate. Ordinary message
representations retain CPU fields. `Buffer::as_slice()` borrows CPU storage;
`to_vec()` copies backend data to the host. Services and actions materialize
accelerator fields on the CPU before serialization.

Buffer-enabled generated crates select the `rosidl-buffer` Cargo feature.
Without it, the runtime uses CPU sequences and does not link `rosidl_buffer`.
Native sequence layouts are selected from the installed C headers; enabling
`rosidl-buffer` against a legacy ABI is rejected at build time.
