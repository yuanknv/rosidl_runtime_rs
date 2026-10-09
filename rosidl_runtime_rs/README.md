# Common types and traits for ROS 2 messages in Rust

ROS 2 is a popular open source robotics framework, used in a variety of fields (self-driving cars, drones, humanoid robots, etc.). `rosidl_runtime_rs` is a library that is mainly used by generated code for ROS 2 messages.

Please see the docs in the [`ros2_rust` repo](https://github.com/ros2-rust/ros2_rust).

## Buffer-backed messages

Generated `msg::buffer`, `srv::buffer`, and `action::buffer` representations
use `rosidl_buffer_rs::Buffer<T>` storage. This crate provides `Sequence<T>`
for native message fields and optional CXX bindings for buffer operations.
Ordinary message representations retain CPU fields. `Buffer::as_slice()` borrows CPU storage;
`to_vec()` copies backend data to the host. Services and actions materialize
accelerator fields on the CPU before serialization.

Buffer-enabled generated crates select the `rosidl-buffer` Cargo feature.
When using generated interfaces through `ros-env`, enable `ros-env/rosidl-buffer`
(also enabled by `rclrs/rosidl-buffer`) to expose their buffer representations;
CPU representations work without it.
Without it, the runtime uses CPU sequences and does not link `rosidl_buffer`.
Native sequence layouts are selected from the installed C headers; enabling
`rosidl-buffer` against a legacy ABI is rejected at build time.
The buffer feature requires Rust 1.88+ and a C++20 compiler.
