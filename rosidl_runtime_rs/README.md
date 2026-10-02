# Common types and traits for ROS 2 messages in Rust

ROS 2 is a popular open source robotics framework, used in a variety of fields (self-driving cars, drones, humanoid robots, etc.). `rosidl_runtime_rs` is a library that is mainly used by generated code for ROS 2 messages.

Please see the docs in the [`ros2_rust` repo](https://github.com/ros2-rust/ros2_rust).

## Buffer-backed messages

- `Sequence<T>` and `BoundedSequence<T, N>` provide CPU sequence storage.
- `PrimitiveSequence<T>` and `BoundedPrimitiveSequence<T, N>` match the native
  transport layout, including Buffer ownership.
- `Buffer<T>` and `BoundedBuffer<T, N>` store CPU or accelerator data.
  `BoundedVec<T, N>` stores bounded sequences of nested buffer messages.

```rust
use rosidl_runtime_rs::Buffer;

let data = Buffer::from(vec![1u8, 2, 3]);
assert_eq!(data.as_slice(), Some(&[1, 2, 3][..]));
```

`as_slice()` borrows contiguous CPU storage. `to_vec()` copies to host memory
and waits for completion. Accelerator adapters provide allocation and device
access. Owned conversion into a native message transfers ownership; cloning
and borrowed publication can copy data. Serde serializes host values.

Generated `msg::Image` retains CPU fields; `msg::buffer::Image` uses buffer
fields with the same ROS type support. Services and actions also have `buffer`
modules. CPU conversion uses `try_from_rmw_message` or `try_into_cpu` to report
transfer errors. Services and actions convert accelerator fields to CPU storage
before serialization.

Build native and Rust interfaces with matching transport layouts. CPU message
source compatibility is preserved; raw RMW layouts require the rebuilt native ABI.
