// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

//! Native ROS buffer bindings shared by runtime sequences and buffer APIs.

pub use cxx::UniquePtr;
use std::fmt;

#[cxx::bridge(namespace = "rosidl_buffer_rs")]
pub mod ffi {
    // SAFETY: references borrow live native objects, slices carry their bounds,
    // and fallible C++ operations translate exceptions into Result.
    unsafe extern "C++" {
        include!("rosidl_runtime_rs/src/buffer_bridge.hpp");

        type CxxBuffer;

        fn size(self: &CxxBuffer) -> usize;
        fn create_cpu(data: &[u8]) -> Result<UniquePtr<CxxBuffer>>;
        fn clone_buffer(buffer: &CxxBuffer, error_code: &mut i32) -> Result<UniquePtr<CxxBuffer>>;
        fn are_equal(lhs: &CxxBuffer, rhs: &CxxBuffer, error_code: &mut i32) -> Result<bool>;
        fn equals_data(buffer: &CxxBuffer, data: &[u8], error_code: &mut i32) -> Result<bool>;
        fn backend_name(buffer: &CxxBuffer, error_code: &mut i32) -> Result<String>;
        fn copy_to_host(buffer: &CxxBuffer, output: &mut [u8], error_code: &mut i32) -> Result<()>;
    }
}

pub use ffi::CxxBuffer;

/// A buffer allocation or transfer failure.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BufferError {
    /// A native buffer operation returned a failure code.
    Native {
        /// Operation that failed.
        operation: &'static str,
        /// Native return code.
        code: i32,
    },
    /// Host allocation failed.
    Allocation(std::string::String),
    /// The native ABI does not support opaque storage for this element type.
    UnsupportedElement,
}

impl fmt::Display for BufferError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Native { operation, code } => write!(f, "{operation} failed (code {code})"),
            Self::Allocation(message) => f.write_str(message),
            Self::UnsupportedElement => f.write_str("opaque storage requires a uint8 sequence"),
        }
    }
}
impl std::error::Error for BufferError {}
