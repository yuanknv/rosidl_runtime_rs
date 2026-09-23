#![warn(missing_docs)]
#![doc = include_str!("README.md")]
//! Bindings to `rosidl_runtime_c` and related functionality for messages.

#[macro_use]
mod sequence;
pub use sequence::{
    BoundedPrimitiveSequence, BoundedSequence, PrimitiveSequence, Sequence,
    SequenceExceedsBoundsError,
};

mod string;
pub use string::{BoundedString, BoundedWString, String, StringExceedsBoundsError, WString};

mod traits;
pub use traits::*;

mod buffer;
pub use buffer::{BoundedBuffer, BoundedVec, Buffer, BufferError};
