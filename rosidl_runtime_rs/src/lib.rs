#![warn(missing_docs)]
//! Bindings to `rosidl_runtime_c` and related functionality for messages.

#[macro_use]
mod sequence;
#[cfg(feature = "rosidl-buffer")]
pub use rosidl_buffer_rs::{
    BoundedBuffer, BoundedPrimitiveSequence, BoundedVec, Buffer, BufferError, PrimitiveSequence,
    PrimitiveSequenceAlloc, SequenceExceedsBoundsError,
};
pub use sequence::{BoundedSequence, Sequence};

mod string;
pub use string::{BoundedString, BoundedWString, String, StringExceedsBoundsError, WString};

mod traits;
pub use traits::*;

#[cfg(feature = "rosidl-buffer")]
#[doc(hidden)]
pub use rosidl_buffer_rs::native;

#[cfg(not(feature = "rosidl-buffer"))]
pub use sequence::SequenceExceedsBoundsError;
#[cfg(not(feature = "rosidl-buffer"))]
pub use SequenceAlloc as PrimitiveSequenceAlloc;
#[cfg(not(feature = "rosidl-buffer"))]
pub use {BoundedSequence as BoundedPrimitiveSequence, Sequence as PrimitiveSequence};
/// CPU-only conversions cannot produce backend transfer errors.
#[cfg(not(feature = "rosidl-buffer"))]
pub type BufferError = std::convert::Infallible;

// Generated sources may be included by ros-env without their Cargo manifests.
// Select buffer-only items using this runtime's feature, not the including crate's.
#[cfg(feature = "rosidl-buffer")]
#[doc(hidden)]
#[macro_export]
macro_rules! cfg_buffer {
    ($($item:item)*) => { $($item)* };
}

#[cfg(not(feature = "rosidl-buffer"))]
#[doc(hidden)]
#[macro_export]
macro_rules! cfg_buffer {
    ($($item:item)*) => {};
}

// String sequences inherit this layout from the installed C primitive-sequence macro.
#[cfg(rosidl_buffer_abi)]
#[doc(hidden)]
pub type NativeSequenceMetadata = [bool; 2];
#[cfg(not(rosidl_buffer_abi))]
#[doc(hidden)]
pub type NativeSequenceMetadata = ();
