#![warn(missing_docs)]
//! Bindings to `rosidl_runtime_c` and related functionality for messages.

#[macro_use]
mod sequence;
#[cfg(feature = "rosidl-buffer")]
pub use native::BufferError;
pub use sequence::{BoundedSequence, Sequence, SequenceExceedsBoundsError};

mod string;
pub use string::{BoundedString, BoundedWString, String, StringExceedsBoundsError, WString};

mod traits;
pub use traits::*;

/// Whether this runtime can convert native backend storage to CPU messages.
#[doc(hidden)]
pub const BUFFER_SUPPORT_ENABLED: bool = cfg!(feature = "rosidl-buffer");

#[cfg(feature = "rosidl-buffer")]
#[doc(hidden)]
pub mod native;

pub use SequenceAlloc as PrimitiveSequenceAlloc;
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
