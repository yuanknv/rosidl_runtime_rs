// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

//! Backend-neutral message storage.

use std::ffi::{c_char, c_void};
use std::fmt;

use crate::{
    BoundedPrimitiveSequence, PrimitiveSequence, PrimitiveSequenceAlloc, SequenceExceedsBoundsError,
};

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

fn check(operation: &'static str, code: i32) -> Result<(), BufferError> {
    if code == 0 {
        Ok(())
    } else {
        Err(BufferError::Native { operation, code })
    }
}

#[link(name = "rosidl_buffer")]
unsafe extern "C" {
    fn rosidl_buffer_uint8_backend_name(
        buffer: *const c_void,
        output: *mut c_char,
        capacity: usize,
        required: *mut usize,
    ) -> i32;
    fn rosidl_buffer_uint8_copy_to(buffer: *const c_void, output: *mut u8, size: usize) -> i32;
}

pub(crate) fn copy_to_host(pointer: *const c_void, len: usize) -> Result<Vec<u8>, BufferError> {
    let mut values = Vec::new();
    values
        .try_reserve_exact(len)
        .map_err(|error| BufferError::Allocation(error.to_string()))?;
    values.resize(len, 0);
    // SAFETY: the caller borrows a live native byte buffer; output owns len bytes.
    check("copy_to_host", unsafe {
        rosidl_buffer_uint8_copy_to(pointer, values.as_mut_ptr(), len)
    })?;
    Ok(values)
}

/// Owned sequence storage on a CPU or accelerator backend.
///
/// `as_slice` exposes contiguous CPU storage. `to_vec` copies any supported
/// backend to the host. Native accelerator storage is supported for `u8`.
///
/// ```
/// use rosidl_runtime_rs::Buffer;
///
/// let data = Buffer::from(vec![1u8, 2, 3]);
/// assert_eq!(data.as_slice(), Some(&[1, 2, 3][..]));
/// ```
#[derive(Clone, Default, PartialEq)]
pub struct Buffer<T: PrimitiveSequenceAlloc> {
    sequence: PrimitiveSequence<T>,
}

impl<T: PrimitiveSequenceAlloc> Buffer<T> {
    /// Number of elements.
    pub fn len(&self) -> usize {
        self.sequence.len()
    }
    /// Whether there are no elements.
    pub fn is_empty(&self) -> bool {
        self.sequence.is_empty()
    }
    /// Returns a slice for contiguous CPU storage.
    pub fn as_slice(&self) -> Option<&[T]> {
        (!self.sequence.is_rosidl_buffer()).then(|| self.sequence.as_slice())
    }
    /// Returns exclusive access to contiguous CPU storage.
    pub fn as_mut_slice(&mut self) -> Option<&mut [T]> {
        if self.sequence.is_rosidl_buffer() {
            None
        } else {
            Some(self.sequence.as_mut_slice())
        }
    }
    /// Copies elements to host memory through the selected backend.
    pub fn to_vec(&self) -> Result<Vec<T>, BufferError> {
        self.sequence.try_to_vec()
    }
    /// Returns the storage backend, such as `cpu` or `cuda`.
    pub fn backend_name(&self) -> Result<std::string::String, BufferError> {
        let Some(pointer) = self.sequence.opaque_ptr() else {
            return Ok("cpu".into());
        };
        let mut size = 0;
        // SAFETY: sequence retains the native Buffer through both queries.
        check("backend_name", unsafe {
            rosidl_buffer_uint8_backend_name(pointer, std::ptr::null_mut(), 0, &mut size)
        })?;
        let mut bytes = vec![0; size];
        check("backend_name", unsafe {
            rosidl_buffer_uint8_backend_name(
                pointer,
                bytes.as_mut_ptr().cast(),
                bytes.len(),
                &mut size,
            )
        })?;
        if bytes.last() != Some(&0) {
            return Err(BufferError::Native {
                operation: "backend_name",
                code: -1,
            });
        }
        bytes.pop();
        std::string::String::from_utf8(bytes).map_err(|_| BufferError::Native {
            operation: "backend_name",
            code: -1,
        })
    }
    /// Borrows the native transport sequence without transferring ownership.
    pub fn as_sequence(&self) -> &PrimitiveSequence<T> {
        &self.sequence
    }
    /// Transfers storage into its native transport sequence without copying.
    pub fn into_sequence(self) -> PrimitiveSequence<T> {
        self.sequence
    }
    /// Clones storage through the backend and reports allocation failures.
    pub fn try_clone(&self) -> Result<Self, BufferError> {
        let mut sequence = PrimitiveSequence::default();
        if !T::primitive_sequence_copy(&self.sequence, &mut sequence) {
            return Err(BufferError::Native {
                operation: "clone",
                code: -1,
            });
        }
        Ok(Self { sequence })
    }
}
impl<T: PrimitiveSequenceAlloc> From<PrimitiveSequence<T>> for Buffer<T> {
    fn from(sequence: PrimitiveSequence<T>) -> Self {
        Self { sequence }
    }
}
impl<T: PrimitiveSequenceAlloc> From<Vec<T>> for Buffer<T> {
    fn from(values: Vec<T>) -> Self {
        Self {
            sequence: values.into(),
        }
    }
}
impl<T: PrimitiveSequenceAlloc> From<&[T]> for Buffer<T> {
    fn from(values: &[T]) -> Self {
        Self {
            sequence: values.into(),
        }
    }
}
impl<T: PrimitiveSequenceAlloc> fmt::Debug for Buffer<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Buffer")
            .field("len", &self.len())
            .field("backend", &self.backend_name())
            .finish()
    }
}
impl<T: PrimitiveSequenceAlloc + PartialOrd> PartialOrd for Buffer<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.to_vec().ok()?.partial_cmp(&other.to_vec().ok()?)
    }
}

/// Buffer storage with an IDL sequence bound.
#[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct BoundedBuffer<T: PrimitiveSequenceAlloc, const N: usize>(Buffer<T>);
impl<T: PrimitiveSequenceAlloc, const N: usize> BoundedBuffer<T, N> {
    /// Borrows the underlying buffer.
    pub fn as_buffer(&self) -> &Buffer<T> {
        &self.0
    }
    /// Returns exclusive access to contiguous CPU elements without changing the bound.
    pub fn as_mut_slice(&mut self) -> Option<&mut [T]> {
        self.0.as_mut_slice()
    }
    /// Transfers ownership of the underlying buffer.
    pub fn into_buffer(self) -> Buffer<T> {
        self.0
    }
    /// Returns the native sequence without copying storage.
    pub fn into_sequence(self) -> BoundedPrimitiveSequence<T, N> {
        BoundedPrimitiveSequence::try_from_unbounded(self.0.into_sequence())
            .expect("validated buffer bound")
    }
}
impl<T: PrimitiveSequenceAlloc, const N: usize> std::ops::Deref for BoundedBuffer<T, N> {
    type Target = Buffer<T>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<T: PrimitiveSequenceAlloc, const N: usize> TryFrom<Buffer<T>> for BoundedBuffer<T, N> {
    type Error = SequenceExceedsBoundsError;
    fn try_from(buffer: Buffer<T>) -> Result<Self, Self::Error> {
        if buffer.len() > N {
            return Err(SequenceExceedsBoundsError {
                len: buffer.len(),
                upper_bound: N,
            });
        }
        Ok(Self(buffer))
    }
}
impl<T: PrimitiveSequenceAlloc, const N: usize> TryFrom<Vec<T>> for BoundedBuffer<T, N> {
    type Error = SequenceExceedsBoundsError;
    fn try_from(values: Vec<T>) -> Result<Self, Self::Error> {
        if values.len() > N {
            return Err(SequenceExceedsBoundsError {
                len: values.len(),
                upper_bound: N,
            });
        }
        Ok(Self(values.into()))
    }
}
impl<T: PrimitiveSequenceAlloc, const N: usize> From<BoundedPrimitiveSequence<T, N>>
    for BoundedBuffer<T, N>
{
    fn from(sequence: BoundedPrimitiveSequence<T, N>) -> Self {
        Self(sequence.into_unbounded().into())
    }
}

/// A Rust vector with an IDL sequence bound.
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BoundedVec<T, const N: usize>(Vec<T>);
impl<T, const N: usize> Default for BoundedVec<T, N> {
    fn default() -> Self {
        Self(Vec::new())
    }
}
impl<T, const N: usize> TryFrom<Vec<T>> for BoundedVec<T, N> {
    type Error = SequenceExceedsBoundsError;
    fn try_from(values: Vec<T>) -> Result<Self, Self::Error> {
        if values.len() > N {
            return Err(SequenceExceedsBoundsError {
                len: values.len(),
                upper_bound: N,
            });
        }
        Ok(Self(values))
    }
}
impl<T, const N: usize> std::ops::Deref for BoundedVec<T, N> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<T, const N: usize> std::ops::DerefMut for BoundedVec<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl<T, const N: usize> IntoIterator for BoundedVec<T, N> {
    type Item = T;
    type IntoIter = std::vec::IntoIter<T>;
    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

#[cfg(feature = "serde")]
mod serde_impl {
    use super::*;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};
    impl<T: PrimitiveSequenceAlloc + Serialize> Serialize for Buffer<T> {
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            self.to_vec()
                .map_err(serde::ser::Error::custom)?
                .serialize(serializer)
        }
    }
    impl<'de, T: PrimitiveSequenceAlloc + Deserialize<'de>> Deserialize<'de> for Buffer<T> {
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            Ok(Vec::<T>::deserialize(deserializer)?.into())
        }
    }
    impl<T: PrimitiveSequenceAlloc + Serialize, const N: usize> Serialize for BoundedBuffer<T, N> {
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            self.0.serialize(serializer)
        }
    }
    impl<'de, T: PrimitiveSequenceAlloc + Deserialize<'de>, const N: usize> Deserialize<'de>
        for BoundedBuffer<T, N>
    {
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            Vec::<T>::deserialize(deserializer)?
                .try_into()
                .map_err(serde::de::Error::custom)
        }
    }
    impl<T: Serialize, const N: usize> Serialize for BoundedVec<T, N> {
        fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
            self.0.serialize(serializer)
        }
    }
    impl<'de, T: Deserialize<'de>, const N: usize> Deserialize<'de> for BoundedVec<T, N> {
        fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
            Vec::<T>::deserialize(deserializer)?
                .try_into()
                .map_err(serde::de::Error::custom)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_buffer_transfers_ownership_and_preserves_values() {
        let mut buffer = Buffer::from(vec![2u32, 4, 6]);
        assert_eq!(buffer.backend_name().unwrap(), "cpu");
        buffer.as_mut_slice().unwrap()[1] = 9;
        assert_eq!(buffer.to_vec().unwrap(), vec![2, 9, 6]);
        let pointer = buffer.as_slice().unwrap().as_ptr();
        let cloned = buffer.try_clone().unwrap();
        let sequence = buffer.into_sequence();
        assert_eq!(sequence.as_slice().as_ptr(), pointer);
        drop(sequence);
        assert_eq!(cloned.as_slice().unwrap(), &[2, 9, 6]);
        fn send_sync<T: Send + Sync>() {}
        send_sync::<Buffer<u8>>();
    }

    #[test]
    fn bounded_containers_enforce_idl_limits() {
        let buffer: BoundedBuffer<u8, 2> = vec![7, 9].try_into().unwrap();
        assert!(BoundedBuffer::<u8, 2>::try_from(vec![1, 2, 3]).is_err());
        let sequence = buffer.into_sequence();
        assert_eq!(sequence.as_slice(), &[7, 9]);
        assert_eq!(BoundedBuffer::from(sequence).len(), 2);
        let mut nested = BoundedVec::<_, 2>::try_from(vec![Buffer::from(vec![1u8])]).unwrap();
        nested[0] = Buffer::from(vec![2]);
        assert_eq!(
            nested.into_iter().next().unwrap().to_vec().unwrap(),
            vec![2]
        );
        assert!(BoundedVec::<u8, 2>::try_from(vec![1, 2, 3]).is_err());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_preserves_host_values_and_checks_bounds() {
        let buffer = Buffer::from(vec![1u8, 2]);
        assert_eq!(serde_json::to_string(&buffer).unwrap(), "[1,2]");
        let restored: Buffer<u8> = serde_json::from_str("[1,2]").unwrap();
        assert_eq!(restored, buffer);
        assert!(serde_json::from_str::<BoundedBuffer<u8, 1>>("[1,2]").is_err());
        assert!(serde_json::from_str::<BoundedVec<Buffer<u8>, 1>>("[[1],[2]]").is_err());
        let nested: BoundedVec<Buffer<u8>, 1> = serde_json::from_str("[[1,2]]").unwrap();
        assert_eq!(serde_json::to_string(&nested).unwrap(), "[[1,2]]");
    }
}
