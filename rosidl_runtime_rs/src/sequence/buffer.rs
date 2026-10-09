// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

use super::{BoundedSequence, Sequence, SequenceAlloc, SequenceExceedsBoundsError};
use crate::native::{ffi, CxxBuffer};
use crate::BufferError;

impl<T: SequenceAlloc> Sequence<T> {
    /// Allocates elements, reporting native allocation failure.
    pub fn try_new(len: usize) -> Result<Self, BufferError> {
        let mut seq = Self::default();
        if !T::sequence_init(&mut seq, len) {
            return Err(BufferError::Allocation("sequence allocation failed".into()));
        }
        Ok(seq)
    }

    /// Copies contents to host memory, including opaque backend storage.
    pub fn try_to_vec(&self) -> Result<Vec<T>, BufferError>
    where
        T: Clone,
    {
        match T::sequence_to_cpu(self)? {
            Some(cpu) => Ok(cpu.as_slice().to_vec()),
            None => Ok(self.as_slice().to_vec()),
        }
    }

    /// Materializes opaque storage; ordinary CPU storage keeps its allocation.
    pub fn try_into_cpu(self) -> Result<Self, BufferError> {
        Ok(T::sequence_to_cpu(&self)?.unwrap_or(self))
    }

    /// Borrows an opaque backend object, if present.
    #[doc(hidden)]
    pub fn opaque_ptr(&self) -> Option<*mut std::ffi::c_void> {
        self.is_rosidl_buffer().then_some(self.data.cast())
    }

    /// Takes ownership of CPU storage allocated by the matching C allocator.
    ///
    /// # Safety
    /// The parts must describe a live, exclusively owned C sequence allocation,
    /// with `size <= capacity` and `size` initialized elements of this type.
    pub unsafe fn from_cpu_raw_parts(data: *mut T, size: usize, capacity: usize) -> Self {
        Self {
            data,
            size,
            capacity,
            metadata: std::mem::MaybeUninit::zeroed(),
        }
    }

    /// Releases CPU storage to the caller, who must use the matching C allocator.
    pub fn into_cpu_raw_parts(self) -> Result<(*mut T, usize, usize), Self> {
        if self.is_rosidl_buffer() {
            return Err(self);
        }
        let sequence = std::mem::ManuallyDrop::new(self);
        Ok((sequence.data, sequence.size, sequence.capacity))
    }
}

impl Sequence<u8> {
    /// Takes ownership of an opaque native byte buffer.
    ///
    /// # Safety
    /// `buffer` must point to a live `rosidl::Buffer<uint8_t>` with `size` bytes.
    /// No other owner may delete that object after this call succeeds.
    pub unsafe fn from_owned_rosidl_buffer(
        buffer: *mut std::ffi::c_void,
        size: usize,
    ) -> Option<Self> {
        if buffer.is_null() {
            return None;
        }
        Some(Self {
            data: buffer.cast(),
            size,
            capacity: size,
            metadata: std::mem::MaybeUninit::new([true, true]),
        })
    }

    /// Borrows the native buffer pointer without transferring ownership.
    pub fn rosidl_buffer_ptr(&self) -> Option<*mut std::ffi::c_void> {
        self.opaque_ptr()
    }

    /// Releases an owned opaque buffer; CPU and non-owning sequences are returned unchanged.
    pub fn into_owned_rosidl_buffer(mut self) -> Result<*mut std::ffi::c_void, Self> {
        // SAFETY: byte sequence constructors and native deserialization initialize both flags.
        if !self.is_rosidl_buffer() || !unsafe { self.metadata.assume_init_ref()[1] } {
            return Err(self);
        }
        let pointer = self.data.cast();
        self.data = std::ptr::null_mut();
        self.size = 0;
        self.capacity = 0;
        self.metadata = std::mem::MaybeUninit::new([false, false]);
        Ok(pointer)
    }
}

impl<T: SequenceAlloc, const N: usize> BoundedSequence<T, N> {
    /// Materializes opaque storage without changing the IDL bound.
    pub fn try_into_cpu(mut self) -> Result<Self, BufferError> {
        if let Some(cpu) = T::sequence_to_cpu(&self.inner)? {
            self.inner = cpu;
        }
        Ok(self)
    }

    /// Transfers the underlying sequence without copying its storage.
    pub fn into_unbounded(mut self) -> Sequence<T> {
        std::mem::take(&mut self.inner)
    }

    /// Applies an IDL bound without copying storage.
    pub fn try_from_unbounded(inner: Sequence<T>) -> Result<Self, SequenceExceedsBoundsError> {
        if inner.len() > N {
            return Err(SequenceExceedsBoundsError {
                len: inner.len(),
                upper_bound: N,
            });
        }
        Ok(Self { inner })
    }
}

pub(super) fn copy_sequence(source: &Sequence<u8>, target: &mut Sequence<u8>) -> Option<bool> {
    if !source.is_rosidl_buffer() && !target.is_rosidl_buffer() {
        return None;
    }
    let replacement = if let Some(pointer) = source.rosidl_buffer_ptr() {
        // SAFETY: source keeps the native object alive during cloning.
        let buffer = unsafe { &*pointer.cast::<CxxBuffer>() };
        let mut code = 0;
        let Ok(clone) = ffi::clone_buffer(buffer, &mut code) else {
            return Some(false);
        };
        let Some(buffer) = clone.as_ref() else {
            return Some(false);
        };
        let size = buffer.size();
        // SAFETY: the non-null unique pointer transfers its sole ownership.
        unsafe { Sequence::from_owned_rosidl_buffer(clone.into_raw().cast(), size).unwrap() }
    } else {
        let mut replacement = Sequence::default();
        if !u8::sequence_copy(source, &mut replacement) {
            return Some(false);
        }
        replacement
    };
    *target = replacement;
    Some(true)
}

pub(super) fn sequences_equal(lhs: &Sequence<u8>, rhs: &Sequence<u8>) -> Option<bool> {
    // SAFETY: pointers borrow live native objects owned by the input sequences.
    let lhs_buffer = lhs
        .rosidl_buffer_ptr()
        .map(|p| unsafe { &*p.cast::<CxxBuffer>() });
    let rhs_buffer = rhs
        .rosidl_buffer_ptr()
        .map(|p| unsafe { &*p.cast::<CxxBuffer>() });
    let mut code = 0;
    let result = match (lhs_buffer, rhs_buffer) {
        (None, None) => return None,
        (Some(lhs), Some(rhs)) => ffi::are_equal(lhs, rhs, &mut code),
        (Some(lhs), None) => ffi::equals_data(lhs, rhs.as_slice(), &mut code),
        (None, Some(rhs)) => ffi::equals_data(rhs, lhs.as_slice(), &mut code),
    };
    Some(result.unwrap_or(false))
}

pub(super) fn sequence_to_cpu(seq: &Sequence<u8>) -> Result<Option<Sequence<u8>>, BufferError> {
    let Some(pointer) = seq.rosidl_buffer_ptr() else {
        return Ok(None);
    };
    let mut cpu = Sequence::try_new(seq.len())?;
    // SAFETY: seq retains the live byte buffer for this synchronous transfer.
    let buffer = unsafe { &*pointer.cast::<CxxBuffer>() };
    let mut code = 0;
    ffi::copy_to_host(buffer, cpu.as_mut_slice(), &mut code).map_err(|_| BufferError::Native {
        operation: "copy_to_host",
        code,
    })?;
    Ok(Some(cpu))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn opaque_sequence_preserves_ownership_and_materializes_explicitly() {
        let owner = ffi::create_cpu(&[3, 5, 7]).unwrap();
        let pointer = owner.into_raw().cast();
        // SAFETY: the unique pointer transferred the only owning reference.
        let sequence = unsafe { Sequence::from_owned_rosidl_buffer(pointer, 3).unwrap() };
        assert_eq!(sequence.rosidl_buffer_ptr(), Some(pointer));
        assert_eq!(sequence.len(), 3);
        assert!(format!("{sequence:?}").contains("is_rosidl_buffer"));
        assert!(std::panic::catch_unwind(|| sequence.as_slice()).is_err());
        assert_eq!(sequence.try_to_vec().unwrap(), [3, 5, 7]);
        #[cfg(feature = "serde")]
        assert_eq!(serde_json::to_string(&sequence).unwrap(), "[3,5,7]");
        let cloned = sequence.clone();
        assert_eq!(sequence, cloned);
        drop(sequence);
        assert_eq!(cloned.try_into_cpu().unwrap().as_slice(), [3, 5, 7]);
    }
}
