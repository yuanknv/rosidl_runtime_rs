// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

use std::{borrow::Borrow, mem::ManuallyDrop};

use super::{BoundedSequence, Sequence, SequenceAlloc};
use crate::{
    BoundedBuffer, BoundedPrimitiveSequence, Buffer, BufferError, PrimitiveSequence,
    PrimitiveSequenceAlloc,
};

impl<T> Sequence<T>
where
    T: SequenceAlloc<SequenceMetadata = [bool; 2]> + PrimitiveSequenceAlloc,
{
    fn native(&self) -> &PrimitiveSequence<T> {
        // SAFETY: both repr(C) sequences have data, size, capacity, and two
        // initialized bool flags in that order. The metadata bound excludes
        // message sequences; PrimitiveSequenceAlloc excludes string sequences.
        unsafe { &*(self as *const Self).cast::<PrimitiveSequence<T>>() }
    }

    fn native_mut(&mut self) -> &mut PrimitiveSequence<T> {
        // SAFETY: the same layout as native(), with an exclusive borrow.
        unsafe { &mut *(self as *mut Self).cast::<PrimitiveSequence<T>>() }
    }
}

/// Borrows the backend primitive representation without copying storage.
impl<T> Borrow<PrimitiveSequence<T>> for Sequence<T>
where
    T: SequenceAlloc<SequenceMetadata = [bool; 2]> + PrimitiveSequenceAlloc,
{
    fn borrow(&self) -> &PrimitiveSequence<T> {
        self.native()
    }
}

impl<T> From<PrimitiveSequence<T>> for Sequence<T>
where
    T: SequenceAlloc<SequenceMetadata = [bool; 2]> + PrimitiveSequenceAlloc,
{
    fn from(sequence: PrimitiveSequence<T>) -> Self {
        let sequence = ManuallyDrop::new(sequence);
        // SAFETY: native() establishes the common layout. Suppressing the
        // source destructor transfers its sole ownership to the returned value.
        unsafe { std::ptr::read((&*sequence as *const PrimitiveSequence<T>).cast::<Self>()) }
    }
}

impl<T> From<Sequence<T>> for PrimitiveSequence<T>
where
    T: SequenceAlloc<SequenceMetadata = [bool; 2]> + PrimitiveSequenceAlloc,
{
    fn from(sequence: Sequence<T>) -> Self {
        let sequence = ManuallyDrop::new(sequence);
        // SAFETY: inverse of the ownership transfer above, with identical layout.
        unsafe { std::ptr::read((&*sequence as *const Sequence<T>).cast::<Self>()) }
    }
}

impl<T> From<Buffer<T>> for Sequence<T>
where
    T: SequenceAlloc<SequenceMetadata = [bool; 2]> + PrimitiveSequenceAlloc,
{
    fn from(buffer: Buffer<T>) -> Self {
        buffer.into_sequence().into()
    }
}

impl<T> From<Sequence<T>> for Buffer<T>
where
    T: SequenceAlloc<SequenceMetadata = [bool; 2]> + PrimitiveSequenceAlloc,
{
    fn from(sequence: Sequence<T>) -> Self {
        PrimitiveSequence::from(sequence).into()
    }
}

impl<T, const N: usize> From<BoundedPrimitiveSequence<T, N>> for BoundedSequence<T, N>
where
    T: SequenceAlloc<SequenceMetadata = [bool; 2]> + PrimitiveSequenceAlloc,
{
    fn from(sequence: BoundedPrimitiveSequence<T, N>) -> Self {
        Self {
            inner: sequence.into_unbounded().into(),
        }
    }
}

impl<T, const N: usize> From<BoundedSequence<T, N>> for BoundedBuffer<T, N>
where
    T: SequenceAlloc<SequenceMetadata = [bool; 2]> + PrimitiveSequenceAlloc,
{
    fn from(mut sequence: BoundedSequence<T, N>) -> Self {
        let buffer = Buffer::from(std::mem::take(&mut sequence.inner));
        buffer
            .try_into()
            .expect("native sequence respects its IDL bound")
    }
}

impl<T: SequenceAlloc> Sequence<T> {
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
}

impl Sequence<u8> {
    /// Borrows the native Buffer pointer without transferring ownership.
    pub fn rosidl_buffer_ptr(&self) -> Option<*mut std::ffi::c_void> {
        self.native().rosidl_buffer_ptr()
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
}

macro_rules! impl_native_sequence_alloc {
    ($($ty:ty),+ $(,)?) => {$(
        impl SequenceAlloc for $ty {
            type SequenceMetadata = [bool; 2];

            fn sequence_init(seq: &mut Sequence<Self>, size: usize) -> bool {
                Self::primitive_sequence_init(seq.native_mut(), size)
            }

            fn sequence_fini(seq: &mut Sequence<Self>) {
                Self::primitive_sequence_fini(seq.native_mut());
            }

            fn sequence_copy(input: &Sequence<Self>, output: &mut Sequence<Self>) -> bool {
                Self::primitive_sequence_copy(input.native(), output.native_mut())
            }

            fn sequence_is_rosidl_buffer(seq: &Sequence<Self>) -> bool {
                seq.native().is_rosidl_buffer()
            }

            fn sequence_are_equal(lhs: &Sequence<Self>, rhs: &Sequence<Self>) -> Option<bool> {
                Some(Self::primitive_sequence_are_equal(lhs.native(), rhs.native()))
            }

            fn sequence_to_cpu(seq: &Sequence<Self>) -> Result<Option<Sequence<Self>>, BufferError> {
                if seq.is_rosidl_buffer() {
                    let values = seq.native().try_to_vec()?;
                    let mut cpu = PrimitiveSequence::try_new(values.len())?;
                    cpu.as_mut_slice().copy_from_slice(&values);
                    Ok(Some(cpu.into()))
                } else {
                    Ok(None)
                }
            }
        }
    )+};
}

impl_native_sequence_alloc!(bool, u8, i8, u16, i16, u32, i32, u64, i64, f32, f64);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn opaque_sequence_preserves_ownership_and_materializes_explicitly() {
        let buffer =
            crate::native::into_buffer(crate::native::ffi::create_cpu(&[3, 5, 7]).unwrap())
                .unwrap();
        let pointer = buffer.as_sequence().rosidl_buffer_ptr();
        let sequence: Sequence<u8> = buffer.into();
        assert_eq!(sequence.rosidl_buffer_ptr(), pointer);
        let borrowed: &PrimitiveSequence<u8> = sequence.borrow();
        assert_eq!(borrowed.rosidl_buffer_ptr(), pointer);
        assert_eq!(sequence.len(), 3);
        assert!(format!("{sequence:?}").contains("is_rosidl_buffer"));
        assert!(std::panic::catch_unwind(|| sequence.as_slice()).is_err());
        assert_eq!(sequence.try_to_vec().unwrap(), [3, 5, 7]);
        #[cfg(feature = "serde")]
        assert_eq!(serde_json::to_string(&sequence).unwrap(), "[3,5,7]");
        let cloned = sequence.clone();
        assert_eq!(sequence, cloned);
        let buffer = Buffer::from(sequence);
        assert_eq!(buffer.as_sequence().rosidl_buffer_ptr(), pointer);
        drop(buffer);
        assert_eq!(cloned.try_into_cpu().unwrap().as_slice(), [3, 5, 7]);
    }
}
