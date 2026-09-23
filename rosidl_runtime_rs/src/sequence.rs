use std::{
    cmp::Ordering,
    fmt::{self, Debug, Display},
    hash::{Hash, Hasher},
    iter::{Extend, FromIterator, FusedIterator},
    ops::{Deref, DerefMut},
};

#[cfg(feature = "serde")]
mod serde;

use crate::traits::{PrimitiveSequenceAlloc, SequenceAlloc};

/// An unbounded sequence.
///
/// For message and string elements, the layout matches the corresponding native
/// sequence. Primitive CPU elements retain the legacy three-field layout; native
/// primitive transport fields use [`PrimitiveSequence`]. For instance,
/// `rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>` is the same
/// as `std_msgs__msg__String__Sequence`. See the [`Message`](crate::Message) trait for background
/// information on this topic.
///
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::{Sequence, String, seq};
/// let mut list = Sequence::<String>::new(3);
/// // Sequences deref to slices
/// assert_eq!(list.len(), 3);
/// list[0] = "three".into();
/// // Alternatively, use the seq! macro
/// list = seq!["three".into(), "two".into(), "one".into()];
/// // The default sequence is empty
/// assert!(Sequence::<String>::default().is_empty());
/// ```
#[repr(C)]
pub struct Sequence<T: SequenceAlloc> {
    data: *mut T,
    size: usize,
    capacity: usize,
}

/// A bounded sequence.
///
/// Message and string elements use the corresponding native sequence layout.
/// Primitive CPU elements retain the legacy layout; native primitive transport
/// fields use [`BoundedPrimitiveSequence`]. For instance,
/// `rosidl_runtime_rs::BoundedSequence<rosidl_runtime_rs::String, 5>`
/// is the same as `std_msgs__msg__String__Sequence`, which also represents both bounded
/// sequences.  See the [`Message`](crate::Message) trait for background information on this
/// topic.
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::{BoundedSequence, String, seq};
/// let mut list = BoundedSequence::<String, 5>::new(3);
/// // BoundedSequences deref to slices
/// assert_eq!(list.len(), 3);
/// list[0] = "three".into();
/// // Alternatively, use the seq! macro with the length specifier
/// list = seq![5 # "three".into(), "two".into(), "one".into()];
/// // The default bounded sequence is empty
/// assert!(BoundedSequence::<String, 5>::default().is_empty());
/// ```
#[derive(Clone)]
#[repr(transparent)]
pub struct BoundedSequence<T: SequenceAlloc, const N: usize> {
    inner: Sequence<T>,
}

/// Error type for [`BoundedSequence::try_new()`].
#[derive(Debug)]
pub struct SequenceExceedsBoundsError {
    /// The actual length the sequence would have after the operation.
    pub len: usize,
    /// The upper bound on the sequence length.
    pub upper_bound: usize,
}

/// A by-value iterator created by [`Sequence::into_iter()`] and [`BoundedSequence::into_iter()`].
pub struct SequenceIterator<T: SequenceAlloc> {
    seq: Sequence<T>,
    idx: usize,
}

impl<T: SequenceAlloc> Clone for Sequence<T> {
    fn clone(&self) -> Self {
        let mut seq = Self::default();
        if T::sequence_copy(self, &mut seq) {
            seq
        } else {
            panic!("Cloning Sequence failed")
        }
    }
}

impl<T: Debug + SequenceAlloc> Debug for Sequence<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.as_slice().fmt(f)
    }
}

impl<T: SequenceAlloc> Default for Sequence<T> {
    fn default() -> Self {
        Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        }
    }
}

impl<T: SequenceAlloc> Deref for Sequence<T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl<T: SequenceAlloc> DerefMut for Sequence<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl<T: SequenceAlloc> Drop for Sequence<T> {
    fn drop(&mut self) {
        T::sequence_fini(self)
    }
}

impl<T: SequenceAlloc + Eq> Eq for Sequence<T> {}

impl<T: SequenceAlloc> Extend<T> for Sequence<T> {
    fn extend<I>(&mut self, iter: I)
    where
        I: IntoIterator<Item = T>,
    {
        let it = iter.into_iter();
        // The index in the sequence where the next element will be stored
        let mut cur_idx = self.size;
        // Convenience closure for resizing self
        let resize = |seq: &mut Self, new_size: usize| {
            let old_seq = std::mem::replace(seq, Sequence::new(new_size));
            for (i, elem) in old_seq.into_iter().enumerate().take(new_size) {
                seq[i] = elem;
            }
        };
        // First, when there is a size hint > 0 (lower bound), make room for
        // that many elements.
        let num_remaining = it.size_hint().0;
        if num_remaining > 0 {
            let new_size = self
                .size
                .checked_add(num_remaining)
                .expect("sequence length overflow");
            resize(self, new_size);
        }
        for item in it {
            // If there is no more capacity for the next element, resize to the
            // next power of two.
            if cur_idx == self.size {
                let new_size = self
                    .size
                    .checked_add(1)
                    .and_then(usize::checked_next_power_of_two)
                    .expect("sequence length overflow");
                resize(self, new_size);
            }
            self[cur_idx] = item;
            cur_idx += 1;
        }
        // All items from the iterator are stored. Shrink the sequence to fit.
        if cur_idx < self.size {
            resize(self, cur_idx);
        }
    }
}

impl<T: SequenceAlloc + Clone> From<&[T]> for Sequence<T> {
    fn from(slice: &[T]) -> Self {
        let mut seq = Sequence::new(slice.len());
        seq.clone_from_slice(slice);
        seq
    }
}

impl<T: SequenceAlloc> From<Vec<T>> for Sequence<T> {
    fn from(v: Vec<T>) -> Self {
        Sequence::from_iter(v)
    }
}

/// Copies a C-owned sequence into a Rust vector.
impl<T: SequenceAlloc + Copy> From<Sequence<T>> for Vec<T> {
    fn from(seq: Sequence<T>) -> Self {
        seq.as_slice().to_vec()
    }
}

impl<T: SequenceAlloc> FromIterator<T> for Sequence<T> {
    fn from_iter<I>(iter: I) -> Self
    where
        I: IntoIterator<Item = T>,
    {
        let mut seq = Sequence::new(0);
        seq.extend(iter);
        seq
    }
}

impl<T: SequenceAlloc + Hash> Hash for Sequence<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.as_slice().hash(state)
    }
}

impl<T: SequenceAlloc> IntoIterator for Sequence<T> {
    type Item = T;
    type IntoIter = SequenceIterator<T>;
    fn into_iter(self) -> Self::IntoIter {
        SequenceIterator { seq: self, idx: 0 }
    }
}

impl<T: SequenceAlloc + Ord> Ord for Sequence<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.as_slice().cmp(other.as_slice())
    }
}

impl<T: SequenceAlloc + PartialEq> PartialEq for Sequence<T> {
    fn eq(&self, other: &Self) -> bool {
        self.as_slice().eq(other.as_slice())
    }
}

impl<T: SequenceAlloc + PartialOrd> PartialOrd for Sequence<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.as_slice().partial_cmp(other.as_slice())
    }
}

// SAFETY: A sequence is a simple data structure, and therefore not thread-specific.
unsafe impl<T: Send + SequenceAlloc> Send for Sequence<T> {}
// SAFETY: A sequence does not have interior mutability, so it can be shared.
unsafe impl<T: Sync + SequenceAlloc> Sync for Sequence<T> {}

impl<T> Sequence<T>
where
    T: SequenceAlloc,
{
    /// Creates a sequence of `len` elements with default values.
    pub fn new(len: usize) -> Self {
        let mut seq = Self::default();
        if !T::sequence_init(&mut seq, len) {
            panic!("Sequence initialization failed");
        }
        seq
    }

    /// Extracts a slice containing the entire sequence.
    ///
    /// Equivalent to `&seq[..]`.
    pub fn as_slice(&self) -> &[T] {
        if self.data.is_null() {
            &[]
        } else {
            // SAFETY: self.data is not null and points to self.size consecutive,
            // initialized elements and isn't modified externally.
            unsafe { std::slice::from_raw_parts(self.data, self.size) }
        }
    }

    /// Extracts a mutable slice containing the entire sequence.
    ///
    /// Equivalent to `&mut seq[..]`.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        if self.data.is_null() {
            &mut []
        } else {
            // SAFETY: self.data is not null and points to self.size consecutive,
            // initialized elements and isn't modified externally.
            unsafe { std::slice::from_raw_parts_mut(self.data, self.size) }
        }
    }
}

impl<T: Debug + SequenceAlloc, const N: usize> Debug for BoundedSequence<T, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        self.as_slice().fmt(f)
    }
}

impl<T: SequenceAlloc, const N: usize> Default for BoundedSequence<T, N> {
    fn default() -> Self {
        Self {
            inner: Sequence {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
        }
    }
}

impl<T: SequenceAlloc, const N: usize> Deref for BoundedSequence<T, N> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        self.inner.deref()
    }
}

impl<T: SequenceAlloc, const N: usize> DerefMut for BoundedSequence<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.deref_mut()
    }
}

impl<T: SequenceAlloc, const N: usize> Drop for BoundedSequence<T, N> {
    fn drop(&mut self) {
        T::sequence_fini(&mut self.inner)
    }
}

impl<T: SequenceAlloc + Eq, const N: usize> Eq for BoundedSequence<T, N> {}

impl<T: SequenceAlloc, const N: usize> Extend<T> for BoundedSequence<T, N> {
    fn extend<I>(&mut self, iter: I)
    where
        I: IntoIterator<Item = T>,
    {
        self.inner
            .extend(iter.into_iter().take(N - self.inner.size));
    }
}

impl<T: SequenceAlloc + Clone, const N: usize> TryFrom<&[T]> for BoundedSequence<T, N> {
    type Error = SequenceExceedsBoundsError;
    fn try_from(slice: &[T]) -> Result<Self, Self::Error> {
        let mut seq = BoundedSequence::try_new(slice.len())?;
        seq.clone_from_slice(slice);
        Ok(seq)
    }
}

impl<T: SequenceAlloc, const N: usize> TryFrom<Vec<T>> for BoundedSequence<T, N> {
    type Error = SequenceExceedsBoundsError;
    fn try_from(v: Vec<T>) -> Result<Self, Self::Error> {
        if v.len() > N {
            Err(SequenceExceedsBoundsError {
                len: v.len(),
                upper_bound: N,
            })
        } else {
            Ok(BoundedSequence::from_iter(v))
        }
    }
}

impl<T: SequenceAlloc, const N: usize> FromIterator<T> for BoundedSequence<T, N> {
    fn from_iter<I>(iter: I) -> Self
    where
        I: IntoIterator<Item = T>,
    {
        let mut seq = BoundedSequence::new(0);
        seq.extend(iter);
        seq
    }
}

impl<T: SequenceAlloc + Hash, const N: usize> Hash for BoundedSequence<T, N> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.as_slice().hash(state)
    }
}

impl<T: SequenceAlloc, const N: usize> IntoIterator for BoundedSequence<T, N> {
    type Item = T;
    type IntoIter = SequenceIterator<T>;
    fn into_iter(mut self) -> Self::IntoIter {
        let seq = std::mem::replace(
            &mut self.inner,
            Sequence {
                data: std::ptr::null_mut(),
                size: 0,
                capacity: 0,
            },
        );
        SequenceIterator { seq, idx: 0 }
    }
}

impl<T: SequenceAlloc + Ord, const N: usize> Ord for BoundedSequence<T, N> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.as_slice().cmp(other.as_slice())
    }
}

impl<T: SequenceAlloc + PartialEq, const N: usize> PartialEq for BoundedSequence<T, N> {
    fn eq(&self, other: &Self) -> bool {
        self.as_slice().eq(other.as_slice())
    }
}

impl<T: SequenceAlloc + PartialOrd, const N: usize> PartialOrd for BoundedSequence<T, N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.as_slice().partial_cmp(other.as_slice())
    }
}

impl<T, const N: usize> BoundedSequence<T, N>
where
    T: SequenceAlloc,
{
    /// Creates a sequence of `len` elements with default values.
    ///
    /// If `len` is greater than `N`, this function panics.
    pub fn new(len: usize) -> Self {
        Self::try_new(len).unwrap()
    }

    /// Attempts to create a sequence of `len` elements with default values.
    ///
    /// If `len` is greater than `N`, this function returns an error.
    pub fn try_new(len: usize) -> Result<Self, SequenceExceedsBoundsError> {
        if len > N {
            return Err(SequenceExceedsBoundsError {
                len,
                upper_bound: N,
            });
        }
        let mut seq = Self::default();
        if !T::sequence_init(&mut seq.inner, len) {
            panic!("BoundedSequence initialization failed");
        }
        Ok(seq)
    }

    /// Extracts a slice containing the entire sequence.
    ///
    /// Equivalent to `&seq[..]`.
    pub fn as_slice(&self) -> &[T] {
        self.inner.as_slice()
    }

    /// Extracts a mutable slice containing the entire sequence.
    ///
    /// Equivalent to `&mut seq[..]`.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        self.inner.as_mut_slice()
    }
}

impl<T: SequenceAlloc> Iterator for SequenceIterator<T> {
    type Item = T;
    fn next(&mut self) -> Option<Self::Item> {
        if self.idx >= self.seq.size {
            return None;
        }
        // SAFETY: data + idx is in bounds and points to a valid value
        let elem = unsafe {
            let ptr = self.seq.data.add(self.idx);
            let elem = ptr.read();
            // Need to make sure that dropping the sequence later will not fini() the elements
            ptr.write(std::mem::zeroed::<T>());
            elem
        };
        self.idx += 1;
        Some(elem)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let len = self.seq.size - self.idx;
        (len, Some(len))
    }
}

impl<T: SequenceAlloc> ExactSizeIterator for SequenceIterator<T> {
    fn len(&self) -> usize {
        self.seq.size - self.idx
    }
}

impl<T: SequenceAlloc> FusedIterator for SequenceIterator<T> {}

/// An ABI-compatible `rosidl_runtime_c` primitive sequence.
///
/// CPU sequences support slice access. Opaque Buffer sequences support cloning,
/// equality, length queries, and metadata formatting. Slice access, iteration,
/// ordering, and hashing panic for opaque storage.
#[repr(C)]
pub struct PrimitiveSequence<T: PrimitiveSequenceAlloc> {
    data: *mut T,
    size: usize,
    capacity: usize,
    is_rosidl_buffer: bool,
    owns_rosidl_buffer: bool,
}

/// A bounded primitive sequence.
#[derive(Clone)]
#[repr(transparent)]
pub struct BoundedPrimitiveSequence<T: PrimitiveSequenceAlloc, const N: usize> {
    inner: PrimitiveSequence<T>,
}

/// A by-value iterator over a normal, C-allocated primitive sequence.
pub struct PrimitiveSequenceIterator<T: PrimitiveSequenceAlloc> {
    seq: PrimitiveSequence<T>,
    idx: usize,
}

impl<T: PrimitiveSequenceAlloc> PrimitiveSequence<T> {
    /// Creates a sequence of `len` zero-initialized elements.
    pub fn new(len: usize) -> Self {
        Self::try_new(len).expect("PrimitiveSequence initialization failed")
    }

    /// Allocates zero-initialized elements, reporting native allocation failure.
    pub fn try_new(len: usize) -> Result<Self, crate::BufferError> {
        let mut seq = Self::default();
        if !T::primitive_sequence_init(&mut seq, len) {
            return Err(crate::BufferError::Allocation(
                "primitive sequence allocation failed".into(),
            ));
        }
        Ok(seq)
    }

    /// Returns whether this sequence contains an opaque `rosidl::Buffer<T>`.
    pub fn is_rosidl_buffer(&self) -> bool {
        self.is_rosidl_buffer
    }

    /// Returns the number of elements represented by the sequence.
    pub fn len(&self) -> usize {
        self.size
    }

    /// Returns whether the sequence contains no elements.
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// Extracts a slice from a normal primitive sequence.
    ///
    /// Panics when the C sequence contains an opaque `rosidl::Buffer<T>`.
    pub fn as_slice(&self) -> &[T] {
        self.assert_contiguous();
        if self.data.is_null() {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(self.data, self.size) }
        }
    }

    /// Extracts a mutable slice from a normal primitive sequence.
    ///
    /// Panics when the C sequence contains an opaque `rosidl::Buffer<T>`.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        self.assert_contiguous();
        if self.data.is_null() {
            &mut []
        } else {
            unsafe { std::slice::from_raw_parts_mut(self.data, self.size) }
        }
    }

    fn assert_contiguous(&self) {
        assert!(
            !self.is_rosidl_buffer,
            "an opaque rosidl buffer cannot be viewed as a primitive slice"
        );
    }
}

impl<T: PrimitiveSequenceAlloc> PrimitiveSequence<T> {
    /// Copies contents to a host vector, including opaque backend storage.
    pub fn try_to_vec(&self) -> Result<Vec<T>, crate::BufferError> {
        T::primitive_sequence_to_vec(self)
    }

    /// Materializes opaque storage; contiguous storage retains its allocation.
    pub fn try_into_cpu(self) -> Result<Self, crate::BufferError> {
        if self.is_rosidl_buffer {
            let values = self.try_to_vec()?;
            let mut sequence = Self::try_new(values.len())?;
            sequence.as_mut_slice().copy_from_slice(&values);
            Ok(sequence)
        } else {
            Ok(self)
        }
    }

    pub(crate) fn opaque_ptr(&self) -> Option<*mut std::ffi::c_void> {
        self.is_rosidl_buffer.then_some(self.data.cast())
    }
}

impl<T: PrimitiveSequenceAlloc> Clone for PrimitiveSequence<T> {
    fn clone(&self) -> Self {
        let mut seq = Self::default();
        if !T::primitive_sequence_copy(self, &mut seq) {
            panic!("Cloning PrimitiveSequence failed");
        }
        seq
    }
}

impl<T: Debug + PrimitiveSequenceAlloc> Debug for PrimitiveSequence<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_rosidl_buffer {
            f.debug_struct("PrimitiveSequence")
                .field("len", &self.size)
                .field("buffer", &self.data)
                .field("owned", &self.owns_rosidl_buffer)
                .finish()
        } else {
            self.as_slice().fmt(f)
        }
    }
}

impl<T: PrimitiveSequenceAlloc> Default for PrimitiveSequence<T> {
    fn default() -> Self {
        Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            is_rosidl_buffer: false,
            owns_rosidl_buffer: false,
        }
    }
}

impl<T: PrimitiveSequenceAlloc> Deref for PrimitiveSequence<T> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl<T: PrimitiveSequenceAlloc> DerefMut for PrimitiveSequence<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

impl<T: PrimitiveSequenceAlloc> Drop for PrimitiveSequence<T> {
    fn drop(&mut self) {
        T::primitive_sequence_fini(self);
    }
}

impl<T: PrimitiveSequenceAlloc> Extend<T> for PrimitiveSequence<T> {
    fn extend<I: IntoIterator<Item = T>>(&mut self, iter: I) {
        self.assert_contiguous();
        let old_len = self.size;
        let appended: Vec<T> = iter.into_iter().collect();
        if appended.is_empty() {
            return;
        }
        let len = old_len
            .checked_add(appended.len())
            .expect("sequence length overflow");
        let mut replacement = Self::new(len);
        let (prefix, suffix) = replacement.as_mut_slice().split_at_mut(old_len);
        prefix.copy_from_slice(self.as_slice());
        suffix.copy_from_slice(&appended);
        *self = replacement;
    }
}

impl<T: PrimitiveSequenceAlloc + Copy> From<&[T]> for PrimitiveSequence<T> {
    fn from(slice: &[T]) -> Self {
        let mut seq = Self::new(slice.len());
        seq.as_mut_slice().copy_from_slice(slice);
        seq
    }
}

impl<T: PrimitiveSequenceAlloc> From<Vec<T>> for PrimitiveSequence<T> {
    fn from(values: Vec<T>) -> Self {
        Self::from(values.as_slice())
    }
}

impl<T: PrimitiveSequenceAlloc + Copy> From<PrimitiveSequence<T>> for Vec<T> {
    fn from(seq: PrimitiveSequence<T>) -> Self {
        seq.as_slice().to_vec()
    }
}

impl<T: PrimitiveSequenceAlloc> FromIterator<T> for PrimitiveSequence<T> {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let values: Vec<T> = iter.into_iter().collect();
        Self::from(values.as_slice())
    }
}

impl<T: PrimitiveSequenceAlloc> IntoIterator for PrimitiveSequence<T> {
    type Item = T;
    type IntoIter = PrimitiveSequenceIterator<T>;

    fn into_iter(self) -> Self::IntoIter {
        self.assert_contiguous();
        PrimitiveSequenceIterator { seq: self, idx: 0 }
    }
}

impl<T: PrimitiveSequenceAlloc + PartialEq> PartialEq for PrimitiveSequence<T> {
    fn eq(&self, other: &Self) -> bool {
        T::primitive_sequence_are_equal(self, other)
    }
}

impl<T: PrimitiveSequenceAlloc + Eq> Eq for PrimitiveSequence<T> {}

impl<T: PrimitiveSequenceAlloc + PartialOrd> PartialOrd for PrimitiveSequence<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.as_slice().partial_cmp(other.as_slice())
    }
}

impl<T: PrimitiveSequenceAlloc + Ord> Ord for PrimitiveSequence<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.as_slice().cmp(other.as_slice())
    }
}

impl<T: PrimitiveSequenceAlloc + Hash> Hash for PrimitiveSequence<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.as_slice().hash(state);
    }
}

unsafe impl<T: PrimitiveSequenceAlloc + Send> Send for PrimitiveSequence<T> {}
unsafe impl<T: PrimitiveSequenceAlloc + Sync> Sync for PrimitiveSequence<T> {}

impl PrimitiveSequence<u8> {
    /// Takes ownership of an opaque `rosidl::Buffer<u8>`.
    ///
    /// # Safety
    ///
    /// `buffer` must be a non-null pointer returned by the ROS IDL Buffer API,
    /// `size` must be its byte length, and ownership must not be retained
    /// elsewhere.
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
            is_rosidl_buffer: true,
            owns_rosidl_buffer: true,
        })
    }

    /// Returns the opaque Buffer pointer when this sequence is Buffer-backed.
    pub fn rosidl_buffer_ptr(&self) -> Option<*mut std::ffi::c_void> {
        self.is_rosidl_buffer.then_some(self.data.cast())
    }

    /// Releases and returns the owned opaque Buffer pointer.
    ///
    /// Returns `Err(self)` for normal sequences or non-owning Buffer views.
    pub fn into_owned_rosidl_buffer(mut self) -> Result<*mut std::ffi::c_void, Self> {
        if !self.is_rosidl_buffer || !self.owns_rosidl_buffer {
            return Err(self);
        }
        let buffer = self.data.cast();
        self.data = std::ptr::null_mut();
        self.size = 0;
        self.capacity = 0;
        self.is_rosidl_buffer = false;
        self.owns_rosidl_buffer = false;
        Ok(buffer)
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> BoundedPrimitiveSequence<T, N> {
    /// Creates a bounded sequence of `len` zero-initialized elements.
    pub fn new(len: usize) -> Self {
        Self::try_new(len).unwrap()
    }

    /// Attempts to create a bounded sequence.
    pub fn try_new(len: usize) -> Result<Self, SequenceExceedsBoundsError> {
        if len > N {
            return Err(SequenceExceedsBoundsError {
                len,
                upper_bound: N,
            });
        }
        Ok(Self {
            inner: PrimitiveSequence::new(len),
        })
    }

    /// Returns whether this sequence contains an opaque `rosidl::Buffer<T>`.
    pub fn is_rosidl_buffer(&self) -> bool {
        self.inner.is_rosidl_buffer()
    }

    /// Returns the number of elements, including for opaque storage.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Returns whether the sequence has no elements.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Extracts a slice from a normal bounded primitive sequence.
    pub fn as_slice(&self) -> &[T] {
        self.inner.as_slice()
    }

    /// Extracts a mutable slice from a normal bounded primitive sequence.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        self.inner.as_mut_slice()
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> Default for BoundedPrimitiveSequence<T, N> {
    fn default() -> Self {
        Self {
            inner: PrimitiveSequence::default(),
        }
    }
}

impl<T: PrimitiveSequenceAlloc + Debug, const N: usize> Debug for BoundedPrimitiveSequence<T, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.inner.fmt(f)
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> Deref for BoundedPrimitiveSequence<T, N> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        self.inner.deref()
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> DerefMut for BoundedPrimitiveSequence<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner.deref_mut()
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> TryFrom<Vec<T>> for BoundedPrimitiveSequence<T, N> {
    type Error = SequenceExceedsBoundsError;

    fn try_from(values: Vec<T>) -> Result<Self, Self::Error> {
        if values.len() > N {
            return Err(SequenceExceedsBoundsError {
                len: values.len(),
                upper_bound: N,
            });
        }
        Ok(Self {
            inner: values.into(),
        })
    }
}

impl<T: PrimitiveSequenceAlloc + Copy, const N: usize> TryFrom<&[T]>
    for BoundedPrimitiveSequence<T, N>
{
    type Error = SequenceExceedsBoundsError;

    fn try_from(values: &[T]) -> Result<Self, Self::Error> {
        Self::try_from(values.to_vec())
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> FromIterator<T> for BoundedPrimitiveSequence<T, N> {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let values: Vec<T> = iter.into_iter().take(N).collect();
        Self {
            inner: values.into(),
        }
    }
}

impl<T: PrimitiveSequenceAlloc, const N: usize> IntoIterator for BoundedPrimitiveSequence<T, N> {
    type Item = T;
    type IntoIter = PrimitiveSequenceIterator<T>;

    fn into_iter(self) -> Self::IntoIter {
        self.inner.into_iter()
    }
}

impl<T: PrimitiveSequenceAlloc + PartialEq, const N: usize> PartialEq
    for BoundedPrimitiveSequence<T, N>
{
    fn eq(&self, other: &Self) -> bool {
        self.inner == other.inner
    }
}

impl<T: PrimitiveSequenceAlloc + Eq, const N: usize> Eq for BoundedPrimitiveSequence<T, N> {}

impl<T: PrimitiveSequenceAlloc + PartialOrd, const N: usize> PartialOrd
    for BoundedPrimitiveSequence<T, N>
{
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.inner.partial_cmp(&other.inner)
    }
}

impl<T: PrimitiveSequenceAlloc + Ord, const N: usize> Ord for BoundedPrimitiveSequence<T, N> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.inner.cmp(&other.inner)
    }
}

impl<T: PrimitiveSequenceAlloc + Hash, const N: usize> Hash for BoundedPrimitiveSequence<T, N> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.inner.hash(state);
    }
}

impl<T: PrimitiveSequenceAlloc> Iterator for PrimitiveSequenceIterator<T> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.idx >= self.seq.size {
            return None;
        }
        let elem = unsafe { self.seq.data.add(self.idx).read() };
        self.idx += 1;
        Some(elem)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let len = self.seq.size - self.idx;
        (len, Some(len))
    }
}

impl<T: PrimitiveSequenceAlloc> ExactSizeIterator for PrimitiveSequenceIterator<T> {}
impl<T: PrimitiveSequenceAlloc> FusedIterator for PrimitiveSequenceIterator<T> {}

impl Display for SequenceExceedsBoundsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        write!(
            f,
            "BoundedSequence with upper bound {} initialized with len {}",
            self.upper_bound, self.len
        )
    }
}

impl std::error::Error for SequenceExceedsBoundsError {}

macro_rules! impl_primitive_sequence_alloc {
    ($rust_type:ty, $init_func:ident, $fini_func:ident, $copy_func:ident, $equal_func:ident $(, $extra:item)*) => {
        #[link(name = "rosidl_runtime_c")]
        unsafe extern "C" {
            fn $init_func(seq: *mut PrimitiveSequence<$rust_type>, size: usize) -> bool;
            fn $fini_func(seq: *mut PrimitiveSequence<$rust_type>);
            fn $copy_func(
                in_seq: *const PrimitiveSequence<$rust_type>,
                out_seq: *mut PrimitiveSequence<$rust_type>,
            ) -> bool;
            fn $equal_func(
                lhs: *const PrimitiveSequence<$rust_type>,
                rhs: *const PrimitiveSequence<$rust_type>,
            ) -> bool;
        }

        impl PrimitiveSequenceAlloc for $rust_type {
            $($extra)*
            fn primitive_sequence_init(seq: &mut PrimitiveSequence<Self>, size: usize) -> bool {
                unsafe {
                    let ret = $init_func(seq as *mut _, size);
                    if ret && !seq.data.is_null() {
                        std::ptr::write_bytes(seq.data, 0u8, size);
                    }
                    ret
                }
            }
            fn primitive_sequence_fini(seq: &mut PrimitiveSequence<Self>) {
                unsafe { $fini_func(seq as *mut _) }
            }
            fn primitive_sequence_copy(
                in_seq: &PrimitiveSequence<Self>,
                out_seq: &mut PrimitiveSequence<Self>,
            ) -> bool {
                unsafe { $copy_func(in_seq as *const _, out_seq as *mut _) }
            }
            fn primitive_sequence_are_equal(
                lhs: &PrimitiveSequence<Self>,
                rhs: &PrimitiveSequence<Self>,
            ) -> bool {
                unsafe { $equal_func(lhs, rhs) }
            }
        }
    };
}

// Primitives are not messages themselves, but there can be sequences of them.
//
// See https://github.com/ros2/rosidl/blob/master/rosidl_runtime_c/include/rosidl_runtime_c/primitives_sequence.h
// Long double isn't available in Rust, so it is skipped.
impl_primitive_sequence_alloc!(
    f32,
    rosidl_runtime_c__float__Sequence__init,
    rosidl_runtime_c__float__Sequence__fini,
    rosidl_runtime_c__float__Sequence__copy,
    rosidl_runtime_c__float__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    f64,
    rosidl_runtime_c__double__Sequence__init,
    rosidl_runtime_c__double__Sequence__fini,
    rosidl_runtime_c__double__Sequence__copy,
    rosidl_runtime_c__double__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    bool,
    rosidl_runtime_c__boolean__Sequence__init,
    rosidl_runtime_c__boolean__Sequence__fini,
    rosidl_runtime_c__boolean__Sequence__copy,
    rosidl_runtime_c__boolean__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    u8,
    rosidl_runtime_c__uint8__Sequence__init,
    rosidl_runtime_c__uint8__Sequence__fini,
    rosidl_runtime_c__uint8__Sequence__copy,
    rosidl_runtime_c__uint8__Sequence__are_equal,
    fn primitive_sequence_to_vec(
        seq: &PrimitiveSequence<Self>,
    ) -> Result<Vec<Self>, crate::BufferError> {
        match seq.opaque_ptr() {
            Some(pointer) => crate::buffer::copy_to_host(pointer, seq.len()),
            None => Ok(seq.as_slice().to_vec()),
        }
    }
);
impl_primitive_sequence_alloc!(
    i8,
    rosidl_runtime_c__int8__Sequence__init,
    rosidl_runtime_c__int8__Sequence__fini,
    rosidl_runtime_c__int8__Sequence__copy,
    rosidl_runtime_c__int8__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    u16,
    rosidl_runtime_c__uint16__Sequence__init,
    rosidl_runtime_c__uint16__Sequence__fini,
    rosidl_runtime_c__uint16__Sequence__copy,
    rosidl_runtime_c__uint16__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    i16,
    rosidl_runtime_c__int16__Sequence__init,
    rosidl_runtime_c__int16__Sequence__fini,
    rosidl_runtime_c__int16__Sequence__copy,
    rosidl_runtime_c__int16__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    u32,
    rosidl_runtime_c__uint32__Sequence__init,
    rosidl_runtime_c__uint32__Sequence__fini,
    rosidl_runtime_c__uint32__Sequence__copy,
    rosidl_runtime_c__uint32__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    i32,
    rosidl_runtime_c__int32__Sequence__init,
    rosidl_runtime_c__int32__Sequence__fini,
    rosidl_runtime_c__int32__Sequence__copy,
    rosidl_runtime_c__int32__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    u64,
    rosidl_runtime_c__uint64__Sequence__init,
    rosidl_runtime_c__uint64__Sequence__fini,
    rosidl_runtime_c__uint64__Sequence__copy,
    rosidl_runtime_c__uint64__Sequence__are_equal
);
impl_primitive_sequence_alloc!(
    i64,
    rosidl_runtime_c__int64__Sequence__init,
    rosidl_runtime_c__int64__Sequence__fini,
    rosidl_runtime_c__int64__Sequence__copy,
    rosidl_runtime_c__int64__Sequence__are_equal
);

macro_rules! impl_cpu_sequence_alloc {
    ($($ty:ty),+ $(,)?) => {$(
        impl SequenceAlloc for $ty {
            fn sequence_init(seq: &mut Sequence<Self>, size: usize) -> bool {
                let mut native = PrimitiveSequence::<Self>::default();
                if !Self::primitive_sequence_init(&mut native, size) {
                    return false;
                }
                let replacement = Sequence {
                    data: native.data,
                    size: native.size,
                    capacity: native.capacity,
                };
                std::mem::forget(native);
                *seq = replacement;
                true
            }
            fn sequence_fini(seq: &mut Sequence<Self>) {
                let native = PrimitiveSequence {
                    data: seq.data,
                    size: seq.size,
                    capacity: seq.capacity,
                    is_rosidl_buffer: false,
                    owns_rosidl_buffer: false,
                };
                seq.data = std::ptr::null_mut();
                seq.size = 0;
                seq.capacity = 0;
                drop(native);
            }
            fn sequence_copy(input: &Sequence<Self>, output: &mut Sequence<Self>) -> bool {
                let mut replacement = Sequence::<Self>::default();
                if !Self::sequence_init(&mut replacement, input.len()) {
                    return false;
                }
                replacement.as_mut_slice().copy_from_slice(input.as_slice());
                *output = replacement;
                true
            }
        }
    )+};
}

impl_cpu_sequence_alloc!(bool, u8, i8, u16, i16, u32, i32, u64, i64, f32, f64);

impl<T: PrimitiveSequenceAlloc, const N: usize> BoundedPrimitiveSequence<T, N> {
    /// Materializes opaque storage without changing the bound.
    pub fn try_into_cpu(self) -> Result<Self, crate::BufferError> {
        Ok(Self {
            inner: self.inner.try_into_cpu()?,
        })
    }

    /// Returns the underlying sequence without copying its storage.
    pub fn into_unbounded(self) -> PrimitiveSequence<T> {
        self.inner
    }

    /// Checks the bound without copying sequence storage.
    pub fn try_from_unbounded(
        inner: PrimitiveSequence<T>,
    ) -> Result<Self, SequenceExceedsBoundsError> {
        if inner.len() > N {
            return Err(SequenceExceedsBoundsError {
                len: inner.len(),
                upper_bound: N,
            });
        }
        Ok(Self { inner })
    }
}

/// Creates a sequence, similar to the `vec!` macro.
///
/// It's possible to create message and primitive sequences.
/// Unbounded sequences are created by a comma-separated list of values.
/// Bounded sequences are created by additionally specifying the maximum capacity (the `N` type
/// parameter) in the beginning, followed by a `#`.
///
/// # Example
/// ```
/// # use rosidl_runtime_rs::{BoundedPrimitiveSequence, PrimitiveSequence, seq};
/// let unbounded: PrimitiveSequence<i32> = seq![1, 2, 3];
/// let bounded: BoundedPrimitiveSequence<i32, 5> = seq![5 # 1, 2, 3];
/// assert_eq!(&unbounded[..], &bounded[..])
/// ```
#[macro_export]
macro_rules! seq {
    [$( $elem:expr ),*] => {
        vec![$($elem),*].into()
    };
    [$len:literal # $( $elem:expr ),*] => {
        ::std::convert::TryInto::try_into(vec![$($elem),*]).unwrap()
    };
}

#[cfg(test)]
mod tests {
    use quickcheck::{quickcheck, Arbitrary, Gen};

    use super::*;

    impl<T: Arbitrary + PrimitiveSequenceAlloc> Arbitrary for PrimitiveSequence<T> {
        fn arbitrary(g: &mut Gen) -> Self {
            Vec::arbitrary(g).into()
        }
    }

    impl<T: Arbitrary + PrimitiveSequenceAlloc> Arbitrary for BoundedPrimitiveSequence<T, 256> {
        fn arbitrary(g: &mut Gen) -> Self {
            let len = u8::arbitrary(g);
            (0..len).map(|_| T::arbitrary(g)).collect()
        }
    }

    #[test]
    fn test_empty_sequence() {
        assert!(PrimitiveSequence::<i32>::default().is_empty());
        assert!(BoundedPrimitiveSequence::<i32, 5>::default().is_empty());
    }

    #[test]
    fn test_sequence_layouts() {
        let primitive_fields_size =
            std::mem::size_of::<*mut u8>() + 2 * std::mem::size_of::<usize>() + 2;
        let alignment = std::mem::align_of::<PrimitiveSequence<u8>>();
        let expected_primitive_size = primitive_fields_size.div_ceil(alignment) * alignment;
        assert_eq!(
            std::mem::size_of::<PrimitiveSequence<u8>>(),
            expected_primitive_size
        );
        assert_eq!(
            std::mem::size_of::<BoundedPrimitiveSequence<u8, 4>>(),
            expected_primitive_size
        );
        assert_eq!(
            std::mem::size_of::<Sequence<crate::String>>(),
            std::mem::size_of::<*mut crate::String>() + 2 * std::mem::size_of::<usize>()
        );
    }

    #[test]
    fn test_buffer_sequence_rejects_slice_access() {
        let seq = std::mem::ManuallyDrop::new(PrimitiveSequence::<u8> {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            is_rosidl_buffer: true,
            owns_rosidl_buffer: false,
        });
        assert!(std::panic::catch_unwind(|| seq.as_slice()).is_err());
    }

    quickcheck! {
        fn test_extend(xs: Vec<i32>, ys: Vec<i32>) -> bool {
            let mut xs_seq = PrimitiveSequence::new(xs.len());
            xs_seq.copy_from_slice(&xs);
            xs_seq.extend(ys.clone());
            if xs_seq.len() != xs.len() + ys.len() {
                return false;
            }
            if xs_seq[..xs.len()] != xs[..] {
                return false;
            }
            if xs_seq[xs.len()..] != ys[..] {
                return false;
            }
            true
        }
    }

    quickcheck! {
        fn test_iteration(xs: Vec<i32>) -> bool {
            let mut seq_1 = PrimitiveSequence::new(xs.len());
            seq_1.copy_from_slice(&xs);
            let seq_2 = seq_1.clone().into_iter().collect();
            seq_1 == seq_2
        }
    }

    #[test]
    fn test_into_vec_primitive_roundtrip() {
        let xs: Vec<i32> = (0..1024).collect();
        let seq: PrimitiveSequence<i32> = PrimitiveSequence::from(&xs[..]);
        let ys: Vec<i32> = seq.into();
        assert_eq!(xs, ys);
    }

    quickcheck! {
        fn test_into_vec_primitive_quickcheck(xs: Vec<u8>) -> bool {
            let seq: PrimitiveSequence<u8> = PrimitiveSequence::from(&xs[..]);
            let ys: Vec<u8> = seq.into();
            xs == ys
        }
    }

    #[test]
    fn empty_primitive_sequences_collect_clone_and_extend() {
        let mut sequence: PrimitiveSequence<u32> = std::iter::empty().collect();
        assert!(sequence.clone().is_empty());
        sequence.extend([7, 11]);
        sequence.extend([]);
        assert_eq!(sequence.as_slice(), &[7, 11]);
        assert!(PrimitiveSequence::<u32>::from(Vec::new()).is_empty());
    }

    #[test]
    fn failed_initialization_preserves_existing_elements() {
        let mut sequence = PrimitiveSequence::from(&[17u64, 23][..]);
        assert!(!u64::primitive_sequence_init(&mut sequence, usize::MAX));
        assert_eq!(sequence.as_slice(), &[17, 23]);
    }

    #[test]
    fn primitive_iterator_tracks_remaining_elements() {
        let mut iter = PrimitiveSequence::from(&[3i32, 5][..]).into_iter();
        assert_eq!(iter.len(), 2);
        assert_eq!(iter.next(), Some(3));
        assert_eq!(iter.size_hint(), (1, Some(1)));
        assert_eq!(iter.next(), Some(5));
        assert_eq!(iter.len(), 0);
        assert_eq!(iter.next(), None);
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn bounded_primitive_sequences_enforce_limits() {
        type Bounded = BoundedPrimitiveSequence<u32, 2>;
        assert!(Bounded::try_new(3).is_err());
        assert!(Bounded::try_from(vec![1, 2, 3]).is_err());
        assert!(Bounded::try_from(&[1, 2, 3][..]).is_err());
        let mut sequence = Bounded::try_from(&[1, 2][..]).unwrap();
        sequence.as_mut_slice()[1] = 9;
        assert_eq!(sequence.len(), 2);
        assert!(!sequence.is_empty());
        assert_eq!(sequence.clone().into_iter().collect::<Vec<_>>(), vec![1, 9]);
        assert_eq!((0..4).collect::<Bounded>().as_slice(), &[0, 1]);
    }

    #[test]
    fn primitive_types_initialize_copy_and_compare() {
        macro_rules! check {
            ($($ty:ty),+ $(,)?) => {$(
                let sequence = PrimitiveSequence::<$ty>::new(3);
                assert_eq!(sequence.as_slice(), &[<$ty>::default(); 3]);
                assert_eq!(sequence, sequence.clone());
                assert_ne!(sequence, PrimitiveSequence::<$ty>::new(2));
            )+};
        }
        check!(bool, u8, i8, u16, i16, u32, i32, u64, i64, f32, f64);
        let nan = PrimitiveSequence::from(&[f32::NAN][..]);
        assert_ne!(nan, nan.clone());
    }

    #[test]
    fn opaque_sequence_metadata_and_rejections() {
        let mut sequence = PrimitiveSequence::<u8> {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
            is_rosidl_buffer: true,
            owns_rosidl_buffer: false,
        };
        assert!(format!("{sequence:?}").contains("owned: false"));
        assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            sequence.as_mut_slice();
        }))
        .is_err());
        let bounded = BoundedPrimitiveSequence::<u8, 4> { inner: sequence };
        assert_eq!(bounded.len(), 0);
        assert!(bounded.is_empty());
        assert!(bounded.is_rosidl_buffer());
        assert!(bounded.inner.into_owned_rosidl_buffer().is_err());
        assert!(PrimitiveSequence::<u8>::from(&[1, 2][..])
            .into_owned_rosidl_buffer()
            .is_err());
        assert!(
            unsafe { PrimitiveSequence::from_owned_rosidl_buffer(std::ptr::null_mut(), 0) }
                .is_none()
        );
    }

    #[test]
    fn message_iterator_reports_exact_length_and_drops_remaining_values() {
        let sequence = Sequence::from(vec![crate::String::from("one"), crate::String::from("two")]);
        let mut iter = sequence.into_iter();
        assert_eq!(iter.len(), 2);
        let first = iter.next().unwrap();
        assert_eq!(iter.size_hint(), (1, Some(1)));
        drop(iter);
        assert_eq!(first.to_string(), "one");
        let mut empty = Sequence::<crate::String>::default().into_iter();
        assert_eq!(empty.len(), 0);
        assert_eq!(empty.next(), None);
        assert_eq!(empty.size_hint(), (0, Some(0)));
    }
}
