//! Library split: the native bin (main.rs) and the wasm cdylib (wasm.rs) share
//! the solver/sim/scene/render core. Native gets rayon + serve + pack; wasm
//! builds with --no-default-features (sequential fallback in `par`).

pub mod render;
pub mod scene;
pub mod sim;
pub mod solve;

pub mod api;
#[cfg(feature = "native")]
pub mod pack;
#[cfg(feature = "native")]
pub mod serve;
#[cfg(target_arch = "wasm32")]
pub mod wasm;

/// rayon when the `rayon` feature is on; sequential stand-ins with the same
/// method names otherwise (wasm on GitHub Pages: no COOP/COEP, no threads).
#[cfg(feature = "rayon")]
pub mod par {
    pub use rayon::prelude::*;
}
#[cfg(not(feature = "rayon"))]
pub mod par {
    pub trait SliceExt<T> {
        fn par_iter(&self) -> std::slice::Iter<'_, T>;
    }
    impl<T> SliceExt<T> for [T] {
        fn par_iter(&self) -> std::slice::Iter<'_, T> {
            self.iter()
        }
    }
    pub trait SliceMutExt<T> {
        fn par_chunks_mut(&mut self, n: usize) -> std::slice::ChunksMut<'_, T>;
    }
    impl<T> SliceMutExt<T> for [T] {
        fn par_chunks_mut(&mut self, n: usize) -> std::slice::ChunksMut<'_, T> {
            self.chunks_mut(n)
        }
    }
    pub trait IterExt: Iterator + Sized {
        fn flat_map_iter<U: IntoIterator, F: FnMut(Self::Item) -> U>(
            self,
            f: F,
        ) -> std::iter::FlatMap<Self, U, F> {
            self.flat_map(f)
        }
    }
    impl<I: Iterator> IterExt for I {}
}
