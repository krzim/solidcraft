//! Work split over threads: natively one thread a share of the cores, in order; in the browser,
//! which has no threads, one after the other on the caller's thread. The results are the same
//! either way, in the order of the work.

use std::sync::{Arc, Mutex};

use crate::{CamError, Result};

/// How many threads to split `work` items over (each worth at least `least` of them): one per
/// core up to 16; one in the browser.
pub(crate) fn threads_for(work: usize, least: usize) -> usize {
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (work, least);
        1
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        /// Most threads one call uses.
        const MAX_THREADS: usize = 16;
        let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
        cores.min(MAX_THREADS).min(work / least.max(1)).max(1)
    }
}

/// `f` run for every item, each on a thread of its own when there is more than one (else here),
/// the results in order. A thread that cannot be had runs its item here; one that stops is an
/// error, never a panic.
pub(crate) fn map<T: Send, R: Send>(items: Vec<T>, f: impl Fn(T) -> Result<R> + Sync) -> Result<Vec<R>> {
    if items.len() <= 1 || cfg!(target_arch = "wasm32") {
        return items.into_iter().map(&f).collect();
    }
    let f = &f;
    let lost = || CamError::Unsupported("a worker thread lost its work".into());
    std::thread::scope(|scope| {
        let mut pending = Vec::with_capacity(items.len());
        for (b, item) in items.into_iter().enumerate() {
            // The item goes through a slot the thread takes it from, so that it is still here
            // when no thread can be had.
            let slot = Arc::new(Mutex::new(Some(item)));
            let mine = Arc::clone(&slot);
            let job = move || match mine.lock().ok().and_then(|mut s| s.take()) {
                Some(item) => f(item),
                None => Err(lost()),
            };
            match std::thread::Builder::new().name(format!("cam-work-{b}")).spawn_scoped(scope, job) {
                Ok(h) => pending.push(Err(h)),
                Err(_) => pending.push(Ok(match slot.lock().ok().and_then(|mut s| s.take()) {
                    Some(item) => f(item),
                    None => Err(lost()),
                })),
            }
        }
        pending
            .into_iter()
            .map(|p| match p {
                Ok(done) => done,
                Err(h) => h.join().unwrap_or_else(|_| Err(CamError::Unsupported("a worker thread stopped".into()))),
            })
            .collect()
    })
}

/// The ranges `0..len` split into `parts` runs of about equal `weight` (each item's), in order.
pub(crate) fn split(weights: &[usize], parts: usize) -> Vec<std::ops::Range<usize>> {
    let total: usize = weights.iter().sum();
    let parts = parts.max(1);
    let share = total.div_ceil(parts).max(1);
    let mut out = Vec::with_capacity(parts);
    let (mut start, mut acc) = (0, 0usize);
    for (i, w) in weights.iter().enumerate() {
        acc = acc.saturating_add(*w);
        if acc >= share && out.len() + 1 < parts {
            out.push(start..i + 1);
            (start, acc) = (i + 1, 0);
        }
    }
    if start < weights.len() || out.is_empty() {
        out.push(start..weights.len());
    }
    out
}
