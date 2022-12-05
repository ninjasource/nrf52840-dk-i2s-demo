/*

Copyright notice:
The code in this module has been copied out of this repo
https://github.com/ferrous-systems/embedded-trainings-2020
License: MIT

It may have been modified a little to suit the needs of this project

*/

use core::{
    sync::atomic::{AtomicU32, Ordering},
    time::Duration,
};

use nrf52840_hal::pac::RTC0;

// Counter of OVERFLOW events -- an OVERFLOW occurs every (1<<24) ticks
static OVERFLOWS: AtomicU32 = AtomicU32::new(0);

/// Returns the time elapsed since the call to the `dk::init` function
///
/// The clock that is read to compute this value has a resolution of 30 microseconds.
///
/// Calling this function before calling `dk::init` will return a value of `0` nanoseconds.
pub fn uptime() -> Duration {
    // here we are going to perform a 64-bit read of the number of ticks elapsed
    //
    // a 64-bit load operation cannot performed in a single instruction so the operation can be
    // preempted by the RTC0 interrupt handler (which increases the OVERFLOWS counter)
    //
    // the loop below will load both the lower and upper parts of the 64-bit value while preventing
    // the issue of mixing a low value with an "old" high value -- note that, due to interrupts, an
    // arbitrary amount of time may elapse between the `hi1` load and the `low` load
    let overflows = &OVERFLOWS as *const AtomicU32 as *const u32;
    let ticks = loop {
        unsafe {
            // NOTE volatile is used to order these load operations among themselves
            let hi1 = overflows.read_volatile();
            let low = core::mem::transmute::<_, RTC0>(())
                .counter
                .read()
                .counter()
                .bits();
            let hi2 = overflows.read_volatile();

            if hi1 == hi2 {
                break u64::from(low) | (u64::from(hi1) << 24);
            }
        }
    };

    // 2**15 ticks = 1 second
    let freq = 1 << 15;
    let secs = ticks / freq;
    // subsec ticks
    let ticks = (ticks % freq) as u32;
    // one tick is equal to `1e9 / 32768` nanos
    // the fraction can be reduced to `1953125 / 64`
    // which can be further decomposed as `78125 * (5 / 4) * (5 / 4) * (1 / 4)`.
    // Doing the operation this way we can stick to 32-bit arithmetic without overflowing the value
    // at any stage
    let nanos =
        (((ticks % 32768).wrapping_mul(78125) >> 2).wrapping_mul(5) >> 2).wrapping_mul(5) >> 2;
    Duration::new(secs, nanos as u32)
}

pub fn process_tick() {
    let curr = OVERFLOWS.load(Ordering::Relaxed);
    OVERFLOWS.store(curr + 1, Ordering::Relaxed);

    // clear the EVENT register
    unsafe { core::mem::transmute::<_, RTC0>(()).events_ovrflw.reset() }
}
