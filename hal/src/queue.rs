//! The event queue sensorservice reads: samples and flush-completes become `Event`s, then its reader is woken

use std::io;
use std::ptr;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use android_hardware_sensors::aidl::android::hardware::sensors::{
    Event::{
        Event,
        EventPayload::{
            EventPayload,
            MetaData::{MetaData, MetaDataEventType::MetaDataEventType},
            Vec3::Vec3,
        },
    },
    ISensors::EVENT_QUEUE_FLAG_BITS_READ_AND_PROCESS,
    SensorStatus::SensorStatus,
    SensorType::SensorType,
};
use fmq::{MQDescriptor, MessageQueue, SynchronizedReadWrite};
use log::warn;
use ssc::accel;
use ssc::worker::{Sample, Sink};

use crate::every::Every;

const WARN_EVERY: Duration = Duration::from_secs(10);

/// sensorservice's event queue, from `ISensors.initialize`
pub struct EventQueue {
    queue: MessageQueue<Event>,
    flushes: usize,
    dropped: u64,
    warn: Every,
}

impl EventQueue {
    /// The queue behind `desc`; `None` without an event flag word to wake the reader with
    pub fn new(desc: &MQDescriptor<Event, SynchronizedReadWrite>) -> Option<Self> {
        let queue = MessageQueue::from_desc(desc, true);
        let _ = queue.event_flag_word()?;
        Some(Self {
            queue,
            flushes: 0,
            dropped: 0,
            warn: Every::new(WARN_EVERY),
        })
    }

    fn write(&mut self, samples: Vec<Event>) {
        let count = self.flushes.saturating_add(samples.len());
        let Some(mut txn) = self.queue.write_many(count) else {
            self.drop_samples(samples.len());
            return;
        };
        // Not `txn.write`: android-17 aidl's generated `WriteTo` puts union payloads at a wrong offset
        for (slot, event) in (0..self.flushes).map(|_| flush_complete()).chain(samples).enumerate() {
            // SAFETY: `slot` < `count`, so `ptr` is this transaction's aligned, writable slot for it, and `Event` is
            // `repr(C)`, so the copy has the layout sensorservice reads
            unsafe { ptr::write_volatile(txn.ptr(slot), event) };
        }
        // SAFETY: the loop wrote all `count` slots
        unsafe { txn.assume_written(count) };
        drop(txn);
        self.flushes = 0;
        self.wake();
    }

    fn drop_samples(&mut self, count: usize) {
        self.dropped = self.dropped.saturating_add(u64::try_from(count).unwrap_or(u64::MAX));
        if self.warn.due() {
            warn!(
                "event queue full: {} samples dropped so far, {} flushes waiting",
                self.dropped, self.flushes
            );
        }
    }

    fn wake(&self) {
        if let Some(word) = self.queue.event_flag_word() {
            wake(word, EVENT_QUEUE_FLAG_BITS_READ_AND_PROCESS.cast_unsigned());
        }
    }
}

fn wake(word: &AtomicU32, bits: u32) {
    let old = word.fetch_or(bits, Ordering::SeqCst);
    if !old & bits == 0 {
        return;
    }
    // SAFETY: `word` is the queue's event flag word, valid while the queue lives; FUTEX_WAKE_BITSET only uses the
    // address to find its waiters and reads nothing else, so the null timeout and second address are unused
    let woken = unsafe {
        libc::syscall(
            libc::SYS_futex,
            word.as_ptr(),
            libc::FUTEX_WAKE_BITSET,
            i32::MAX,
            ptr::null::<libc::timespec>(),
            ptr::null::<u32>(),
            bits,
        )
    };
    if woken == -1 {
        warn!("waking the event queue reader: {}", io::Error::last_os_error());
    }
}

const fn event(sample: &Sample) -> Event {
    Event {
        timestamp: sample.timestamp_ns,
        sensorHandle: accel::HANDLE,
        sensorType: SensorType::ACCELEROMETER,
        payload: EventPayload::Vec3(Vec3 {
            x: sample.x,
            y: sample.y,
            z: sample.z,
            status: SensorStatus(sample.status),
        }),
    }
}

const fn flush_complete() -> Event {
    Event {
        timestamp: 0,
        sensorHandle: accel::HANDLE,
        sensorType: SensorType::META_DATA,
        payload: EventPayload::Meta(MetaData {
            what: MetaDataEventType::META_DATA_FLUSH_COMPLETE,
        }),
    }
}

impl Sink for EventQueue {
    fn samples(&mut self, samples: &[Sample]) {
        self.write(samples.iter().map(event).collect());
    }

    fn flush_complete(&mut self) {
        self.flushes = self.flushes.saturating_add(1);
        self.write(Vec::new());
    }
}
