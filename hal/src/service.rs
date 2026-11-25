//! `ISensors`: the one-entry sensor list, the framework's requests passed on to the worker, everything else refused

use std::sync::mpsc::Sender;
use std::sync::{Mutex, PoisonError};

use android_hardware_sensors::aidl::android::hardware::sensors::{
    Event::Event,
    ISensors::{ISensors, OperationMode::OperationMode, RateLevel::RateLevel, SharedMemInfo::SharedMemInfo},
    ISensorsCallback::ISensorsCallback,
    SensorInfo::{SENSOR_FLAG_BITS_CONTINUOUS_MODE, SensorInfo},
    SensorType::SensorType,
};
use binder::{ExceptionCode, Interface, Status, Strong};
use fmq::{MQDescriptor, SynchronizedReadWrite};
use log::info;
use ssc::accel;
use ssc::worker::{Command, Request};

use crate::queue::EventQueue;

/// The `ISensors` service
pub struct Sensors {
    state: Mutex<State>,
}

struct State {
    request: Request,
    commands: Sender<Command<EventQueue>>,
}

impl State {
    fn send(&self, command: Command<EventQueue>) -> binder::Result<()> {
        self.commands
            .send(command)
            .map_err(|_| Status::new_exception(ExceptionCode::ILLEGAL_STATE, None))
    }

    fn update(&mut self, request: Request) -> binder::Result<()> {
        self.request = request;
        self.send(Command::Request(request))
    }
}

impl Sensors {
    /// The service, passing requests on to the worker behind `commands`
    pub fn new(commands: Sender<Command<EventQueue>>) -> Self {
        Self {
            state: Mutex::new(State {
                request: idle(),
                commands,
            }),
        }
    }

    fn with<T>(&self, f: impl FnOnce(&mut State) -> binder::Result<T>) -> binder::Result<T> {
        f(&mut self.state.lock().unwrap_or_else(PoisonError::into_inner))
    }
}

fn idle() -> Request {
    Request {
        active: false,
        period_ns: i64::from(accel::MAX_DELAY_US).saturating_mul(1000),
    }
}

fn illegal_argument() -> Status {
    Status::new_exception(ExceptionCode::ILLEGAL_ARGUMENT, None)
}

fn unsupported<T>() -> binder::Result<T> {
    Err(Status::new_exception(ExceptionCode::UNSUPPORTED_OPERATION, None))
}

fn check(handle: i32) -> binder::Result<()> {
    if handle == accel::HANDLE {
        Ok(())
    } else {
        Err(illegal_argument())
    }
}

fn accelerometer() -> SensorInfo {
    SensorInfo {
        sensorHandle: accel::HANDLE,
        name: accel::NAME.into(),
        vendor: accel::VENDOR.into(),
        version: accel::VERSION,
        r#type: SensorType::ACCELEROMETER,
        typeAsString: String::new(),
        maxRange: accel::MAX_RANGE,
        resolution: accel::RESOLUTION,
        power: accel::POWER_MA,
        minDelayUs: accel::MIN_DELAY_US,
        fifoReservedEventCount: 0,
        fifoMaxEventCount: 0,
        requiredPermission: String::new(),
        maxDelayUs: accel::MAX_DELAY_US,
        flags: SENSOR_FLAG_BITS_CONTINUOUS_MODE,
    }
}

impl Interface for Sensors {}

impl ISensors for Sensors {
    fn activate(&self, handle: i32, enabled: bool) -> binder::Result<()> {
        check(handle)?;
        self.with(|s| {
            s.update(Request {
                active: enabled,
                ..s.request
            })
        })
    }

    fn batch(&self, handle: i32, period_ns: i64, _max_report_latency_ns: i64) -> binder::Result<()> {
        check(handle)?;
        self.with(|s| s.update(Request { period_ns, ..s.request }))
    }

    fn configDirectReport(&self, _handle: i32, _channel: i32, _rate: RateLevel) -> binder::Result<i32> {
        unsupported()
    }

    fn flush(&self, handle: i32) -> binder::Result<()> {
        check(handle)?;
        self.with(|s| {
            if s.request.active {
                s.send(Command::Flush)
            } else {
                Err(illegal_argument())
            }
        })
    }

    fn getSensorsList(&self) -> binder::Result<Vec<SensorInfo>> {
        Ok(vec![accelerometer()])
    }

    fn initialize(
        &self,
        events: &MQDescriptor<Event, SynchronizedReadWrite>,
        _wake_lock: &MQDescriptor<i32, SynchronizedReadWrite>,
        _callback: &Strong<dyn ISensorsCallback>,
    ) -> binder::Result<()> {
        self.with(|s| {
            s.update(idle())?;
            let queue = EventQueue::new(events).ok_or_else(illegal_argument)?;
            info!("initialize: event queue ready");
            s.send(Command::Sink(queue))
        })
    }

    fn injectSensorData(&self, _event: &Event) -> binder::Result<()> {
        unsupported()
    }

    fn registerDirectChannel(&self, _mem: &SharedMemInfo) -> binder::Result<i32> {
        unsupported()
    }

    fn setOperationMode(&self, mode: OperationMode) -> binder::Result<()> {
        if mode == OperationMode::NORMAL {
            Ok(())
        } else {
            unsupported()
        }
    }

    fn unregisterDirectChannel(&self, _channel: i32) -> binder::Result<()> {
        unsupported()
    }
}
