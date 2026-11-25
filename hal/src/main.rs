//! `android.hardware.sensors` V3 for the Onyx Boox Go 7: the LSM6DST accelerometer, through the Snapdragon Sensor Core

mod device;
mod every;
mod queue;
mod service;

use std::process::ExitCode;
use std::sync::mpsc;
use std::thread;

use android_hardware_sensors::aidl::android::hardware::sensors::ISensors::BnSensors;
use binder::{BinderFeatures, ProcessState};
use log::{LevelFilter, error, info};
use ssc::worker::{Timing, Worker};

const SERVICE: &str = "android.hardware.sensors.ISensors/default";

fn main() -> ExitCode {
    let _ = logger::init(
        logger::Config::default()
            .with_tag_on_device("sensors-onyx")
            .with_max_level(LevelFilter::Info),
    );
    let (commands, received) = mpsc::channel();
    let worker = thread::Builder::new()
        .name("ssc".into())
        .spawn(move || Worker::new(device::Device::new(), Timing::HAL).run(&received));
    if let Err(e) = worker {
        error!("starting the sensor core worker: {e}");
        return ExitCode::FAILURE;
    }
    ProcessState::set_thread_pool_max_thread_count(0);
    let sensors = BnSensors::new_binder(service::Sensors::new(commands), BinderFeatures::default());
    if let Err(e) = binder::add_service(SERVICE, sensors.as_binder()) {
        error!("registering {SERVICE}: {e:?}");
        return ExitCode::FAILURE;
    }
    info!("registered {SERVICE}");
    ProcessState::join_thread_pool();
    ExitCode::FAILURE
}
