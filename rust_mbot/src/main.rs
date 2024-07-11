#![no_std]
#![no_main]

use panic_halt as _;
use mpu6050::*;



#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.d20.into_pull_up_input(),
        pins.d21.into_pull_up_input(),
        50000,
    );

    let mut mpu = Mpu6050::new_with_addr(i2c, 0x69);

    let mut delay = arduino_hal::Delay::new();
    mpu.init(&mut delay).unwrap_or_else(|error| { if matches ! (error, Mpu6050Error::InvalidChipId(_)) { ufmt::uwriteln!(&mut serial, "Invalid chip id");} else { ufmt::uwriteln!(&mut serial, "Invalid i2c");}});

    loop {
        // get roll and pitch estimate
        //let ang = mpu.get_acc_angles().unwrap();
        //let ang_write = ang[0] as u32;
        //ufmt::uwriteln!(&mut serial, "r/p: {}", ang_write).unwrap();

        ufmt::uwriteln!(&mut serial, "Reading angles:").unwrap();

        // get sensor temp
        let rot = mpu.get_acc().unwrap();
        // Convert to unsigned int:
        let temp_write = (rot[1] * 100.0f32) as u32;

        ufmt::uwriteln!(&mut serial, "temp: {}c", temp_write).unwrap();

        // get gyro data, scaled with sensitivity
        //let gyro = mpu.get_gyro().unwrap();
        //let gyro_write = uFmt_f32::Three(gyro[0]);
        //ufmt::uwriteln!(&mut serial, "gyro: {}", gyro_write).unwrap();

        // get accelerometer data, scaled with sensitivity
        //let acc = mpu.get_acc().unwrap();
        //let acc_write = uFmt_f32::Three(acc[0]);
        //ufmt::uwriteln!(&mut serial, "acc: {}", acc_write).unwrap();

        arduino_hal::delay_ms(100);
    }
}
