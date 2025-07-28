use gxccd::*;

fn main() -> Result<(), String> {
    let camera = Camera::new()?;

    let param = camera.get_string_parameter(StringParams::CameraDescription)?;
    eprintln!("Camera description: {}", param);

    let major = camera.get_integer_parameter(IntegerParams::FirmwareMajor)?;
    let minor = camera.get_integer_parameter(IntegerParams::FirmwareMinor)?;
    let build = camera.get_integer_parameter(IntegerParams::FirmwareBuild)?;
    eprintln!("Camera FW version: {}.{}.{}", major, minor, build);

    let temp = camera.get_value(Values::ChipTemperature)?;
    eprintln!("Camera chip temp: {:0.2} °C", temp);

    let voltage = camera.get_value(Values::SupplyVoltage)?;
    eprintln!("Camera supply voltage: {:0.2} V", voltage);
    
    let mut i = 0;
    while let Ok(result) = camera.enumerate_read_modes(i) {
        eprintln!("Read mode #{}: {}", i, result);
        i += 1;
    }
    Ok(())
}
