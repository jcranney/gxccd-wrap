//! This script will read all possible parameters from the camera and print
//! them to stdout. Useful for knowing the ranges/possible values of your 
//! camera.
use gxccd::*;
use enum_iterator::all;

fn main() -> Result<(), String> {
    let camera = Camera::new()?;

    let param = camera.get_string_parameter(StringParams::CameraDescription)?;
    println!("Camera description: {}", param);

    let major = camera.get_integer_parameter(IntegerParams::FirmwareMajor)?;
    let minor = camera.get_integer_parameter(IntegerParams::FirmwareMinor)?;
    let build = camera.get_integer_parameter(IntegerParams::FirmwareBuild)?;
    println!("Camera FW version: {}.{}.{}", major, minor, build);

    let temp = camera.get_value(Values::ChipTemperature)?;
    println!("Camera chip temp: {:0.2} °C", temp);

    let voltage = camera.get_value(Values::SupplyVoltage)?;
    println!("Camera supply voltage: {:0.2} V", voltage);
    
    let mut i = 0;
    while let Ok(result) = camera.enumerate_read_modes(i) {
        println!("Read mode #{}: {}", i, result);
        i += 1;
    }
    
    let mut i = 0;
    while let Ok(result) = camera.enumerate_filters(i) {
        println!("Filter #{}: {:?}", i, result);
        i += 1;
    }

    println!("\n------ Boolean Parameters ------");
    for param in all::<BooleanParams>().collect::<Vec<_>>() {
        print!("{:?}: ", param);
        match camera.get_boolean_parameter(param) {
            Ok(result) => println!("{}", result),
            Err(err) => println!("{}", err),
        }
    }
    
    println!("\n------ Integer Parameters ------");
    for param in all::<IntegerParams>().collect::<Vec<_>>() {
        print!("{:?}: ", param);
        match camera.get_integer_parameter(param) {
            Ok(result) => println!("{}", result),
            Err(err) => println!("{}", err),
        }
    }

    println!("\n------ String Parameters ------");
    for param in all::<StringParams>().collect::<Vec<_>>() {
        print!("{:?}: ", param);
        match camera.get_string_parameter(param) {
            Ok(result) => println!("{}", result),
            Err(err) => println!("{}", err),
        }
    }


    println!("\n------ Values ------");
    for value in all::<Values>().collect::<Vec<_>>() {
        print!("{:?}: ", value);
        match camera.get_value(value) {
            Ok(result) => println!("{}", result),
            Err(err) => println!("{}", err),
        }
    }

    Ok(())
}
