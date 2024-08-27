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

    eprintln!("taking dark frames");
    camera.set_read_mode(3)?;
    let exp_time: f64 = 2.0;
    for i in 0..10 {
        eprintln!("taking frame {}", i);
        let primary_hdu = take_full_frame(&camera, &exp_time, false)?;
        fitrs::Fits::create(format!("./dark_{:03}.fits",i), primary_hdu).map_err(|e| e.to_string())?;
    }    
    eprintln!("taking light frames");
    camera.set_read_mode(3)?;
    let exp_time: f64 = 2.0;
    for i in 0..10 {
        eprintln!("taking frame {}", i);
        let primary_hdu = take_full_frame(&camera, &exp_time, true)?;
        fitrs::Fits::create(format!("./light_{:03}.fits",i), primary_hdu).map_err(|e| e.to_string())?;
    }    
    Ok(())
}

fn take_full_frame(camera: &Camera, exp_time: &f64, light: bool) -> Result<fitrs::Hdu, String> {
    // start exposure
    let chip_w = camera.get_integer_parameter(IntegerParams::ChipW)?;
    let chip_d = camera.get_integer_parameter(IntegerParams::ChipD)?;
    
    camera.start_exposure(*exp_time, light, 0, 0, chip_w, chip_d)?;
    // sleep during exposure
    std::thread::sleep(std::time::Duration::from_secs_f64(*exp_time));

    while !(camera.image_ready()?) {};

    let mut buf = vec![0u8;(chip_d*chip_w*2) as usize];
    camera.read_image(&mut buf)?;
    let mut data = vec![0u32; buf.len()/2];
    for i in 0..buf.len()/2 {
        data[i] = buf[2*i] as u32 + buf[2*i+1] as u32 * 256;
    }

    let shape: [usize; 2] = [chip_w as usize, chip_d as usize];
    Ok(fitrs::Hdu::new(&shape, data))
}
