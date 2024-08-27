use gxccd::*;

fn main() -> Result<(), String> {
    // Enumerate USB cameras
    let id = enumerate_usb()?;
    eprintln!("{}", id);

    let camera = initialize_usb(id);

    let param = get_string_parameter(camera, StringParams::CAMERA_DESCRIPTION)?;
    eprintln!("Camera description: {}", param);

    let major = get_integer_parameter(camera, IntegerParams::FIRMWARE_MAJOR)?;
    let minor = get_integer_parameter(camera, IntegerParams::FIRMWARE_MINOR)?;
    let build = get_integer_parameter(camera, IntegerParams::DRIVER_BUILD)?;
    eprintln!("Camera FW version: {}.{}.{}", major, minor, build);

    let temp = get_value(camera, Values::CHIP_TEMPERATURE)?;
    eprintln!("Camera chip temp: {:0.2} °C", temp);

    let voltage = get_value(camera, Values::SUPPLY_VOLTAGE)?;
    eprintln!("Camera supply voltage: {:0.2} V", voltage);
    
    let mut i = 0;
    while let Ok(result) = enumerate_read_modes(camera, i) {
        eprintln!("Read mode #{}: {}", i, result);
        i += 1;
    }

    eprintln!("taking dark frames");
    set_read_mode(camera, 3)?;
    let exp_time: f64 = 2.0;
    for i in 0..10 {
        eprintln!("taking frame {}", i);
        let primary_hdu = take_full_frame(camera, &exp_time, false)?;
        fitrs::Fits::create(format!("./dark_{:03}.fits",i), primary_hdu).or_else(|e| Err(e.to_string()))?;
    }    
    eprintln!("taking light frames");
    set_read_mode(camera, 3)?;
    let exp_time: f64 = 2.0;
    for i in 0..10 {
        eprintln!("taking frame {}", i);
        let primary_hdu = take_full_frame(camera, &exp_time, true)?;
        fitrs::Fits::create(format!("./light_{:03}.fits",i), primary_hdu).or_else(|e| Err(e.to_string()))?;
    }    
    Ok(())
}

fn take_full_frame(camera: *mut Camera, exp_time: &f64, light: bool) -> Result<fitrs::Hdu, String> {
    // start exposure
    let chip_w = get_integer_parameter(camera, IntegerParams::CHIP_W)?;
    let chip_d = get_integer_parameter(camera, IntegerParams::CHIP_D)?;
    
    start_exposure(camera, *exp_time, light, 0, 0, chip_w, chip_d)?;
    // sleep during exposure
    std::thread::sleep(std::time::Duration::from_secs_f64(*exp_time));

    while image_ready(camera)? == false {};

    let mut buf = vec![0u8;(chip_d*chip_w*2) as usize];
    read_image(camera, &mut buf)?;
    let mut data = vec![0u32; buf.len()/2];
    for i in 0..buf.len()/2 {
        data[i] = buf[2*i] as u32 + buf[2*i+1] as u32 * 256;
    }

    let shape: [usize; 2] = [chip_w as usize, chip_d as usize];
    Ok(fitrs::Hdu::new(&shape, data))
}
