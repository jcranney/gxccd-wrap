use std::{path::PathBuf, str::FromStr};

use gxccd::*;
use clap::{builder::Str, Parser, Subcommand};

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    //#[command(subcommand)]
    //command: Commands,
    #[clap(alias = "e")]
    exposure: Option<f64>,
    #[clap(alias = "f")]
    filename: Option<String>,
}

/*#[derive(Subcommand, Debug, Clone)]
enum Commands {
    Snap {
        nframes: Option<u32>,
    },
}*/

fn main() -> Result<(), String> {
    let args = Args::parse();
    let exposure: f64 = match &args.exposure {
        Some(x) => *x,
        None => 1.0,
    };
    let filename: PathBuf = match &args.filename {
        Some(fname) => PathBuf::from_str(&fname).or_else(|e| Err(e.to_string()))?,
        None => PathBuf::from_str("./out.fits").or_else(|e| Err(e.to_string()))?,
    };
    let camera = Camera::new()?;
    eprintln!("taking light frames");
    camera.set_read_mode(3)?;
    let primary_hdu = take_full_frame(&camera, &exposure, true)?;
    fitrs::Fits::create(filename, primary_hdu).map_err(|e| e.to_string())?;
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
