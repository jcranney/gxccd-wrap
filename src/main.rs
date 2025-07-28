use gxccd::*;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// how many frames to take
    #[arg(short, long, default_value_t = 1)]
    nframes: u32,
    /// exposure time in seconds
    #[arg(short, long, default_value_t = 1.0)]
    exp_time: f64,
    /// prefix to save for files, e.g., first frame: "<prefix>000.fits"
    #[arg(short, long, default_value_t = ("frame_".to_string()))]
    prefix: String,
    /// configure non-default camera read mode
    #[arg(short, long, )]
    readmode: Option<i32>,
    /// configure binning in x (if supported, otherwise ignored)
    #[arg(long, default_value_t = 1)]
    binning_x: i32,
    /// configure binning in y (if supported, otherwise ignored)
    #[arg(long, default_value_t = 1)]
    binning_y: i32,
    /// configure non-default camera gain
    #[arg(long, )]
    gain: Option<u16>,
    /// flag for closing shutter during exposure (normally open)
    #[arg(short, long)]
    dark: bool,

}

fn main() -> Result<(), String> {
    let Args { nframes, exp_time, prefix, binning_x, binning_y, gain, readmode, dark } = Args::parse();
    let camera = Camera::new()?;
    camera.set_binning(binning_x, binning_y)?;
    if let Some(g) = gain {
        camera.set_gain(g)?;
    }
    if let Some(rm) = readmode {
        camera.set_read_mode(rm)?;
    }
    for i in 0..nframes {
        let filename = format!("./{}{:03}.fits", prefix, i);
        println!("taking frame {}/{} -> {}", i, nframes, filename);
        let primary_hdu = take_full_frame(&camera, &exp_time, !dark)?;
        fitrs::Fits::create(filename, primary_hdu).map_err(|e| e.to_string())?;
    }    
    println!("took {} frames", nframes);
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

    let mut buf = vec![0u8;((chip_d*chip_w*2)) as usize];
    camera.read_image(&mut buf)?;
    let mut data = vec![0u32; buf.len()/2];
    for i in 0..buf.len()/2 {
        data[i] = buf[2*i] as u32 + buf[2*i+1] as u32 * 256;
    }

    let shape: [usize; 2] = [(chip_w) as usize, (chip_d) as usize];
    Ok(fitrs::Hdu::new(&shape, data))
}
