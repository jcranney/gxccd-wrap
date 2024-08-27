extern crate libc;

/*
 * The Moravian Instruments (MI) camera library.
 *
 * Copyright (c) 2016-2023, Moravian Instruments <http://www.gxccd.com, linux@gxccd.com>
 * All rights reserved.
 *
 * Redistribution.  Redistribution and use in binary form, without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * - Redistributions must reproduce the above copyright notice and the
 *   following disclaimer in the documentation and/or other materials
 *   provided with the distribution.
 * - Neither the name of Moravian Instruments nor the names of its
 *   suppliers may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 * - No reverse engineering, decompilation, or disassembly of this software
 *   is permitted.
 *
 * DISCLAIMER.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
*/

pub use wrap::{
    BooleanParams,
    IntegerParams,
    StringParams,
    Values,
    TimeStamp,
    Filter,
    GPSData,
    Gain,
};

/// Convenience struct to create and interact with an initialised camera
pub struct Camera {
    _camera: *mut wrap::Camera,
}

impl Camera {
    /// Find camera and connect to it.
    /// 
    /// returns error if no camera connected. Undefined behaivour if multiple cameras
    /// are connected (raise a github issue if you need this).
    pub fn new() -> Result<Self, String> {
        let id = wrap::enumerate_usb()?;
        Ok(Camera {
            _camera: wrap::initialize_usb(id)
        })
    }


    /// Open camera shutter.
    pub fn open_shutter(&self) -> Result<(), String> {
        wrap::open_shutter(self._camera)
    }

    /// Close camera shutter.
    pub fn close_shutter(&self) -> Result<(), String> {
        wrap::close_shutter(self._camera)
    }

    /// Disconnects from camera and releases other resources. The memory pointed by
    /// "camera" becomes invalid and you must not pass it to any of the following
    /// functions!
    pub fn release(&self) {
        wrap::release(self._camera)
    }

    pub fn get_boolean_parameter(&self, param: BooleanParams) -> Result<bool, String> {
        wrap::get_boolean_parameter(self._camera, param)
    }

    pub fn get_integer_parameter(&self, param: IntegerParams) -> Result<i32, String> {
        wrap::get_integer_parameter(self._camera, param)
    }

    pub fn get_string_parameter(&self, param: StringParams) -> Result<String, String> {
        wrap::get_string_parameter(self._camera, param)
    }

    pub fn get_value(&self, param: Values) -> Result<f32, String> {
        wrap::get_value(self._camera, param)
    }

    /// Sets the required chip temperature.
    /// If the camera has no cooler, this function has no effect.
    /// "temp" is expressed in degrees Celsius.
    pub fn set_temperature(&self, temp: f32) -> Result<(), String> {
        wrap::set_temperature(self._camera, temp)
    }

    /// Sets the maximum speed with which the driver changes chip temperature.
    /// If the camera has no cooler, this function has no effect.
    /// "temp_ramp" is expressed in degrees Celsius per minute.
    pub fn set_temperature_ramp(&self, temp_ramp: f32) -> Result<(), String> {
        wrap::set_temperature_ramp(self._camera, temp_ramp)
    }

    /// Sets the required read binning.
    /// If the camera does not support binning, this function has no effect.
    pub fn set_binning(&self, x: i32, y: i32) -> Result<(), String> {
        wrap::set_binning(self._camera, x, y)
    }

    /// If the camera is equipped with preflash electronics, this function sets it.
    /// "preflash_time" defines time for which the preflash LED inside the camera is
    /// switched on. "clear_num" defines how many times the chip has to be cleared
    /// after the preflash. Actual values of these parameters depends on
    /// the particular camera model (e.g. number and luminance of the LEDs used etc.).
    /// Gx series of cameras typically need less than 1 second to completely
    /// saturate the chip ("preflash_time"). Number of subsequent clears should be
    /// at last 2, but more than 4 or 5 clears is not useful, too.
    pub fn set_preflash(&self, preflash_time: f64, clear_num: i32) -> Result<(), String> {
        wrap::set_preflash(self._camera, preflash_time, clear_num)
    }

    /// Starts new exposure.
    /// "exp_time" is the required exposure time in seconds. "use_shutter" parameter
    /// tells the driver the dark frame (without light) has to be acquired (false),
    /// or the shutter has to be opened and closed to acquire normal light image (true).
    /// Sub-frame coordinates are passed in "x", "y", "w" and "h".
    /// If the camera does not support sub-frame read, "x" and "y" must be 0 and "w"
    /// and "h" must be the chip pixel dimensions.
    /// The y-axis grows down, 0 is at the top.
    pub fn start_exposure(&self, exp_time: f64, use_shutter: bool, x: i32, y: i32, width: i32, height: i32) -> Result<(), String> {
        wrap::start_exposure(self._camera, exp_time, use_shutter, x, y, width, height)
    }

    /// The camera enters waiting state and when a signal is detected on trigger port,
    /// it starts a new exposure.
    /// The parameters are the same as for gxccd_start_exposure.
    pub fn start_exposure_trigger(&self, exp_time: f64, use_shutter: bool, x: i32, y: i32, width: i32, height: i32) -> Result<(), String> {
        wrap::start_exposure_trigger(self._camera, exp_time, use_shutter, x, y, width, height)
    }

    /// When the exposure already started by gxccd_start_exposure() call has to be
    /// terminated before the exposure time expires, this function has to be called.
    /// Parameter "download" indicates whether the image should be digitized, because
    /// the user will call gxccd_read_image() later or the image should be discarded.
    pub fn abort_exposure(&self, download: bool) -> Result<(), String> {
        wrap::abort_exposure(self._camera, download)
    }

    /// When the exposure already started by gxccd_start_exposure() call, parameter
    /// "ready" is false if the exposure is still running. When the exposure finishes
    /// and it is possible to call gxccd_read_image(), parameter "ready" is true.
    /// It is recommended to count the exposure time in the application despite
    /// the fact the exact exposure time is determined by the camera/driver and to
    /// start calling of gxccd_image_ready() only upon the exposure time expiration.
    /// Starting to call gxccd_image_ready() in the infinite loop immediately after
    /// gxccd_start_exposure() and call it for the whole exposure time (and thus
    /// keeping at last one CPU core at 100% utilization) is a bad programming
    /// manner (politely expressed).
    pub fn image_ready(&self) -> Result<bool, String> {
        wrap::image_ready(self._camera)
    }

    /// When gxccd_image_ready() returns "ready" == true, it is possible to call
    /// gxccd_read_image(). Driver returns 16 bits per pixel (2 bytes) matrix copied
    /// to "buf" address. The buffer must be allocated by the caller, driver does not
    /// allocate any memory. The "size" parameter specifies allocated memory block
    /// length in bytes (not in pixels!). It has to be greater or equal to image size
    /// in bytes else the function fails.
    /// Application can use: size = wanted_w /// 2 /// wanted_h;
    ///
    /// Format of the returned buffer:
    ///   - one-dimensional array formed from lines (rows) stacked one after another
    ///   - orientation of the image is similar to Cartesian coordinate system,
    ///     pixel [0, 0] (first line) is located at bottom left of the resulting image,
    ///     x coordinate grows right and y coordinate grows up
    ///   - data is in little-endian encoding -> lower byte first
    ///
    /// Example with width = 2000px and height = 1000px:
    ///   - allocate one-dimensional buffer: malloc(2000*2*1000) (malloc(width*2*height))
    ///   - bottom left pixel's lower byte is located at buffer[0] and higher byte is
    ///     at buffer[1]
    ///   - first line has width = 2000 /// 2 (bytes) -> bottom right pixel is located
    ///     at buffer[3998] and buffer[3999] (width /// 2 - 2 and width /// 2 - 1)
    ///   - top left pixel is at buffer[3996000] and buffer[3996001]
    ///     ((height - 1) /// width /// 2 and (height - 1) /// width /// 2 + 1)
    ///   - top right pixel is at buffer[3999998] and buffer[3999999]
    ///     ((height - 1) /// width /// 2 + width /// 2 - 2 and (height - 1) /// width /// 2
    ///     + width /// 2 - 1)
    pub fn read_image(&self, buf: &mut [u8]) -> Result<(), String> {
        wrap::read_image(self._camera, buf)
    }

    /// Functionality and parameters are the same as for gxccd_read_image.
    /// The only difference is that the camera starts another exposure before returning
    /// the image. The time between continous exposures is therefore reduced by the time
    /// of downloading the image to the PC.
    pub fn read_image_exposure(&self,  buf: &mut [u8]) -> Result<(), String> { 
        wrap::read_image_exposure(self._camera, buf)
    }

    /// Enumerates all read modes provided by the camera.
    /// This enumeration does not use any callback, the caller passes index
    /// beginning with 0 and repeats the call with incremented index until the call
    /// returns -1.
    /// The caller must specify the size of the buffer in parameter "size".
    pub fn enumerate_read_modes(&self, index: i32) -> Result<String, String> {
        wrap::enumerate_read_modes(self._camera, index)
    }

    /// Sets required read mode.
    /// "mode" is the "index" used in gxccd_enumerate_read_modes() call.
    pub fn set_read_mode(&self, mode: i32) -> Result<(), String> {
        wrap::set_read_mode(self._camera, mode)
    }

    /// Sets required gain. Range of parameter "gain" depends on particular camera
    /// hardware, as it typically represents directly a register value.
    /// This method is chosen to allow to control gain as precisely as each particular
    /// camera allows. Low limit is 0, high limit is returned by function
    /// gxccd_get_integer_parameter with index GIP_MAX_GAIN.
    pub fn set_gain(&self, gain: u16) -> Result<(), String> { 
        wrap::set_gain(self._camera, gain)
    }


    /// As the gxccd_set_gain function accepts camera-dependent parameter gain,
    /// which typically does not represent actual gain as signal multiply or value in dB,
    /// a helper function gxccd_convert_gain is provided to convert register value into
    /// gain in logarithmic dB units as well as in linear times-signal units.
    pub fn convert_gain(&self, gain: u16) -> Result<Gain, String> { 
        wrap::convert_gain(self._camera, gain)
    }


    /// Enumerates all filters provided by the camera.
    /// This enumeration does not use any callback, by the caller passes index
    /// beginning with 0 and repeats the call with incremented index until the call
    /// returns -1.
    /// Returns filter name in "buf". The caller must specify the size of the buffer
    /// in parameter size.
    /// "color" parameter hints the RGB color (e.g. cyan color is 0x00ffff), which
    /// can be used to draw the filter name in the application.
    /// "offset" indicates the focuser shift when the particular filter is selected.
    /// Units of the "offset" can be micrometers or arbitrary focuser specific units
    /// (steps). If the units used are micrometers, driver returns true from
    /// gxccd_get_boolean_parameter() with GBP_MICROMETER_FILTER_OFFSETS "index".
    pub fn enumerate_filters(&self, index: i32) -> Result<Filter, String> {
        wrap::enumerate_filters(self._camera, index)
    }

    /// Sets the required filter.
    /// If the camera is not equipped with filter wheel, this function has no effect.
    pub fn set_filter(&self, index: i32) -> Result<(), String> {
        wrap::set_filter(self._camera, index)
    }

    /// Reinitializes camera filter wheel.
    /// 
    /// returns the number of detected
    /// filters or 0 in case of error (or camera without filter wheel).
    pub fn reinit_filter_wheel(&self) -> Result<i32, String> {
        wrap::reinit_filter_wheel(self._camera)
    }

    /// If the camera is equipped with cooling fan and allows its control,
    /// this function sets the fan rotation speed.
    /// The maximum value of the "speed" parameter should be determined by
    /// gxccd_get_integer_parameter() call with GIP_MAX_FAN "index".
    /// If the particular camera supports only on/off switching, the maximum value
    /// should be 1 (fan on), while value 0 means fan off.
    pub fn set_fan(&self, speed: u8) -> Result<(), String> {
        wrap::set_fan(self._camera, speed)
    }

    /// If the camera is equipped with chip cold chamber front window heater
    /// and allows its control, this function sets heating intensity.
    /// The maximum value of the "heating" parameter should be determined by
    /// gxccd_get_integer_parameter() call with GIP_MAX_WINDOW_HEATING "index".
    /// If the particular camera supports only on/off switching, the maximum value
    /// should be 1 (heating on), while value 0 means heating off.
    pub fn set_window_heating(&self, heating: u8) -> Result<(), String> {
        wrap::set_window_heating(self._camera, heating)
    }

    /// Instructs the camera to initiate telescope movement in the R.A. and/or Dec.
    /// axis for the defined period of time (in milliseconds).
    /// The sign of the parameters defines the movement direction in the respective
    /// coordinate. The maximum length is approx 32.7 seconds.
    /// If the camera is not equipped with autoguider port, this function has no
    /// effect.
    pub fn move_telescope(&self, ra_duration_ms: i16, dec_duration_ms: i16) -> Result<(), String> {
        wrap::move_telescope(self._camera, ra_duration_ms, dec_duration_ms)
    }

    /// Sets "moving" to true if the movement started with gxccd_move_telescope()
    /// call is still in progress.
    pub fn move_in_progress(&self) -> Result<bool, String> {
        wrap::move_in_progress(self._camera)
    }

    /// Returns actual date and exact time of the last image exposure.
    /// Date and time is obtained from GPS and is in UTC time standard.
    /// Subsecond precision is additionally achieved with internal camera counter.
    /// For this function to work, the camera must contain a GPS receiver module and
    /// it must be synchronized with at least 5 satellites. You can call
    /// "get_gps_data()" to obtain GPS status.
    pub fn get_image_time_stamp(&self) -> Result<TimeStamp, String> {
        wrap::get_image_time_stamp(self._camera)
    }

    /// Returns actual date, exact time, latitude, longitude, mean sea level and
    /// status of GPS module. Date and time is in UTC time standard. Subsecond precision
    /// is additionally achieved with internal camera counter. For this function to work,
    /// the camera must contain a GPS receiver module. Position information needs at
    /// least 3 satellites, date and time is returned after synchronization with 5
    /// satellites.
    pub fn get_gps_data(&self) -> Result<GPSData, String> {
        wrap::get_gps_data(self._camera)
    }

    /// If any call fails (returns -1), this function returns failure description
    /// in parameter buf.
    /// The caller must specify the size of the buffer in parameter "size".
    pub fn get_last_error(&self) -> String {
        wrap::get_last_error(self._camera)
    }
}

/// Direct wrappers of C functions
pub mod wrap {
    use libc::{c_char, c_double, c_float, c_int, c_uint, c_void, size_t};
    use std::ffi::CStr;
    use std::path::PathBuf;
    use std::str::FromStr;
    use std::sync::Mutex;
    /// Opaque camera object used to interact with C binary
    #[repr(C)]
    pub struct Camera {
        _private: [u8; 0],
    }

    /// Standard [get_boolean_parameter()] parameters
    #[derive(Debug)]
    pub enum BooleanParams {
        Connected = 0,              // true if camera currently connected
        SubFrame,                  // true if camera supports sub-frame read
        ReadModes,                 // true if camera supports multiple read modes
        Shutter,                    // true if camera is equipped with mechanical shutter
        Cooler,                     // true if camera is equipped with active chip cooler
        Fan,                        // true if camera fan can be controlled
        Filters,                    // true if camera controls filter wheel
        Guide,                      // true if camera is capable to guide the telescope mount
        WindowHeating,             // true if camera can control the chip window heating
        Preflash,                   // true if camera can use chip preflash
        AsymmetricBinning,         // true if camera horizontal and vertical binning can differ
        MicrometerFilterOffsets,  // true if filter focusing offsets are expressed in micrometers
        PowerUtilization,          // true if camera can return power utilization in gxccd_get_value()
        Gain,                       // true if camera can return gain in gxccd_get_value()
        ElectronicShutter,         // true if camera has electronic shutter
        GPS,                        // true if camera has gps module
        ContinuousExposures,       // true if camera supports continuous exposures
        Trigger,                    // true if camera supports trigger port
        Configured = 127,           // true if camera is configured
        RGB,                        // true if camera has Bayer RGBG filters on the chip
        CMY,                        // true if camera has CMY filters on the chip
        CMYG,                       // true if camera has CMYG filters on the chip
        DebayerXOdd,              // true if camera Bayer masks starts on horizontal odd pixel
        DebayerYOdd,              // true if camera Bayer masks starts on vertical odd pixel
        Interlaced = 256,           // true if chip is interlaced (else progressive)
        HexVersionNumber = 1024   // true if GIP_FIRMWARE_MAJOR should be represented as hexadecimal number
    }

    /// Standard [get_integer_parameter()] parameters
    #[derive(Debug)]
    pub enum IntegerParams {
        CameraId = 0,          // Identifier of the current camera
        ChipW,                 // Chip width in pixels
        ChipD,                 // Chip depth in pixels
        PixelW,                // Chip pixel width in nanometers
        PixelD,                // Chip pixel depth in nanometers
        MaxBinningX,          // Maximum binning in horizontal direction
        MaxBinningY,          // Maximum binning in vertical direction
        ReadModes,             // Number of read modes offered by the camera
        Filters,                // Number of filters offered by the camera
        MinimalExposure,       // Shortest exposure time in microseconds (µs)
        MaximalExposure,       // Longest exposure time in milliseconds (ms)
        MaximalMoveTime,      // Longest time to move the telescope in milliseconds (ms)
        DefaultReadMode,      // Read mode to be used as default
        PreviewReadMode,      // Read mode to be used for preview (fast read)
        MaxWindowHeating,     // Maximal value for gxccd_set_window_heating()
        MaxFan,                // Maximal value for gxccd_set_fan()
        MaxGain,               // Maximal value for gxccd_set_gain()
        MaxPixelValue,        // Maximal possible pixel value. May vary with read mode and binning,
                                    // read only after gxccd_set_read_mode() and gxccd_set_binning() calls
        FirmwareMajor = 128,   // Camera firmware version (optional)
        FirmwareMinor,
        FirmwareBuild,
        DriverMajor,           // This library version
        DriverMinor,
        DriverBuild,
        FlashMajor,            // Flash version (optional)
        FlashMinor,
        FlashBuild,
    }


    /// Standard [get_string_parameter()] parameters 
    #[derive(Debug)]
    pub enum StringParams {
        CameraDescription = 0, // Camera description 
        Manufacturer,           // Manufacturer name 
        CameraSerial,          // Camera serial number 
        ChipDescription        // Used chip description 
    }

    /// Standard [get_value()] values
    #[derive(Debug)]
    pub enum Values {
        ChipTemperature = 0,     // Current temperature of the chip in deg. Celsius
        HotTemperature,          // Current temperature of the cooler hot side
                                // in deg. Celsius
        CameraTemperature,       // Current temperature inside the camera
                                // in deg. Celsius
        EnvironmentTemperature,  // Current temperature of the environment air
                                // in deg. Celsius
        SupplyVoltage = 10,      // Current voltage of the camera power supply
        PowerUtilization,        // Current utilization of the chip cooler
                                // (rational number from 0.0 to 1.0)
        ADCGain = 20             // Current gain of A/D converter in electrons/ADU
    }

    extern "C" {
        fn gxccd_open_shutter(camera: *mut Camera) -> c_int;
        fn gxccd_close_shutter(camera: *mut Camera) -> c_int;

        /// Sets global path to your configuration .ini file.
        /// You can pass NULL or empty string to use lookup process described above.
        fn gxccd_configure(ini_path: *mut c_char);

        /// Configures ip address and/or port of ethernet adapter.
        /// To configure port only, pass NULL or empty string in "ip".
        /// To configure ip address only, pass 0 in "port".
        fn gxccd_configure_eth(ip: *mut c_char, port: u16);

        /// Enumerates all cameras currently connected to your computer (_usb) or the
        /// ethernet adapter (_eth).
        /// You have to provide callback function, which is called for each connected
        /// camera and the camera identifier (camera ID) is passed as an argument to this
        /// callback function.
        /// If your application is designed to work with one camera only or the camera
        /// identifier is known, gxccd_enumerate_*() needs not to be called at all.

        /// USB variant 
        fn gxccd_enumerate_usb(callback: extern "C" fn(eid: libc::c_int));
        /// Ethernet variant 
        fn gxccd_enumerate_eth(callback: extern "C" fn(eid: libc::c_int));

        /// Driver is designed to handle multiple cameras at once. It distinguishes
        /// individual cameras using pointer to individual instances.
        /// This function returns pointer to initialized structure or NULL in case of
        /// error.
        /// "camera_id" is camera indentifier (e.g. obtained by gxccd_enumerate_*()
        /// function) and is required. If you have one camera connected, you can pass -1
        /// as "camera_id".
        ///USB variant 
        fn gxccd_initialize_usb(camera_id: c_int) -> *mut Camera;
        ///Ethernet variant 
        fn gxccd_initialize_eth(camera_id: c_int) -> *mut Camera;

        /// Disconnects from camera and releases other resources. The memory pointed by
        /// "camera" becomes invalid and you must not pass it to any of the following
        /// functions!
        fn gxccd_release(camera: *mut Camera);

        ///==============================================================================
        /// RULES FOR THE FOLLOWING FUNCTIONS:
        /// 1. Each function works with initialized camera_t structure.
        /// 2. Every parameter is required.
        /// 3. On success returns 0, on error the value -1 is returned and application
        ///    can obtain error explanation by calling gxccd_get_last_error().


        ///Returns true or false in "value" depending on the "index". 
        fn gxccd_get_boolean_parameter(camera: *mut Camera, index: c_int, value: *mut bool) -> c_int;

        ///Returns integer in "value" depending on the "index". 
        fn gxccd_get_integer_parameter(camera: *mut Camera, index: c_int, value: *mut c_int) -> c_int;


        /// Returns string in "buf" depending on the "index".
        /// The caller must specify the size of the buffer in "size".
        fn gxccd_get_string_parameter(camera: *mut Camera, index: c_int, buf: *mut c_char, size: size_t) -> c_int;



        ///Returns float in "value" depending on the "index". 
        fn gxccd_get_value(camera: *mut Camera, index: c_int, value: *mut c_float) -> c_int;

        /// Sets the required chip temperature.
        /// If the camera has no cooler, this function has no effect.
        /// "temp" is expressed in degrees Celsius.
        fn gxccd_set_temperature(camera: *mut Camera, temp: c_float) -> c_int;

        /// Sets the maximum speed with which the driver changes chip temperature.
        /// If the camera has no cooler, this function has no effect.
        /// "temp_ramp" is expressed in degrees Celsius per minute.
        fn gxccd_set_temperature_ramp(camera: *mut Camera, temp_ramp: c_float) -> c_int;

        /// Sets the required read binning.
        /// If the camera does not support binning, this function has no effect.
        fn gxccd_set_binning(camera: *mut Camera, x: c_int, y: c_int) -> c_int;

        /// If the camera is equipped with preflash electronics, this function sets it.
        /// "preflash_time" defines time for which the preflash LED inside the camera is
        /// switched on. "clear_num" defines how many times the chip has to be cleared
        /// after the preflash. Actual values of these parameters depends on
        /// the particular camera model (e.g. number and luminance of the LEDs used etc.).
        /// Gx series of cameras typically need less than 1 second to completely
        /// saturate the chip ("preflash_time"). Number of subsequent clears should be
        /// at last 2, but more than 4 or 5 clears is not useful, too.
        fn gxccd_set_preflash(camera: *mut Camera, preflash_time: c_double, clear_num: c_int) -> c_int;

        /// Starts new exposure.
        /// "exp_time" is the required exposure time in seconds. "use_shutter" parameter
        /// tells the driver the dark frame (without light) has to be acquired (false),
        /// or the shutter has to be opened and closed to acquire normal light image (true).
        /// Sub-frame coordinates are passed in "x", "y", "w" and "h".
        /// If the camera does not support sub-frame read, "x" and "y" must be 0 and "w"
        /// and "h" must be the chip pixel dimensions.
        /// The y-axis grows down, 0 is at the top.
        fn gxccd_start_exposure(camera: *mut Camera, exp_time: c_double, use_shutter: bool, x: c_int, y: c_int, w: c_int, h: c_int) -> c_int;

        /// The camera enters waiting state and when a signal is detected on trigger port,
        /// it starts a new exposure.
        /// The parameters are the same as for gxccd_start_exposure.
        fn gxccd_start_exposure_trigger(camera: *mut Camera,  exp_time: c_double, use_shutter: bool, x: c_int, y: c_int, w: c_int, h: c_int) -> c_int;

        /// When the exposure already started by gxccd_start_exposure() call has to be
        /// terminated before the exposure time expires, this function has to be called.
        /// Parameter "download" indicates whether the image should be digitized, because
        /// the user will call gxccd_read_image() later or the image should be discarded.
        fn gxccd_abort_exposure(camera: *mut Camera, download: bool) -> c_int;

        /// When the exposure already started by gxccd_start_exposure() call, parameter
        /// "ready" is false if the exposure is still running. When the exposure finishes
        /// and it is possible to call gxccd_read_image(), parameter "ready" is true.
        /// It is recommended to count the exposure time in the application despite
        /// the fact the exact exposure time is determined by the camera/driver and to
        /// start calling of gxccd_image_ready() only upon the exposure time expiration.
        /// Starting to call gxccd_image_ready() in the infinite loop immediately after
        /// gxccd_start_exposure() and call it for the whole exposure time (and thus
        /// keeping at last one CPU core at 100% utilization) is a bad programming
        /// manner (politely expressed).
        fn gxccd_image_ready(camera: *mut Camera, ready: *mut bool) -> c_int;

        /// When gxccd_image_ready() returns "ready" == true, it is possible to call
        /// gxccd_read_image(). Driver returns 16 bits per pixel (2 bytes) matrix copied
        /// to "buf" address. The buffer must be allocated by the caller, driver does not
        /// allocate any memory. The "size" parameter specifies allocated memory block
        /// length in bytes (not in pixels!). It has to be greater or equal to image size
        /// in bytes else the function fails.
        /// Application can use: size = wanted_w /// 2 /// wanted_h;

        /// Format of the returned buffer:
        ///   - one-dimensional array formed from lines (rows) stacked one after another
        ///   - orientation of the image is similar to Cartesian coordinate system,
        ///     pixel [0, 0] (first line) is located at bottom left of the resulting image,
        ///     x coordinate grows right and y coordinate grows up
        ///   - data is in little-endian encoding -> lower byte first

        /// Example with width = 2000px and height = 1000px:
        ///   - allocate one-dimensional buffer: malloc(2000*2*1000) (malloc(width*2*height))
        ///   - bottom left pixel's lower byte is located at buffer[0] and higher byte is
        ///     at buffer[1]
        ///   - first line has width = 2000 /// 2 (bytes) -> bottom right pixel is located
        ///     at buffer[3998] and buffer[3999] (width /// 2 - 2 and width /// 2 - 1)
        ///   - top left pixel is at buffer[3996000] and buffer[3996001]
        ///     ((height - 1) /// width /// 2 and (height - 1) /// width /// 2 + 1)
        ///   - top right pixel is at buffer[3999998] and buffer[3999999]
        ///     ((height - 1) /// width /// 2 + width /// 2 - 2 and (height - 1) /// width /// 2
        ///     + width /// 2 - 1)
        fn gxccd_read_image(camera: *mut Camera, buf: *mut c_void, size: size_t) -> c_int;

        /// Functionality and parameters are the same as for gxccd_read_image.
        /// The only difference is that the camera starts another exposure before returning
        /// the image. The time between continous exposures is therefore reduced by the time
        /// of downloading the image to the PC.
        fn gxccd_read_image_exposure(camera: *mut Camera, buf: *mut c_void, size: size_t) -> c_int;

        /// Enumerates all read modes provided by the camera.
        /// This enumeration does not use any callback, the caller passes index
        /// beginning with 0 and repeats the call with incremented index until the call
        /// returns -1.
        /// The caller must specify the size of the buffer in parameter "size".
        fn gxccd_enumerate_read_modes(camera: *mut Camera, index: c_int, buf: *mut c_char, size: size_t) -> c_int;

        /// Sets required read mode.
        /// "mode" is the "index" used in gxccd_enumerate_read_modes() call.
        fn gxccd_set_read_mode(camera: *mut Camera, mode: c_int) -> c_int;

        /// Sets required gain. Range of parameter "gain" depends on particular camera
        /// hardware, as it typically represents directly a register value.
        /// This method is chosen to allow to control gain as precisely as each particular
        /// camera allows. Low limit is 0, high limit is returned by function
        /// gxccd_get_integer_parameter with index GIP_MAX_GAIN.
        fn gxccd_set_gain(camera: *mut Camera, gain: u16) -> c_int;

        /// As the gxccd_set_gain function accepts camera-dependent parameter gain,
        /// which typically does not represent actual gain as signal multiply or value in dB,
        /// a helper function gxccd_convert_gain is provided to convert register value into
        /// gain in logarithmic dB units as well as in linear times-signal units.
        fn gxccd_convert_gain(camera: *mut Camera, gain: u16, db: *mut c_double, times: *mut c_double) -> c_int;

        /// Enumerates all filters provided by the camera.
        /// This enumeration does not use any callback, by the caller passes index
        /// beginning with 0 and repeats the call with incremented index until the call
        /// returns -1.
        /// Returns filter name in "buf". The caller must specify the size of the buffer
        /// in parameter size.
        /// "color" parameter hints the RGB color (e.g. cyan color is 0x00ffff), which
        /// can be used to draw the filter name in the application.
        /// "offset" indicates the focuser shift when the particular filter is selected.
        /// Units of the "offset" can be micrometers or arbitrary focuser specific units
        /// (steps). If the units used are micrometers, driver returns true from
        /// gxccd_get_boolean_parameter() with GBP_MICROMETER_FILTER_OFFSETS "index".
        fn gxccd_enumerate_filters(camera: *mut Camera, index: c_int, buf: *mut c_char, size: size_t, color: *mut u32, offset: *mut c_int) -> c_int;

        /// Sets the required filter.
        /// If the camera is not equipped with filter wheel, this function has no effect.
        fn gxccd_set_filter(camera: *mut Camera, index: c_int) -> c_int;

        /// Reinitializes camera filter wheel.
        /// If parameter "num_filter" is not NULL, it will contain the number of detected
        /// filters or 0 in case of error (or camera without filter wheel).
        fn gxccd_reinit_filter_wheel(camera: *mut Camera, num_filters: *mut c_int) -> c_int;

        /// If the camera is equipped with cooling fan and allows its control,
        /// this function sets the fan rotation speed.
        /// The maximum value of the "speed" parameter should be determined by
        /// gxccd_get_integer_parameter() call with GIP_MAX_FAN "index".
        /// If the particular camera supports only on/off switching, the maximum value
        /// should be 1 (fan on), while value 0 means fan off.
        fn gxccd_set_fan(camera: *mut Camera, speed: u8) -> c_int;

        /// If the camera is equipped with chip cold chamber front window heater
        /// and allows its control, this function sets heating intensity.
        /// The maximum value of the "heating" parameter should be determined by
        /// gxccd_get_integer_parameter() call with GIP_MAX_WINDOW_HEATING "index".
        /// If the particular camera supports only on/off switching, the maximum value
        /// should be 1 (heating on), while value 0 means heating off.
        fn gxccd_set_window_heating(camera: *mut Camera, heating: u8) -> c_int;

        /// Instructs the camera to initiate telescope movement in the R.A. and/or Dec.
        /// axis for the defined period of time (in milliseconds).
        /// The sign of the parameters defines the movement direction in the respective
        /// coordinate. The maximum length is approx 32.7 seconds.
        /// If the camera is not equipped with autoguider port, this function has no
        /// effect.
        fn gxccd_move_telescope(camera: *mut Camera, ra_duration_ms: i16, dec_duration_ms: i16) -> c_int;

        /// Sets "moving" to true if the movement started with gxccd_move_telescope()
        /// call is still in progress.
        fn gxccd_move_in_progress(camera: *mut Camera, moving: *mut bool) -> c_int;

        /// Returns actual date and exact time of the last image exposure.
        /// Date and time is obtained from GPS and is in UTC time standard.
        /// Subsecond precision is additionally achieved with internal camera counter.
        /// For this function to work, the camera must contain a GPS receiver module and
        /// it must be synchronized with at least 5 satellites. You can call
        /// "gxccd_get_gps_data()" to obtain GPS status.
        fn gxccd_get_image_time_stamp(camera: *mut Camera, year: *mut c_int,month: *mut c_int, day: *mut c_int, hour: *mut c_int, minute: *mut c_int, second: *mut c_double) -> c_int;

        /// Returns actual date, exact time, latitude, longitude, mean sea level and
        /// status of GPS module. Date and time is in UTC time standard. Subsecond precision
        /// is additionally achieved with internal camera counter. For this function to work,
        /// the camera must contain a GPS receiver module. Position information needs at
        /// least 3 satellites, date and time is returned after synchronization with 5
        /// satellites.
        fn gxccd_get_gps_data(camera: *mut Camera, lat: *mut c_double, lon: *mut c_double, msl: *mut c_double,
                            year: *mut c_int, month: *mut c_int, day: *mut c_int, hour: *mut c_int, minute: *mut c_int,
                            second: *mut c_double, satellites: *mut c_uint, fix: *mut bool) -> c_int;

        /// If any call fails (returns -1), this function returns failure description
        /// in parameter buf.
        /// The caller must specify the size of the buffer in parameter "size".
        fn gxccd_get_last_error(camera: *mut Camera, buf: *mut c_char, size: size_t);
    }

    /// Open camera shutter.
    pub fn open_shutter(camera: *mut Camera) -> Result<(), String> {
        // Call the C function
        let result = unsafe { gxccd_open_shutter(camera) };

        // Check for errors
        if result != 0 {
            Err("Failed to open shutter".into())
        } else {
            Ok(())
        }
    }

    /// Close camera shutter.
    pub fn close_shutter(camera: *mut Camera) -> Result<(), String> {
        // Call the C function
        let result = unsafe { gxccd_close_shutter(camera) };

        // Check for errors
        if result != 0 {
            Err("Failed to close shutter".into())
        } else {
            Ok(())
        }
    }

    /// Sets global path to your configuration .ini file.
    /// You can pass NULL or empty string to use lookup process described above.
    pub fn configure(ini_path: Option<PathBuf>) -> Result<PathBuf,String> {
        let mut buf: Vec<c_char>;
        match ini_path {
            Some(path) => buf = path.display().to_string().chars().map(|x| x as i8).collect(),
            None => buf = vec![],
        }
        let out: PathBuf;
        unsafe {
            gxccd_configure(buf.as_mut_ptr());
            out = PathBuf::from_str(CStr::from_ptr(buf.as_ptr()).to_str().unwrap()).unwrap()
        }
        Ok(out)
    }

    /// Configures ip address and/or port of ethernet adapter.
    /// To configure port only, pass NULL or empty string in "ip".
    /// To configure ip address only, pass 0 in "port".
    pub fn configure_eth(ip: &str, port: u16) {
        unsafe {
            let mip = ip.to_string().as_mut_ptr().cast::<i8>();
            gxccd_configure_eth(mip, port);
        }
    }

    static ID: Mutex<i32> = Mutex::new(-1);

    extern "C" fn rust_callback(eid: c_int) {
        let mut id = ID.lock().unwrap();
        *id = eid;
    }

    /// Enumerates all cameras currently connected to your computer _usb
    /// 
    /// If your application is designed to work with one camera only or the camera
    /// identifier is known, [enumerate_usb()] needs not to be called at all.
    pub fn enumerate_usb() -> Result<i32, String> {
        unsafe {
            gxccd_enumerate_usb(rust_callback);
            let id = ID.lock().unwrap();
            if *id == -1 {
                Err("Cannot find USB camera".to_string())
            } else {
                Ok(*id)
            }
        }
    }

    /// Enumerates all cameras currently connected to your computer ethernet adapter (_eth)
    /// 
    /// If your application is designed to work with one camera only or the camera
    /// identifier is known, [enumerate_eth()] needs not to be called at all.
    pub fn enumerate_eth() -> Result<i32, String> {
        unsafe {
            gxccd_enumerate_eth(rust_callback);
            let id = ID.lock().unwrap();
            Ok(*id)
        }
    }

    /// Driver is designed to handle multiple cameras at once. It distinguishes
    /// individual cameras using pointer to individual instances.
    /// This function returns pointer to initialized structure or NULL in case of
    /// error.
    /// "camera_id" is camera indentifier (e.g. obtained by gxccd_enumerate_*()
    /// function) and is required. If you have one camera connected, you can pass -1
    /// as "camera_id".
    ///USB variant 
    pub fn initialize_usb(camera_id: i32) -> *mut Camera {
        unsafe { gxccd_initialize_usb(camera_id) }
    }
    ///Ethernet variant 
    pub fn initialize_eth(camera_id: i32) -> *mut Camera {
        unsafe { gxccd_initialize_eth(camera_id) }
    }

    /// Disconnects from camera and releases other resources. The memory pointed by
    /// "camera" becomes invalid and you must not pass it to any of the following
    /// functions!
    pub fn release(camera: *mut Camera) {
        unsafe { gxccd_release(camera)}
    }

    /// RULES FOR THE FOLLOWING FUNCTIONS:
    /// 1. Each function works with initialized camera_t structure.
    /// 2. Every parameter is required.
    /// 3. On success returns 0, on error the value -1 is returned and application
    ///    can obtain error explanation by calling gxccd_get_last_error().

    pub fn get_boolean_parameter(camera: *mut Camera, param: BooleanParams) -> Result<bool, String> {
        let mut value: bool = false;
        unsafe {
            if 0 == gxccd_get_boolean_parameter(camera, param as c_int, &mut value) {
                Ok(value)
            } else {
                Err("Failed to retrieve integer parameter".to_string())
            }
        }
    }

    pub fn get_integer_parameter(camera: *mut Camera, param: IntegerParams) -> Result<i32, String> {
        let mut value: c_int = 0;
        unsafe {
            if 0 == gxccd_get_integer_parameter(camera, param as c_int, &mut value) {
                Ok(value)
            } else {
                Err("Failed to retrieve integer parameter".to_string())
            }
        }
    }

    pub fn get_string_parameter(camera: *mut Camera, param: StringParams) -> Result<String, String> {
        let mut buf = vec![0 as c_char; 256];
        unsafe {
            if 0 == gxccd_get_string_parameter(camera, param as c_int, buf.as_mut_ptr(), buf.len()) {
                Ok(CStr::from_ptr(buf.as_ptr()).to_string_lossy().into_owned())
            } else {
                Err("Failed to retrieve string parameter".into())
            }
        }
    }

    ///Returns float in "value" depending on the "index". 
    pub fn get_value(camera: *mut Camera, param: Values) -> Result<f32, String> {
        let mut value: c_float = 0.0;
        unsafe {
            if 0 == gxccd_get_value(camera, param as c_int, &mut value) {
                Ok(value)
            } else {
                Err("Failed to retrieve value".to_string())
            }
        }
    }

    /// Sets the required chip temperature.
    /// If the camera has no cooler, this function has no effect.
    /// "temp" is expressed in degrees Celsius.
    pub fn set_temperature(camera: *mut Camera, temp: f32) -> Result<(), String> {
        unsafe {
            if 0 == gxccd_set_temperature(camera, temp) {
                Ok(())
            } else {
                Err("Failed to set temperature".to_string())
            }
        }
    }

    /// Sets the maximum speed with which the driver changes chip temperature.
    /// If the camera has no cooler, this function has no effect.
    /// "temp_ramp" is expressed in degrees Celsius per minute.
    pub fn set_temperature_ramp(camera: *mut Camera, temp_ramp: f32) -> Result<(), String> {
        unsafe {
            if 0 == gxccd_set_temperature_ramp(camera, temp_ramp) {
                Ok(())
            } else {
                Err("Failed to set temperature ramp".to_string())
            }
        }
    }

    /// Sets the required read binning.
    /// If the camera does not support binning, this function has no effect.
    pub fn set_binning(camera: *mut Camera, x: i32, y: i32) -> Result<(), String> {
        unsafe {
            if 0 == gxccd_set_binning(camera, x, y) {
                Ok(())
            } else {
                Err("Failed to set binning".to_string())
            }
        }
    }

    /// If the camera is equipped with preflash electronics, this function sets it.
    /// "preflash_time" defines time for which the preflash LED inside the camera is
    /// switched on. "clear_num" defines how many times the chip has to be cleared
    /// after the preflash. Actual values of these parameters depends on
    /// the particular camera model (e.g. number and luminance of the LEDs used etc.).
    /// Gx series of cameras typically need less than 1 second to completely
    /// saturate the chip ("preflash_time"). Number of subsequent clears should be
    /// at last 2, but more than 4 or 5 clears is not useful, too.
    pub fn set_preflash(camera: *mut Camera, preflash_time: f64, clear_num: i32) -> Result<(), String> {
        unsafe {
            if 0 == gxccd_set_preflash(camera, preflash_time, clear_num) {
                Ok(())
            } else {
                Err("Failed to set preflash".to_string())
            }
        }
    }

    /// Starts new exposure.
    /// "exp_time" is the required exposure time in seconds. "use_shutter" parameter
    /// tells the driver the dark frame (without light) has to be acquired (false),
    /// or the shutter has to be opened and closed to acquire normal light image (true).
    /// Sub-frame coordinates are passed in "x", "y", "w" and "h".
    /// If the camera does not support sub-frame read, "x" and "y" must be 0 and "w"
    /// and "h" must be the chip pixel dimensions.
    /// The y-axis grows down, 0 is at the top.
    pub fn start_exposure(camera: *mut Camera, exp_time: f64, use_shutter: bool, x: i32, y: i32, width: i32, height: i32) -> Result<(), String> {
        unsafe {
            // Convert Rust boolean to C int (true = 1, false = 0)
            
            let result = gxccd_start_exposure(
                camera as *mut _,
                exp_time as c_double,
                use_shutter,
                x as c_int,
                y as c_int,
                width as c_int,
                height as c_int,
            );

            if result != 0 {
                Err(format!("Error starting exposure: {}", result))
            } else {
                Ok(())
            }
        }
    }

    /// The camera enters waiting state and when a signal is detected on trigger port,
    /// it starts a new exposure.
    /// The parameters are the same as for gxccd_start_exposure.
    pub fn start_exposure_trigger(camera: *mut Camera, exp_time: f64, use_shutter: bool, x: i32, y: i32, width: i32, height: i32) -> Result<(), String> {
        unsafe {
            let result = gxccd_start_exposure_trigger(
                camera as *mut _,
                exp_time as c_double,
                use_shutter,
                x as c_int,
                y as c_int,
                width as c_int,
                height as c_int,
            );

            if result != 0 {
                Err(format!("Error starting exposure: {}", result))
            } else {
                Ok(())
            }
        }
    }

    /// When the exposure already started by gxccd_start_exposure() call has to be
    /// terminated before the exposure time expires, this function has to be called.
    /// Parameter "download" indicates whether the image should be digitized, because
    /// the user will call gxccd_read_image() later or the image should be discarded.
    pub fn abort_exposure(camera: *mut Camera, download: bool) -> Result<(), String> {
        unsafe {
            if 0 == gxccd_abort_exposure(camera, download) {
                Ok(())
            } else {
                Err("Failed to abort exposure".to_string())
            }
        }
    }

    /// When the exposure already started by gxccd_start_exposure() call, parameter
    /// "ready" is false if the exposure is still running. When the exposure finishes
    /// and it is possible to call gxccd_read_image(), parameter "ready" is true.
    /// It is recommended to count the exposure time in the application despite
    /// the fact the exact exposure time is determined by the camera/driver and to
    /// start calling of gxccd_image_ready() only upon the exposure time expiration.
    /// Starting to call gxccd_image_ready() in the infinite loop immediately after
    /// gxccd_start_exposure() and call it for the whole exposure time (and thus
    /// keeping at last one CPU core at 100% utilization) is a bad programming
    /// manner (politely expressed).
    pub fn image_ready(camera: *mut Camera) -> Result<bool, String> {
        unsafe {
            let mut ready: bool = false;
            let result = gxccd_image_ready(camera, &mut ready);

            if result != 0 {
                Err(format!("Error checking image readiness: {}", result))
            } else {
                Ok(ready)
            }
        }
    }

    /// When gxccd_image_ready() returns "ready" == true, it is possible to call
    /// gxccd_read_image(). Driver returns 16 bits per pixel (2 bytes) matrix copied
    /// to "buf" address. The buffer must be allocated by the caller, driver does not
    /// allocate any memory. The "size" parameter specifies allocated memory block
    /// length in bytes (not in pixels!). It has to be greater or equal to image size
    /// in bytes else the function fails.
    /// Application can use: size = wanted_w /// 2 /// wanted_h;
    ///
    /// Format of the returned buffer:
    ///   - one-dimensional array formed from lines (rows) stacked one after another
    ///   - orientation of the image is similar to Cartesian coordinate system,
    ///     pixel [0, 0] (first line) is located at bottom left of the resulting image,
    ///     x coordinate grows right and y coordinate grows up
    ///   - data is in little-endian encoding -> lower byte first
    ///
    /// Example with width = 2000px and height = 1000px:
    ///   - allocate one-dimensional buffer: malloc(2000*2*1000) (malloc(width*2*height))
    ///   - bottom left pixel's lower byte is located at buffer[0] and higher byte is
    ///     at buffer[1]
    ///   - first line has width = 2000 /// 2 (bytes) -> bottom right pixel is located
    ///     at buffer[3998] and buffer[3999] (width /// 2 - 2 and width /// 2 - 1)
    ///   - top left pixel is at buffer[3996000] and buffer[3996001]
    ///     ((height - 1) /// width /// 2 and (height - 1) /// width /// 2 + 1)
    ///   - top right pixel is at buffer[3999998] and buffer[3999999]
    ///     ((height - 1) /// width /// 2 + width /// 2 - 2 and (height - 1) /// width /// 2
    ///     + width /// 2 - 1)
    pub fn read_image(camera: *mut Camera, buf: &mut [u8]) -> Result<(), String> {
        unsafe {
            let result = gxccd_read_image(camera as *mut _, buf.as_mut_ptr() as *mut c_void, buf.len());

            if result != 0 {
                Err(format!("Error reading image: {}", result))
            } else {
                Ok(())
            }
        }
    }

    /// Functionality and parameters are the same as for gxccd_read_image.
    /// The only difference is that the camera starts another exposure before returning
    /// the image. The time between continous exposures is therefore reduced by the time
    /// of downloading the image to the PC.
    pub fn read_image_exposure(camera: *mut Camera,  buf: &mut [u8]) -> Result<(), String> { 
        unsafe {
            let result = gxccd_read_image_exposure(camera as *mut _, buf.as_mut_ptr() as *mut c_void, buf.len());

            if result != 0 {
                Err(format!("Error reading image: {}", result))
            } else {
                Ok(())
            }
        }
    }

    /// Enumerates all read modes provided by the camera.
    /// This enumeration does not use any callback, the caller passes index
    /// beginning with 0 and repeats the call with incremented index until the call
    /// returns -1.
    /// The caller must specify the size of the buffer in parameter "size".
    pub fn enumerate_read_modes(camera: *mut Camera, index: i32) -> Result<String, String> {
        // Allocate a buffer for the read mode
        let mut buf = vec![0 as c_char; 256]; 

        // Call the C function
        let result = unsafe {
            gxccd_enumerate_read_modes(camera, index, buf.as_mut_ptr(), buf.len() as libc::size_t)
        };

        if result != 0 {
            return Err("Error enumerating read modes".into());
        }

        // Convert the C string to a Rust string
        let mode_cstr = unsafe { CStr::from_ptr(buf.as_ptr()) };
        let mode_str = mode_cstr.to_str().map_err(|e| e.to_string())?;

        Ok(mode_str.to_string())
    }

    /// Sets required read mode.
    /// "mode" is the "index" used in gxccd_enumerate_read_modes() call.
    pub fn set_read_mode(camera: *mut Camera, mode: i32) -> Result<(), String> {
        // Call the C function
        let result = unsafe { gxccd_set_read_mode(camera, mode) };

        // Check for errors
        if result != 0 {
            Err("Failed to set read mode".into())
        } else {
            Ok(())
        }
    }

    /// Sets required gain. Range of parameter "gain" depends on particular camera
    /// hardware, as it typically represents directly a register value.
    /// This method is chosen to allow to control gain as precisely as each particular
    /// camera allows. Low limit is 0, high limit is returned by function
    /// gxccd_get_integer_parameter with index GIP_MAX_GAIN.
    pub fn set_gain(camera: *mut Camera, gain: u16) -> Result<(), String> { 
        unsafe {
            if 0 == gxccd_set_gain(camera, gain) {
                Ok(())
            } else {
                Err("Failed to set gain".to_string())
            }
        }
    }

    /// gain values returned from [convert_gain()] function
    /// 
    /// not a very useful struct, but clearer than returning a tuple.
    #[derive(Debug)]
    pub struct Gain {
        pub db: f64,
        pub times: f64,
    }
    /// As the gxccd_set_gain function accepts camera-dependent parameter gain,
    /// which typically does not represent actual gain as signal multiply or value in dB,
    /// a helper function gxccd_convert_gain is provided to convert register value into
    /// gain in logarithmic dB units as well as in linear times-signal units.
    pub fn convert_gain(camera: *mut Camera, gain: u16) -> Result<Gain, String> { 
        let mut db: c_double = 0.0;
        let mut times: c_double = 0.0;
        unsafe {
            if 0 == gxccd_convert_gain(camera, gain, &mut db, &mut times) {
                Ok(Gain { db: db as f64, times: times as f64 })
            } else {
                Err("Failed to convert gain".to_string())
            }
        }
    }

    /// Filter parameters retrieved from [enumerate_filters()] function
    pub struct Filter {
        pub name: String,
        pub color: u32,
        pub offset: i32,
    }
    /// Enumerates all filters provided by the camera.
    /// This enumeration does not use any callback, by the caller passes index
    /// beginning with 0 and repeats the call with incremented index until the call
    /// returns -1.
    /// Returns filter name in "buf". The caller must specify the size of the buffer
    /// in parameter size.
    /// "color" parameter hints the RGB color (e.g. cyan color is 0x00ffff), which
    /// can be used to draw the filter name in the application.
    /// "offset" indicates the focuser shift when the particular filter is selected.
    /// Units of the "offset" can be micrometers or arbitrary focuser specific units
    /// (steps). If the units used are micrometers, driver returns true from
    /// gxccd_get_boolean_parameter() with GBP_MICROMETER_FILTER_OFFSETS "index".
    pub fn enumerate_filters(camera: *mut Camera, index: i32) -> Result<Filter, String> {
        let mut buf = vec![0 as c_char; 256];
        let mut color: u32 = 0;
        let mut offset: c_int = 0;
        unsafe {
            if 0 == gxccd_enumerate_filters(camera, index, buf.as_mut_ptr(), buf.len(), &mut color, &mut offset) {
                Ok(Filter {
                    name: CStr::from_ptr(buf.as_ptr()).to_string_lossy().into_owned(),
                    color,
                    offset,
                })
            } else {
                Err("Failed to enum filters".to_string())
            }
        }
    }

    /// Sets the required filter.
    /// If the camera is not equipped with filter wheel, this function has no effect.
    pub fn set_filter(camera: *mut Camera, index: i32) -> Result<(), String> {
        unsafe {
            if 0 == gxccd_set_filter(camera, index) {
                Ok(())
            } else {
                Err("unable to set filter".to_string())
            }
        }
    }

    /// Reinitializes camera filter wheel.
    /// If parameter "num_filter" is not NULL, it will contain the number of detected
    /// filters or 0 in case of error (or camera without filter wheel).
    pub fn reinit_filter_wheel(camera: *mut Camera) -> Result<i32, String> {
        let mut num_filters: c_int = 0;
        unsafe {
            if 0 == gxccd_reinit_filter_wheel(camera, &mut num_filters) {
                Ok(num_filters)
            } else {
                Err("Unable to reinit filter wheel".to_string())
            }
        }
    }

    /// If the camera is equipped with cooling fan and allows its control,
    /// this function sets the fan rotation speed.
    /// The maximum value of the "speed" parameter should be determined by
    /// gxccd_get_integer_parameter() call with GIP_MAX_FAN "index".
    /// If the particular camera supports only on/off switching, the maximum value
    /// should be 1 (fan on), while value 0 means fan off.
    pub fn set_fan(camera: *mut Camera, speed: u8) -> Result<(), String> {
        unsafe {
            if 0 == gxccd_set_fan(camera, speed) {
                Ok(())
            } else {
                Err("Failed to set fan speed".to_string())
            }
        }
    }

    /// If the camera is equipped with chip cold chamber front window heater
    /// and allows its control, this function sets heating intensity.
    /// The maximum value of the "heating" parameter should be determined by
    /// gxccd_get_integer_parameter() call with GIP_MAX_WINDOW_HEATING "index".
    /// If the particular camera supports only on/off switching, the maximum value
    /// should be 1 (heating on), while value 0 means heating off.
    pub fn set_window_heating(camera: *mut Camera, heating: u8) -> Result<(), String> {
        unsafe {
            if 0 == gxccd_set_window_heating(camera, heating) {
                Ok(())
            } else {
                Err("Failed to set window heating".to_string())
            }
        }
    }

    /// Instructs the camera to initiate telescope movement in the R.A. and/or Dec.
    /// axis for the defined period of time (in milliseconds).
    /// The sign of the parameters defines the movement direction in the respective
    /// coordinate. The maximum length is approx 32.7 seconds.
    /// If the camera is not equipped with autoguider port, this function has no
    /// effect.
    pub fn move_telescope(camera: *mut Camera, ra_duration_ms: i16, dec_duration_ms: i16) -> Result<(), String> {
        unsafe {
            if 0 == gxccd_move_telescope(camera, ra_duration_ms, dec_duration_ms) {
                Ok(())
            } else {
                Err("Failed to move telescope".to_string())
            }
        }
    }

    /// Sets "moving" to true if the movement started with gxccd_move_telescope()
    /// call is still in progress.
    pub fn move_in_progress(camera: *mut Camera) -> Result<bool, String> {
        let mut moving: bool = false;
        unsafe {
            if 0 == gxccd_move_in_progress(camera, &mut moving) {
                Ok(moving)
            } else {
                Err("Failed to check moving status".to_string())
            }
        }
    }

    /// Timestamp data retrieved from GPS (if GPS is enabled)
    #[derive(Debug,Clone)]
    pub struct TimeStamp {
        pub year: i32,
        pub month: i32,
        pub day: i32,
        pub hour: i32,
        pub minute: i32,
        pub second: f64,
    }
    /// Returns actual date and exact time of the last image exposure.
    /// Date and time is obtained from GPS and is in UTC time standard.
    /// Subsecond precision is additionally achieved with internal camera counter.
    /// For this function to work, the camera must contain a GPS receiver module and
    /// it must be synchronized with at least 5 satellites. You can call
    /// "get_gps_data()" to obtain GPS status.
    pub fn get_image_time_stamp(camera: *mut Camera) -> Result<TimeStamp, String> {
        let mut year: c_int = 0;
        let mut month: c_int = 0;
        let mut day: c_int = 0;
        let mut hour: c_int = 0;
        let mut minute: c_int = 0;
        let mut second: c_double = 0.0;
        unsafe {
            if 0 == gxccd_get_image_time_stamp(camera, &mut year, &mut month, &mut day, &mut hour, &mut minute, &mut second) {
                Ok(TimeStamp{
                    year,
                    month,
                    day,
                    hour,
                    minute,
                    second,
                })
            } else {
                Err("Failed to get image time stamp".to_string())
            }
        }
    }

    /// Full set of GPS data retrieved from [get_gps_data()] function.
    #[derive(Debug,Clone)]
    pub struct GPSData {
        pub lat: f64,
        pub lon: f64,
        pub msl: f64,
        pub timestamp: TimeStamp,
        pub satellites: u32,
        pub fix: bool,
    }

    /// Returns actual date, exact time, latitude, longitude, mean sea level and
    /// status of GPS module. Date and time is in UTC time standard. Subsecond precision
    /// is additionally achieved with internal camera counter. For this function to work,
    /// the camera must contain a GPS receiver module. Position information needs at
    /// least 3 satellites, date and time is returned after synchronization with 5
    /// satellites.
    pub fn get_gps_data(camera: *mut Camera) -> Result<GPSData, String> {
        let mut lat: c_double = 0.0;
        let mut lon: c_double = 0.0;
        let mut msl: c_double = 0.0;
        let mut year: c_int = 0;
        let mut month: c_int = 0;
        let mut day: c_int = 0;
        let mut hour: c_int = 0;
        let mut minute: c_int = 0;
        let mut second: c_double = 0.0;
        let mut satellites: c_uint = 0;
        let mut fix: bool = false;
        unsafe {
            if 0 == gxccd_get_gps_data(camera, &mut lat, &mut lon, &mut msl, &mut year, &mut month, &mut day, &mut hour, &mut minute, &mut second, &mut satellites, &mut fix) {
                Ok(GPSData {
                    lat,
                    lon,
                    msl,
                    timestamp: TimeStamp {
                        year,
                        month,
                        day,
                        hour,
                        minute,
                        second,
                    },
                    satellites,
                    fix,
                })
            } else {
                Err("Failed to get gps data".to_string())
            }
        }
    }

    /// If any call fails (returns -1), this function returns failure description
    /// in parameter buf.
    /// The caller must specify the size of the buffer in parameter "size".
    pub fn get_last_error(camera: *mut Camera) -> String {
        let mut buf = vec![0 as c_char; 256];
        unsafe {
            gxccd_get_last_error(camera, buf.as_mut_ptr(), buf.len());
            CStr::from_ptr(buf.as_ptr()).to_string_lossy().into_owned()
        }
    }
}