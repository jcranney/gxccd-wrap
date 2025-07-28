# gxccd
Rust wrapper around [`gxccd`](https://www.gxccd.com/) C library.

## Setup
```bash
git clone https://github.com/jcranney/gxccd-wrap
cd gxccd-wrap
cargo build --release
```
This will build the main `gxccd` rust program, which connects to a camera, prints some states of the camera, and saves some dark and light images to disk. Obviously this will fail if you do not have a camera connected, but it should fail cleanly with the message:
```
$ gxccd
Error: "Cannot find USB camera"
```

You may also see this error if you don't have the correct permissions on your USB device. Something like the following might clear that up:
```bash
sudo chmod -R 777 /dev/bus/usb
```

If you get some other large output regarding libraries, you probably need to configure your [dependencies](#dependencies). If you get something else entirely, please raise an issue.



## Dependencies
### `libusb-1.0`
This is a readily available library required by the `libgxccd` library. If you don't already have it, you can install it with `apt`:
```bash
apt-get install libusb-1.0-0-dev
```
or (e.g.), `dnf`:
```bash
dnf install libusb1-devel
```

## Usage
The main purpose of this library is to allow for more advanced rust-based software to interface with the camera. However, this library comes with a command line tool for taking frames from the camera.

```
$ gxccd --help
Rust wrapper and CLI tool around gxccd C library

Usage: gxccd [OPTIONS]

Options:
  -n, --nframes <NFRAMES>      how many frames to take [default: 1]
  -e, --exp-time <EXP_TIME>    exposure time in seconds [default: 1]
  -p, --prefix <PREFIX>        prefix to save for files, e.g., first frame: "<prefix>000.fits" [default: frame_]
  -r, --readmode <READMODE>    configure non-default camera read mode
      --binning-x <BINNING_X>  configure binning in x (if supported, otherwise ignored) [default: 1]
      --binning-y <BINNING_Y>  configure binning in y (if supported, otherwise ignored) [default: 1]
      --gain <GAIN>            configure non-default camera gain
  -d, --dark                   flag for closing shutter during exposure (normally open)
  -h, --help                   Print help
  -V, --version                Print version
```
and a command for reading all parameters from your camera:
```
$ gxccd-params
Camera description: C4-16000EC    
Camera FW version: 65.7.0
Camera chip temp: 28.38 °C
Camera supply voltage: 12.21 V
Read mode #0: 16-bit HDR
Read mode #1: 12-bit hi-gain
Read mode #2: 12-bit lo-gain
Read mode #3: "16-bit" lo-gain

------ Boolean Parameters ------
Connected: true
SubFrame: true
ReadModes: true
Shutter: true
Cooler: true
Fan: false
Filters: false
Guide: false
WindowHeating: true
Preflash: true
AsymmetricBinning: true
MicrometerFilterOffsets: false
PowerUtilization: true
Gain: true
ElectronicShutter: true
GPS: false
ContinuousExposures: false
Trigger: false
Configured: Failed to retrieve boolean parameter
RGB: false
CMY: false
CMYG: false
DebayerXOdd: false
DebayerYOdd: false
Interlaced: false
HexVersionNumber: true

------ Integer Parameters ------
CameraId: 80050
ChipW: 4096
ChipD: 4096
PixelW: 9000
PixelD: 9000
MaxBinningX: 4
MaxBinningY: 4
ReadModes: 4
Filters: 0
MinimalExposure: 21
MaximalExposure: 88648
MaximalMoveTime: Failed to retrieve integer parameter
DefaultReadMode: 0
PreviewReadMode: 1
MaxWindowHeating: 100
MaxFan: Failed to retrieve integer parameter
MaxGain: Failed to retrieve integer parameter
MaxPixelValue: 65535
FirmwareMajor: 65
FirmwareMinor: 7
FirmwareBuild: 0
DriverMajor: 0
DriverMinor: 9
DriverBuild: 0
FlashMajor: 65
FlashMinor: 8
FlashBuild: 0

------ String Parameters ------
CameraDescription: C4-16000EC    
Manufacturer: Moravian Instruments
CameraSerial: <hidden from prying eyes>
ChipDescription: GSENSE4040    

------ Values ------
ChipTemperature: 28.382845
HotTemperature: 28.868448
CameraTemperature: 28.868448
EnvironmentTemperature: 28.868448
SupplyVoltage: 12.213135
PowerUtilization: 0.085
ADCGain: 0.85
```
You can install both of these binaries standalone with:
```bash
$ cargo install gxccd
```

### `libgxccd`
This crate is packaged with a copy of the original [gxccd](https://www.gxccd.com/) C library binary available from Moravian Instruments. This is permitted by the `libgxccd` license (reproduced here below). For now, only the *x86_64 Linux (64-bit)* binaries are shipped with this crate, but you can replace them with the version matching your machine by replacing `./lib/libgxccd.a` with the same file for your machine, downloaded from the [Moravian Instruments download page](https://www.gxccd.com/cat?id=156). If this doesn't work for you, let me know by raising an issue.
```license
The Moravian Instruments (MI) camera library.
 
Copyright (c) 2016-2023, Moravian Instruments <http://www.gxccd.com, linux@gxccd.com>
All rights reserved.
 
Redistribution.  Redistribution and use in binary form, without
modification, are permitted provided that the following conditions are
met:
 
- Redistributions must reproduce the above copyright notice and the
  following disclaimer in the documentation and/or other materials
  provided with the distribution.
- Neither the name of Moravian Instruments nor the names of its
  suppliers may be used to endorse or promote products derived from this
  software without specific prior written permission.
- No reverse engineering, decompilation, or disassembly of this software
  is permitted.
 
DISCLAIMER.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
```
This repository is not supported by - or related to - Moravian Instruments in any way, and all of the code in this library is covered by the (more permissive) MIT license (see `./LICENSE`). If you wish to use the source code contained within this library, you need only satisfy the MIT license conditions, but if you include the Moravian Instruments binaries in `./lib/*` in any future redistributions, you must also satisfy the license copied above regarding the *redistribution and use in binary form*.

## Contributing
This repository is incomplete. All "advanced" features of the original library have been left out as they are unsupported. All other functions have been wrapped, but not all have been tested, though this is the goal. Idiomatic rust testing is complicated by the necessity of a connected camera, and by the opacity of the original library. Your contributions are encouraged, and if you choose to contribute, please do so through Github issues and pull requests.