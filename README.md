# gxccd
Rust wrapper around [`gxccd`](https://www.gxccd.com/) C library.

## Contributing
This repository is incomplete. All "advanced" features of the original library have been left out as they are unsupported. All other functions have been wrapped, but not all have been tested, though this is the goal. Idiomatic rust testing is complicated by the necessity of a connected camera, and by the opacity of the original library.

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
If you get some other large output regarding libraries, you probably need to configure your [dependencies](#dependencies). If you get something else entirely, please raise an issue.


## Dependencies
### `libusb-1.0`
This is a readily available library required by the `libgxccd` library. If you don't already have it, you can install it by (e.g.):
```bash
apt-get install libusb-1.0-0-dev
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