## gxccd
Rust wrapper around [`gxccd`](https://www.gxccd.com/) C library.

## Dependencies
### `libgxccd`
This crate is a wrapper around the [gxccd](https://www.gxccd.com/) C library provided by Moravian Instruments, and as such, redistributes the original library in binary form (`./lib/libgxccd.a`) without modification, as permitted by the `libgxccd` license (reproduced here):
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