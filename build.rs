use std::path::PathBuf;

fn main() {
    let lib_path = PathBuf::from("lib");
    println!("cargo:rustc-link-search=native={}", lib_path.display());
    println!("cargo:rustc-link-lib=static=gxccd"); // Use "dylib" for shared library
    println!("cargo:rustc-link-lib=usb-1.0"); // Link to libusb
}
