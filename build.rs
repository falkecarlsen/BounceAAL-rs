#![allow(deprecated)]
extern crate bindgen;

use bindgen::CargoCallbacks;

use std::env;
use std::error::Error;
use std::path::PathBuf;
use vergen::EmitBuilder;

fn main() -> Result<(), Box<dyn Error>> {
    // set linkage to esp-idf
    embuild::espidf::sysenv::output();
    // set git and build info for internal use
    EmitBuilder::builder().all_git().all_build().emit()?;

    let driver = "vl53l5cx_api";
    // This is the directory where the `c` library is located.
    let libdir_path = PathBuf::from("drivers/vl53l5cx-uld")
        // Canonicalize the path as `rustc-link-search` requires an absolut path.
        .canonicalize()
        .expect("cannot canonicalize path");

    // This is the path to the `c` headers file.
    let driver_header = libdir_path.join(driver.to_owned() + ".h");
    let driver_headers_path_str = driver_header.to_str().expect("Path is not a valid string");

    // This is the path to the intermediate object file for our library.
    let driver_obj_path = libdir_path.join(driver.to_owned() + ".o");
    // This is the path to the static library file. Note *needs* prefix `lib`.
    let driver_lib_path = libdir_path.join("lib".to_owned() + driver + ".a");

    // Tell cargo to look for shared libraries in the specified directory
    println!("cargo:rustc-link-search={}", libdir_path.to_str().unwrap());

    // Tell cargo to tell rustc to link our `hello` library. Cargo will
    // automatically know it must look for a `libhello.a` file.
    println!("cargo:rustc-link-lib={}", driver);

    // Tell cargo to invalidate the built crate whenever the header changes.
    println!("cargo:rerun-if-changed={}", driver_headers_path_str);

    // Compile the `c` library by invoking `xtensa-esp32-elf-gcc` assuming it has been sourced to path.
    let clang_out = std::process::Command::new("xtensa-esp32-elf-gcc")
        .arg("-mlongcalls") // -mlongcalls needed for lib due to large code size?
        // (https://gcc.gnu.org/onlinedocs/gcc-3.4.6/gcc/Xtensa-Options.html)
        .arg("-c")
        .arg("-o")
        .arg(&driver_obj_path)
        .arg(libdir_path.join(driver.to_owned() + ".c"))
        .output()
        .unwrap();

    // output stdout and stderr for debugging
    //println!("stdout: {}", String::from_utf8_lossy(&clang_out.stdout));
    println!("stderr: {}", String::from_utf8_lossy(&clang_out.stderr));

    if clang_out.status.code() != Some(0) {
        // Panic if the command was not successful.
        panic!("could not compile: {:?}", driver_header);
    }

    // Run `ar` to generate the `libhello.a` file from the `hello.o` file.
    // Unwrap if it is not possible to spawn the process.
    if !std::process::Command::new("ar")
        .arg("rcs")
        .arg(driver_lib_path)
        .arg(driver_obj_path)
        .output()
        .expect("could not spawn `ar`")
        .status
        .success()
    {
        // Panic if the command was not successful.
        panic!("could not emit library file ");
    }

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .header(driver_headers_path_str)
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(CargoCallbacks))
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs");
    bindings
        .write_to_file(out_path)
        .expect("Couldn't write bindings!");
    Ok(())
}
