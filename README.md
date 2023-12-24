# candleLight_gsusb, tweaked for Canable V2 (STM32F431 based) CAN-FD use 

***Note: This is not likely to be maintained***

Please feel free to fork and build on this.

Many of the changes here are based on:

https://github.com/ddleed/CANNEDPI_SW_GSUSB_CANFD by Ryan Edwards
https://github.com/normaldotcom/candleLight_fw (multitarget branch) by Ethan Zonca

I'm sure there are a good number of bugs that I've added. So far I've only really done basic bidirectional tests with CAN FD running at 5mbps.

This is firmware for certain STM32FG431-based USB-CAN adapters, notably:
- canable v2: http://canable.io/ (STM32G431x8)
- CANable-MKS V2.0: https://github.com/makerbase-mks/CANable-MKS (STM32G431x8)

This fork will currently only work on STM32G431 based adapters; it's probably easy to get it working on other G4's, and possibly H7's, but it requires the FDCAN peripheral.

This implements the interface of the mainline linux gs_usb kernel module and
works out-of-the-box with linux distros packaging this module, e.g. Ubuntu.

## Known issues

Be aware that there is a bug in the gs_usb module in linux<4.5 that can crash the kernel on device removal.

Here is a fixed version that should also work for older kernels:
  https://github.com/HubertD/socketcan_gs_usb

The Firmware also implements WCID USB descriptors and thus can be used on recent Windows versions without installing a driver.

## Building

Building requires arm-none-eabi-gcc toolchain.

```shell
sudo apt-get install gcc-arm-none-eabi

mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi-8-2019-q3-update.cmake

# or,
# cmake-gui ..
# don't forget to specify the cmake toolchain file before configuring.
#
# compile all targets :

make

# OR, each board target is a cmake option and can be disabled before running 'make';
# OR, compile a single target , e.g.
make canable2_fw

#
# to list possible targets :
make help

```

## Flashing

Flashing candleLight on linux: (source: [https://cantact.io/cantact/users-guide.html](https://cantact.io/cantact/users-guide.html))
- Flashing requires the dfu-util tool. On Ubuntu, this can be installed with `sudo apt install dfu-util`.
- compile as above, or download the current binary release: gsusb_cantact_8b2b2b4.bin
- If dfu-util fails due to permission issues on Linux, you may need additional udev rules. Consult your distro's documentation and see `70-candle-usb.rules` provided here.

### recommended simple method
- If compiling with cmake, `make flash-<targetname_fw>`, e.g. `make flash-canable_fw`, to invoke dfu-util.

### method for reflashing a specific device by serial
- when multiple devices are connected, dfu-util may be unable to choose which one to flash.
- Obtain device's serial # by looking at `dfu-util -l`
- adapt the following command accordingly :
 `dfu-util -D CORRECT_FIRMWARE.bin -S "serial_number_here", -a 0 -s 0x08000000:leave`
- note, the `:leave` suffix above may not be supported by older builds of dfu-util and is simply a convenient way to reboot into the normal firmware.

### fail-safe method (or if flashing a blank device)
- Disconnect the USB connector from the CANtact, short the BOOT pins, then reconnect the USB connector. The device should enumerate as "STM32 BOOTLOADER".

- invoke dfu-util manually with: `sudo dfu-util --dfuse-address -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D CORRECT_FIRMWARE.bin` where CORRECT_FIRMWARE is the name of the desired .bin.
- Disconnect the USB connector, un-short the BOOT pins, and reconnect.



## Associating persistent device names 
With udev on linux, it is possible to assign a device name to a certain serial number (see udev manpages and [systemd.link](https://www.freedesktop.org/software/systemd/man/systemd.link.html)).
This can be useful when multiple devices are connected at the same time.

An example configuration :

```
 $ cat /etc/systemd/network/60-persistent-candev.link 
[Match]
Property=ID_MODEL=cannette_gs_usb ID_SERIAL_SHORT="003800254250431420363230"

[Link]
# from systemd.link manpage:
# Note that specifying a name that the kernel might use for another interface (for example "eth0") is dangerous because the name assignment done by udev will race with the assignment done by the kernel, and only one
#   interface may use the name. Depending on the order of operations, either udev or the kernel will win, making the naming unpredictable. It is best to use some different prefix

Name=cannette99
```

( The serial number can be found with the `lsusb` utility). After reloading systemd units and resetting this board :

```
 $ ip a
....
59: cannette99: <NOARP,ECHO> mtu 16 qdisc noop state DOWN group default qlen 10
    link/can 
 $
```


## Hacking
### Submitting pull requests
- Each commit must not contain unrelated changes (e.g. functional and whitespace changes)
- Project must be compilable (with default options) and functional, at each commit. 
- Squash any "WIP" or other temporary commits.
- Make sure your editor is not messing up whitespace or line-ends.
- We include both a `.editorconfig` and `uncrustify.cfg` which should help with whitespace.

Typical command to run uncrustify on all source files (ignoring HAL and third-party libs):
`uncrustify -c ./uncrustify.cfg --replace $(find include src -name "*.[ch]")`

Optionally append `--no-backup` to avoid creating .orig files.

### Profiling
Not great on cortex-M0 cores (F042, F072 targets etc) since they lack hardware support (ITM and SWO). However, it's possible to randomly sample the program counter and get some coarse profiling info.

For example, openocd has the `profile` command (see https://openocd.org/doc/html/General-Commands.html#Misc-Commands), e.g.

```profile 5 test.out 0x8000000 0x8100000```

(from inside gdb, the command needs to be prefixed with `monitor` to forward it to openocd, i.e. `monitor profile 5 .....`.

The .out file can then be processed with `gprof <firmware_name> -l test.out`



## Links to related projects
* [Cangaroo](https://github.com/HubertD/cangaroo) open source can bus analyzer software
* [Candle.NET](https://github.com/elliotwoods/Candle.NET) .NET wrapper for the candle API
