# Configuring DFU over BLE using the LPN example

Device Firmware Upgrade (DFU) over BLE is the process that provides ability to update the application,
SoftDevice, and bootloader through BLE.


**Table of contents**
- [Hardware requirements](@ref examples_lpn_dfu_ble_requirements_hw)
- [Software requirements](@ref examples_lpn_dfu_ble_requirements_sw)
- [Setup](@ref examples_lpn_dfu_ble_setup)
- [Configuring DFU over BLE](@ref examples_lpn_dfu_ble_configure)
    - [Creating signature for the Low Power node example](@ref examples_lpn_dfu_ble_create_signature)
    - [Generating a firmware package with the Low Power node example](@ref examples_lpn_dfu_ble_generate_dfu_package)
    - [Building and programming the bootloader](@ref examples_lpn_dfu_ble_program_bootloader)
    - [Performing DFU over BLE](@ref examples_lpn_dfu_ble_perform_dfu)


---

## Hardware requirements @anchor examples_lpn_dfu_ble_requirements_hw

Taobao nRF52832 one button, one led

---

## Software requirements @anchor examples_lpn_dfu_ble_requirements_sw

Install the following additional tools:
- @link_ic_nrfutil for generating the application signature and the firmware package. See @link_nrfutil_installing
for details.
- Depending on whether you want to perform DFU using mobile or PC:
    - @link_nRFConnectMobile (@link_nrf_connect_mobile_ios or @link_nrf_connect_mobile_android) for performing DFU using a mobile phone.


---

## Setup @anchor examples_lpn_dfu_ble_setup

The nRF5 SDK project files for the DFU over BLE can be found at: `<the path to nRF5 SDK instance>/examples/dfu`.

You can find the source code of the Secure Bootloader example at: `<path to nRF5 SDK instance>/examples/dfu/secure_bootloader`.

The LPN device uses [PCA10040 Development Kit](@ref examples_lpn_requirements_hw).
 You can find the Secure Bootloader project files for this board in the `.../pca10040_ble` folder. 
 
 **Change BOARD_PCA10040 to BOARD_NRF52832_TB**
 **add boards.h and nrf52832_tb.h into `<path to nRF5 SDK instance>/components/boards/`**

---


## Configuring DFU over BLE @anchor examples_lpn_dfu_ble_configure


### Creating signature for the Low Power node example @anchor examples_lpn_dfu_ble_create_signature

To create the keys required for the DFU process:  
0. install nrfutil:
```
pip install nrfutil
```
1. Create a private key:
```
nrfutil keys generate lpn_private_key.pem
```
2. Create a public key in code format and store it in a file named `dfu_public_key.c`:
```
nrfutil keys display --key pk --format code lpn_private_key.pem --out_file dfu_public_key.c
```
3. Replace `dfu_public_key.c` file in the `<the path to nRF5 SDK instance>/examples/dfu` folder with the new one.

@note See @link_working_with_keys and @link_bootloader_signature_verification for more information about signatures.

### Generating a firmware package with the Low Power node example @anchor examples_lpn_dfu_ble_generate_dfu_package

To generate a firmware package:
1. Make sure the DFU over BLE support is enabled (@ref BLE_DFU_SUPPORT_ENABLED is set to 1
in examples/experimental_lpn/include/app_config.h).
2. Build the Low Power node example. To build the example, follow the instructions in
[Building the mesh stack](@ref md_doc_getting_started_how_to_build).
3. Generate a firmware package with the Low Power node example by using the
Low Power node hex file and the private key generated when building the example:
```
nrfutil pkg generate --application <path-to-lpn-example-hex-file> --application-version <application-version> --hw-version 52 --sd-req 0xCB --key-file lpn_private_key.pem lpn_dfu_package.zip
```
    In this command:
        - Replace `<path-to-lpn-example-hex-file>` with the path to the LPN example HEX file and the file name.
        - Replace `<application-version>` with any positive number.
            - After the first time upgrade, make sure that each next update has the application version number greater than the current.
        - `--sd-req` can be obtained with the following command:
```
nrfutil pkg generate --help
```


### Building and programming the bootloader @anchor examples_lpn_dfu_ble_program_bootloader

To perform DFU over BLE update for the Low Power node example, you must build and program the @link_secure_bootloader.  
**Use Segger emProject instead, bootloader setting is optional**


**Programming**<br>
To program the bootloader:
1. You can use one of the following options:
    - Program using SES: Follow the [Running examples using SEGGER Embedded Studio](@ref how_to_run_examples_ses)
    instruction.
2. Observe LED on the device. 

### Performing DFU over BLE @anchor examples_lpn_dfu_ble_perform_dfu

The Device Firmware Upgrade over BLE can be performed using either a mobile phone or PC.

**Mobile**<br>
To perform the DFU transfer over BLE using a mobile phone:
1. Copy the generated `lpn_dfu_package.zip` firmware package to your mobile phone.
2. Use @link_nRFConnectMobile to scan for a target device and connect to it:
    - If the DFU is performed for the first time, the device will show up as "DfuTarget".
    - If the Low Power node example application is already programmed, the device will show up as "nRF5x Mesh LPN Switch".
3. Press the DFU button to perform DFU.
4. Provide the zip archive of the application when prompted.


---

## Usage
Click button to trigger friendship invitation
LED off when invitation is succesfully established 

### Serial output
JLinkExe -device nRF52 -speed 4000 -if SWD     in one terminal
JLinkRTTClient               in another terminal


nrfutil settings generate --family NRF52 --application _build/ruuvi_firmware.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 settings.hex
mergehex -m ~/git/s132_nrf52_3.1.0_softdevice.hex ~/git/ruuvitag_b_bootloader_1.0.0.hex settings.hex -o sbc.hex
mergehex -m sbc.hex _build/ruuvi_firmware.hex -o packet.hex
nrfjprog --family nrf52 --eraseall
nrfjprog --family nrf52 --program packet.hex
nrfjprog --family nrf52 --reset

or 
nrfjprog --family nrf52 --eraseall
nrfjprog --family nrf52 --program latest_softdevice.hex
nrfjprog --family nrf52 --program latest_bootloader.hex
nrfutil pkg generate --debug-mode --application _build/ruuvi_firmware.hex --hw-version 3 --sd-req 0x91 --key-file ~/git/ruuvitag_fw/keys/ruuvi_open_private.pem ruuvi_firmware_dfu.zip

<t:      13864>, mesh_app_utils.c,   65, Device UUID (raw): 01A87100002B2DF8A76393A43E020000
<t:      13867>, mesh_app_utils.c,   70, Device UUID : 0071A801-2B00-F82D-A763-93A43E020000