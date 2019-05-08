# nRF-Radio-802.15.4-Projects
This repository contains sample projects for sending strings between two terminals using Nordic's [nRF IEEE 802.15.4 Radio Driver](https://github.com/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver).

## HW Requirements
- 2x nRF52840 Development Kit (PCA10056)

## SW Requirements
- nRF5 SDK v15.3.0 [download page](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v15.x.x/)

## Setting up environment
Download and extract nRF52 SDK v15.3.0 to a suitable location. Open CMD in the SDK root directory and clone the repository using these commands:
```bash
cd examples\peripheral\
git clone --recursive https://github.com/jorhol/nRF-Radio-802.15.4-Projects.git
cd nRF-Radio-802.15.4-Projects
```
## Testing the examples
- Compile both projects:
```
nRF5_SDK_15.3.0_59ac345\examples\peripheral\nRF-Radio-802.15.4-Projects\transmitter\pca10056\blank\ses\transmitter_pca10056.emProject
nRF5_SDK_15.3.0_59ac345\examples\peripheral\nRF-Radio-802.15.4-Projects\receiver\pca10056\blank\ses\receiver_pca10056.emProject
```
- Flash each project to a separate nRF52840 DK.
- Open the COM port of each board in a terminal software. Write a string in the terminal of **transmitter**, press enter, and see the string being output in terminal of **receiver**.
- **Transmitter** support using the native USB port of nRF52840 (nRF_USB) instead of the virtual COM port of the Segger OB debugger IC. This is useful if you want to run the example on the nRF52840 Dongle, which does not have the debug chip.
