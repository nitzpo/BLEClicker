# BLEClicker - TI CC2650 Launchpad based BLE clicker
This was my project for the Advanced Computer Systems course at Tel Aviv University, Fall Semester 2016-17.

## Project Definition
I chose the following project form the list of possible projects offered:
> Implement firmware that turns the Launchpad into a Bluetooth clicker (keyboard with just a few buttons, mainly forward and backward keys for presentations). Ideally the PC will recognize the launchpad as a Bluetooth keyboard in a way that does not require installing anything on the PC, not a driver and certainly not a program/process (because such configurations turn the project into a Windows/Linux/MacOS programming project, not an embedded/IoT project).

### Expectations
I've expected this project to be quite simple, even though requiring some technical work. This was one of the reasons I also chose to do this project on my own (in addition to schedule concerns). I thought I could implement this from scratch.
### Outcomes
I was proved to be very wrong.
My expectation was not met. Operating the BLE device was a difficult operation and required a very large amount of time to get it to work. I also realized that the USB HID definition, and the HOGP (HID-over-GAP BLE Profile) definition were also very difficult to directly implement. So I went to use some libraries that came with the Simplelink BLE stack. This was still tough, and took dozens of hours to get to a "MVP"; a basic working product.

Eventually, to my great relief and with strong persistance, I got the BLE HID device to work. Some of the hurdles I had to climb along the way to this are described in the "Problems and how I've overcame them" section of this document. I was very happy and felt satisfied, and then realized I still had to actually write some code... Which I did :)

## Architecture Overview
The application can be logically splitted into a few components and sub-components:
- BLE HID Device Profile: [HID-over-GATT Profile Specification](https://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=245141)
  * HID Service
  * Battery Service
  * Device Information Service
  * Scan Parameters Service
- HID Keyboard Device Implementation (as a service): [Device Class Definition for HID](http://www.usb.org/developers/hidpage/HID1_11.pdf)
- Clicking Functionality Implementation

### Code
My code is based on the Project Zero example and stack projects (which were easy to set up), with the actual application code replaced with that of the SimpleBLEPeripheral example, that was simpler than Project Zero.
In the code, I use the TI-implementation for the HID Device Profile and the HID Keyboard Service implementation. This gives me a new task with higher priority then the basic application task, that operates the GAP and GATT states, exporting callbacks.

In my application context I handle the set up of the GAP and GATT, including connection parameters and more, and the actual key interaction and report sending.

An auditor is invited to browse the commits on this repo to view how the code evolved over time. Take note that not all changes and bug fixes were commited separately, though I've tried to show those in commits.

### Code Management
Because of the fragility of the code and for convenience, I've set up this Git repository. Versioning actually helped me when I made changes that caused the device to stop advertising.

## Problems and how I've overcame them
TODO
### Testing
TODO
