# BLEClicker - TI CC2650 Launchpad based BLE clicker
This was my project for the Advanced Computer Systems course at Tel Aviv University, Fall Semester 2016-17.

## Project Definition
I chose the following project from the list of possible projects offered:
> Implement firmware that turns the Launchpad into a Bluetooth clicker (keyboard with just a few buttons, mainly forward and backward keys for presentations). Ideally the PC will recognize the launchpad as a Bluetooth keyboard in a way that does not require installing anything on the PC, not a driver and certainly not a program/process (because such configurations turn the project into a Windows/Linux/MacOS programming project, not an embedded/IoT project).

### Expectations
I've expected this project to be somewhat simple, while requiring extensive technical work. This was one of the reasons I also chose to do this project on my own (in addition to schedule concerns). I thought I could implement this from scratch.

### Outcomes
I was proved to be very wrong.
My expectation was not met. Operating the BLE device was difficult and required a very large amount of time to get it to work. I also realized that the USB HID definition, and the HOGP (HID-over-GAP BLE Profile) definition were also severely out of the scope I planned in order to directly implement. So I went to use some libraries that came with the Simplelink BLE stack. This was still tough, and took dozens of hours to get to a "MVP"; a basic working product.

Eventually, to my great relief, and with strong persistance, I got the BLE HID device to work. Some of the hurdles I had to climb along the way to this are described in the "Problems and how I've overcome them" section of this document. I was very happy and felt satisfied, and then realized I still had to actually write some code... Which I did :)

I've turned the CC2650-Launchpad into a HID keyboard. The Launchpad has two buttons. I configured the first as a spacebar click, and the second as a backspace click. Space goes forward in a slideshow and backspace goes backwards. Thus we get a slideshow clicker.

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
My code is based on the Project Zero example and stack projects (which were easy to set up), with the actual base application code replaced with that of the SimpleBLEPeripheral example, that was simpler than Project Zero.
In the code, I use the TI implementation for the HID Device Profile and the HID Keyboard Service implementation. This gives me a new task with higher priority then the basic application task, that operates the GAP and GATT states, exporting callbacks.

In my application context I handle the set up of the GAP and GATT, including connection parameters and more, and the actual key interaction and report sending.

An auditor is invited to browse the commits on this repo to view how the code evolved over time. Take note that not all changes and bug fixes were commited separately, though I've tried to show those in commits. The most interesting code is in Application\clicker.c.

### Code Management
Because of the fragility of the code and for convenience, I've set up this Git repository. Versioning actually helped me when I made changes that caused the device to stop advertising.

## Problems and how I've overcome them
As previously noted, I had ALOT of problems throughout the project. I will try to list some of the major ones here for reference.

### Testing
Firstly I wanted to get the UART output to work. This was not trivial as one specifically has to setup UartLog for the device, and the Windows device has to be configured to the UART settings *in addition* to the terminal software (for my case, Putty).

Debugging was also used to test specific code parts, but since there are only 4 breakpoints it was not very useful for this case. In a different section I discuss debugging issues.

I don't have a BLE sniffer device so I used my phone (LG G5) with a BLE Scanner app. This (usually) worked and allowed me to see how the services are functioning.

Probably the **worst** problem I had in this project was that for a whole day of working on the project, I couldn't see the advertisements from the device. I thought this was something I'd changed so I started reverting things (Git helped) but no results. At some point I uploaded Project Zero to test if the board was working. And still nothing! I was nearly sure the board was dead. After hours of trying, my phone battery died and I replaced it with a spare. Then it worked! Apparently there is a bug in LG's BT!

Later on, after addressing multiple other problems, I could test that everything worked on my phone - without requiring an actual PC dongle; this was due to the fact that I had set up spacebar and backspace buttons. My phone allowed me to connect the launchpad as a HID device, and in some text box I could then type spaces and delete them using the launchpad! So exciting!

### Debugging
An issue I had with debugging:
I could place breakpoints in simple code, code from main.c and inside callbacks. But if I set breakpoints inside the profile code - the debugger didn't stop on them. I saw that the symbols are not even generated for the functions I want to break on (from the disassembly view). So I:
- Searched in the TI forums and Google etc.
- Asked in the course's Facebook group. They offered setting some compiler defines and changing compiler optimizations.
- After searching some more using this information, I found some hints online, and realized what was the problem.
I solved it by disabling compiler optimizations. The situation was that I wanted to break inside *static functions* that were only used once in the code and therefore inlined by the compiler. No optimizations means - no inlining.

I have documented this issue in a [stackoverflow question](http://stackoverflow.com/questions/42657395/ti-simplelink-ble-debugging-cant-place-breakpoints-in-most-of-code).

### Horrible Memory Errors and Predefined Symbols
- ICALL_MAX_NUM_TASKS had to be increased to 4. This defines how many tasks will be registered with ICall. My tasks are:
 1. BLE Stack Task
 2. GAPRole Task
 3. Application Task (the first 3 are also in Project Zero)
 4. *HID Device Task*
 
 The symptom for this was an abort after a few seconds from startup.
 
- ICALL_MAX_NUM_ENTITIES had to be increased to 8. The ICall entities contain both tasks and services. As previously described, the HID device profile requires many services to be instantiated.

- OSAL_MAX_NUM_PROXY_TASKS (a BLE stack symbol! changing this means you have to recompile the stack) had to be set to 3 instead of 2. This define controls the number of tasks that are allowed to communicate with the BLE protocol stack through ICALL. The symptom for this problem was no advertising. I used the debugger to pause the device and inspected the ROV task view. I saw all tasks pending on some semaphores, and the stack task *running* on ICall_taskEntry.
 - I saw random stack overruns for the idle stack. So I increased its stack size (later decreased because it was unnecessary).
 - I found a [post on the TI forums](https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/462149), addressing a situation of ICall_taskEntry stuck running. But it was of no help.
 - Eventually I found a [guide for adding an RTOS task](http://processors.wiki.ti.com/index.php/Adding_Custom_RTOS_Task). Unfortunately, this was after I've already encountered and addressed the other issues. This guide explained that this predefined symbol had to be edited.
 
### HID
Finally got the device to work, but a few seconds after a connection is established I get disconnected and the device is not advertising. This was frustrating.
- I went over and debugged the HID device code, and realized that there was a HID_DEV_IDLE_TIMEOUT define set to 6000 (6s).
- The device automatically disconnects after the specified idle time to save battery.
- Another define, AUTO_ADV, is disabled by default. Setting it up meant that the HID device will restart advertising after loosing the connection. Since at this point I didn't write the report sending code yet, I increased the timeout and set AUTO_ADV=TRUE. That way I could use the device, and if the connection was lost I could reconnect.
- If the device wants to send a report, it automatically reconnects. So I understood this was the correct way to go, and disabled AUTO_ADV, and lowered the timeout.

The USB [Device Class Definition for HID](http://www.usb.org/developers/hidpage/HID1_11.pdf) is not a very nice read (gently speaking). I found there the report data structure to send through the HID device, in the Device Class Definition for HID, Appendix B, section 1. [This guide from MBED documentation](https://docs.mbed.com/docs/ble-hid/en/latest/api/md_doc_HID.html) helped too.

### Connection Settings
After getting everything to work (oh the relief..), there was an annoying delay between clicks. This was caused becuase of the default BLE connection paramters set for Project Zero. Those did not fit well for the Clicker application. The changes can be viewed in [the commit](https://github.com/nitzpo/BLEClicker/commit/f478bee60f3078e5f2ac08ca178c05275e0dcd72).

## Pictures
![screenshot_2017-03-07-21-18-50](https://cloud.githubusercontent.com/assets/9297302/23674215/caf6bb32-037d-11e7-84aa-1e24a19f5160.png)
![screenshot_2017-03-07-21-18-40](https://cloud.githubusercontent.com/assets/9297302/23674219/caf756d2-037d-11e7-96f5-6f2bbc628ff6.png)
![screenshot_2017-03-07-21-19-39](https://cloud.githubusercontent.com/assets/9297302/23674218/caf762d0-037d-11e7-9d59-8fd3ea9dd3ef.png)
![20170307_212014](https://cloud.githubusercontent.com/assets/9297302/23674217/caf763fc-037d-11e7-95db-9f60193a3670.jpg)
