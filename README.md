AM32 Bootloader
---------------

This is the bootloader for the AM32 project

Installing Build Tools
----------------------

To install the required build tools please run the script for your
operating system found in the env_setup_scripts directory. This will
download the required tools from
https://firmware.ardupilot.org/Tools/AM32-tools/ and unpack them in
tools. It will also setup your vscode settings for your OS.

Runing VSCode
-------------

When you run vscode it will recommend you install some key extensions:

 - C/C++ tools
 - Cortex-Debug
 - Makefile Tools

You will need to install these before doing a build.

Command Line Build
------------------

To build with the command line use the command "make". If your
environment is setup correctly you should be able to tab complete the
available targets. Otherwise you can run "make targets" to see the
available build targets.

Bootloader Updaters
-------------------

A bootloader updater is a small firmware that carries a bootloader
image inside it. It is flashed as if it were the main application, and
on boot it writes the bootloader it carries into place and resets. This
lets a bootloader be replaced without a SWD debugger. Run "make
updater_targets" to see the available updaters, or "make updaters" to
build them all.

An updater named AM32_<MCU>_BL_UPDATER_<PIN>_CAN_FROM4K_V<n> is a
transition updater. Use it to move an ESC that shipped with a 4k
non-CAN bootloader onto a 16k DroneCAN bootloader, which the normal CAN
updater cannot do as it needs the 16k bootloader to already be
installed. The transition updater has its entry point at 0x08001000
where the 4k bootloader starts the application, while its code and the
bootloader image it carries sit above 0x08004000, out of the way of the
region it erases. A CAN bootloader keeps its settings at the 128k layout
address rather than where the 4k bootloader kept them, and the
transition updater leaves that page blank.

The main firmware is erased in the process, so flash a matching CAN main
firmware once the new bootloader is in place, then configure the ESC
from scratch with the configurator or the DroneCAN GUI tool. Until it is
configured the new bootloader will not boot the application.

CI Builds
---------

All of the bootloaders are automatically built in CI using github
actions. See the Actions tab on
https://github.com/am32-firmware/AM32-bootloader for the latest
builds.

Releases
--------

The latest release is available here:

https://github.com/am32-firmware/AM32-bootloader/releases

Getting Help
------------

If you need help with bootloader development please ask on the AM32
discord server in the development channel
https://discord.com/invite/h7ddYMmEVV
