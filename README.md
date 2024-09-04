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
