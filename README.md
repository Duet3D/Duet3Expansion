# Duet3Expansion

Firmware for Duet 3 expansion and tool boards

Duet 3 expansion and tool board firmware is built from several Github projects. You need to use compatible branches of these projects. As at 20 May 2020, the latest source code is on these branches:

- Duet3Expansion: master
- RRFLibraries: dev
- CANlib: master
- FreeRTOS: master

**Instructions for building under Windows**

1. Download and install the gcc cross-compiler from https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads

2. Download and install a recent version of Eclipse IDE for C/C++ Developers from http://www.eclipse.org/downloads/eclipse-packages/.

3. Download and install GNU Arm Eclipse from https://sourceforge.net/projects/gnuarmeclipse/files/Build%20Tools/gnuarmeclipse-build-tools-win64-2.6-201507152002-setup.exe/download. This provides versions of make.exe, rm.exe and other tools without the 8192-character command line limitation of some other versions.

4. Modify your PATH environment variable to include the 'bin' folder of the GNU ARM Eclipse installation.

5. Run "which rm" and "which make" to make sure that rm and make will be fetched from that folder.

6. In Eclipse create a workspace.

7. Add this github project, the CANlib project, the FreeRTOS project and the RRFLibraries project to the workspace.

8. Select and build the configuration you want.
