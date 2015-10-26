# DALEK On-Board Computer

This repository contains a configured workspace for Eclipse Mars.1 Release (4.5.1) with a working example of RODOS and a simple STM32F4-Example.
That's the starting Point for our CanSat Project "Dalek" - where we inted to build a "Dalek"-Satellite :P
Tested on OSX10.11.

# How to use this repository
As said before, this repository contains everything to compile RODOS for a STM32F4-Discovery Board. To do so, import the projects in this workspace in your
eclipse-installation. File -> Import -> General -> Existing Project into Workspace -> Next -> Select repository directory as root directory.
Then just select all Projects and click Finish.

To compile for ARM an appropriate toolchain is necessary, download the 4.8 2014q3 at https://launchpad.net/gcc-arm-embedded/+download for your OS.


To build the Dalek OBC, you have to build rodos first, then the support libs, then the STM32F4 Template, and only after all those have been compiled without errors (ignore the warnings :P)
you can build the DalekOBC 
