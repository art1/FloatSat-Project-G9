



# CanSat / FloatSat Project at University of Wuerzburg


This repository contains a configured workspace for Eclipse Mars.1 Release (4.5.1) with the most recent project state

Tested on OSX10.11.


# How to use this repository
As said before, this repository contains everything to compile RODOS for a STM32F4-Discovery Board. To do so, import the projects in this workspace in your
eclipse-installation. File -> Import -> General -> Existing Project into Workspace -> Next -> Select repository directory as root directory.
Then just select all Projects and click Finish.

To compile for ARM an appropriate toolchain is necessary, download the 4.8 2014q3 at https://launchpad.net/gcc-arm-embedded/+download for your OS.


To build the Dalek OBC, you have to build rodos first, then the support libs, then the STM32F4 Template, and only after all those have been compiled without the project can be build.
