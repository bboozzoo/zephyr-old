Title: Disco demo

Description:

A simple 'disco' demo. The demo assumes that 2 LEDs are connected to
GPIO outputs of the MCU/board. The sample looks up a predefined GPIO
device (GPIOB), assumes that the devices are connected to pins 3 and
5. The pins get configured into output mode. Once the main loop is
entered, on each iteration the state of GPOI lines will be changed so
that one of the line is in high state, while the other is in low.

--------------------------------------------------------------------------------

Building and Running Project:

This microkernel project outputs to the console.  It can be built and
executed on QEMU as follows:

    make BOARD=<name-of-your-board>

--------------------------------------------------------------------------------

Troubleshooting:

Problems caused by out-dated project information can be addressed by
issuing one of the following commands then rebuilding the project:

    make clean          # discard results of previous builds
                        # but keep existing configuration info
or
    make pristine       # discard results of previous builds
                        # and restore pre-defined configuration info

--------------------------------------------------------------------------------
