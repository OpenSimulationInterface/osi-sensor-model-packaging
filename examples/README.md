OSI Sensor Model Packaging Examples
===================================

The examples in this directory can be built using CMake.
They require that the open-simulation-interface submodule
of the repository is populated.

The OSMPDummySensor example can be used both as a simple dummy
sensor model, demonstrating the use of OSI for sensor models,
and, if the source parameter is set to true, as a simplistic
source of GroundTruth data, that can be connected to the input
of another OSMPDummySensor model, for simple testing and 
demonstration purposes.

The OSMPDummySensor10 example shows how the OSMP definitions
could be mapped to FMI 1.0, however this is NOT part of the
specified standard, i.e. OSMP-compliant FMUs MUST be compliant
to FMI 2.0 or later.

The OSMPNetworkProxy example can be used to send its OSI input
to a TCP socket or a Zer0MQ socket (either pair or publish socket).
It is implemented in C++.
 
The OSMPCNetworkProxy example demonstrates a simple C network proxy
that can send and receive OSI data via TCP sockets.
