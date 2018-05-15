OSI Sensor Model Packaging Examples
===================================

The examples in this directory can be built using CMake.
They require that the open-simulation-interface submodule
of the repository is populated.

The OSMPDummySensor example can be used as a simple dummy sensor
model, demonstrating the use of OSI for sensor models consuming
SensorView data and generating SensorData output.

The OSMPDummySource example can be used as a simplistic source of
SensorView (including GroundTruth) data, that can be connected to
the input of an OSMPDummySensor model, for simple testing and
demonstration purposes.

The OSMPCNetworkProxy example demonstrates a simple C network proxy
that can send and receive OSI data via TCP sockets.
