# OSI Sensor Model Packaging

[![Build Status](https://travis-ci.com/OpenSimulationInterface/osi-sensor-model-packaging.svg?branch=master)](https://travis-ci.com/OpenSimulationInterface/osi-sensor-model-packaging)

OSI Sensor Model Packaging specifies ways in which models (like e.g. environmental effect models, sensor models and logical models) using the [Open Simulation Interface (OSI)][] are to be packaged for their use in simulation environments using FMI 2.0.
The specification can be found in the [doc/specification.rst](doc/specification.rst) document in this repository.

For more detailed information see the [official documentation](https://opensimulationinterface.github.io/osi-documentation/osi-sensor-model-packaging/README.html).

[Open Simulation Interface (OSI)]: https://github.com/OpenSimulationInterface/open-simulation-interface

## Usage
The examples in the directory [`examples`](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples) of this repository can be built using CMake. They require that the open-simulation-interface submodule of the repository is populated.

The [`OSMPDummySource`](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples/OSMPDummySource) example can be used as a simplistic source of SensorView (including GroundTruth) data, that can be connected to the input of an OSMPDummySensor model, for simple testing and demonstration purposes.

The [`OSMPCNetworkProxy`](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples/OSMPCNetworkProxy) example demonstrates a simple C network proxy that can send and receive OSI data via TCP sockets.

The [`OSMPDummySensor`](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples/OSMPDummySensor) example can be used as a simple dummy sensor model, demonstrating the use of OSI for sensor models consuming SensorView data and generating SensorData output.

## Installation
##### Dependencies

Install `cmake` 3.10.2:
```bash
$ sudo apt-get install cmake
```
Install `protobuf` 3.0.0:
```bash
$ sudo apt-get install libprotobuf-dev protobuf-compiler
```

##### Build and install example
```bash
$ git clone https://github.com/OpenSimulationInterface/osi-sensor-model-packaging.git
$ cd osi-sensor-model-packaging
$ git submodule update --init
$ cd examples
$ mkdir -p build
$ cd build
$ cmake ..
$ make
$ sudo make install
```
