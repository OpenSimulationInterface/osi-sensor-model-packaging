# OSI Sensor Model Packaging

[![Build Status](https://travis-ci.org/OpenSimulationInterface/osi-sensor-model-packaging.svg?branch=master)](https://travis-ci.org/OpenSimulationInterface/osi-sensor-model-packaging)

OSI Sensor Model Packaging specifies ways in which environmental effect models, sensor models and logical models using the [Open Simulation Interface (OSI)][] are to be packaged for their use in simulation environments using FMI 2.0. For more detailed information see the [official documentation](https://opensimulationinterface.github.io/osi-documentation/osi-sensor-model-packaging/README.html).

[Open Simulation Interface (OSI)]: https://github.com/OpenSimulationInterface/open-simulation-interface

## Usage
The examples in the directory [`examples`](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples) of this repository can be built using CMake. They require that the open-simulation-interface submodule of the repository is populated.

The [`OSMPDummySource`](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples/OSMPDummySource) example can be used as a simplistic source of SensorView (including GroundTruth) data, that can be connected to the input of an OSMPDummySensor model, for simple testing and demonstration purposes.

The [`OSMPCNetworkProxy`](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples/OSMPCNetworkProxy) example demonstrates a simple C network proxy that can send and receive OSI data via TCP sockets.

The [`OSMPDummySensor`](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples/OSMPDummySensor) example can be used as a simple dummy sensor model, demonstrating the use of OSI for sensor models consuming SensorView data and generating SensorData output. Below you can find an example `modelDescription.xml` file that would satisfy the requirements of this document for a sensor model FMU with one input and output and no additional features:

```XML
<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription
  fmiVersion="2.0"
  modelName="OSI Sensor Model Packaging Demo FMU"
  guid="aabc2174e20f08597cfae6947c96bf86"
  variableNamingConvention="structured">
  <CoSimulation
    modelIdentifier="OSMPDemoFMU"
    canNotUseMemoryManagementFunctions="true"/>
  <DefaultExperiment startTime="0.0" stepSize="0.020"/>
  <VendorAnnotations>
    <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp version="1.0.0" osi-version="3.0.0"/></Tool>
  </VendorAnnotations>
  <ModelVariables>
    <ScalarVariable name="OSMPSensorViewIn.base.lo" valueReference="0" causality="input" variability="discrete">
      <Integer start="0"/>
      <Annotations>
        <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp-binary-variable name="OSMPSensorViewIn" role="base.lo" mime-type="application/x-open-simulation-interface; type=SensorView; version=3.0.0"/></Tool>
      </Annotations>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorViewIn.base.hi" valueReference="1" causality="input" variability="discrete">
      <Integer start="0"/>
      <Annotations>
        <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp-binary-variable name="OSMPSensorViewIn" role="base.hi" mime-type="application/x-open-simulation-interface; type=SensorView; version=3.0.0"/></Tool>
      </Annotations>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorViewIn.size" valueReference="2" causality="input" variability="discrete">
      <Integer start="0"/>
      <Annotations>
        <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp-binary-variable name="OSMPSensorViewIn" role="size" mime-type="application/x-open-simulation-interface; type=SensorView; version=3.0.0"/></Tool>
      </Annotations>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorDataOut.base.lo" valueReference="3" causality="output" variability="discrete" initial="exact">
      <Integer start="0"/>
      <Annotations>
        <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp-binary-variable name="OSMPSensorDataOut" role="base.lo" mime-type="application/x-open-simulation-interface; type=SensorData; version=3.0.0"/></Tool>
      </Annotations>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorDataOut.base.hi" valueReference="4" causality="output" variability="discrete" initial="exact">
      <Integer start="0"/>
      <Annotations>
        <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp-binary-variable name="OSMPSensorDataOut" role="base.hi" mime-type="application/x-open-simulation-interface; type=SensorData; version=3.0.0"/></Tool>
      </Annotations>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorDataOut.size" valueReference="5" causality="output" variability="discrete" initial="exact">
      <Integer start="0"/>
      <Annotations>
        <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp-binary-variable name="OSMPSensorDataOut" role="size" mime-type="application/x-open-simulation-interface; type=SensorData; version=3.0.0"/></Tool>
      </Annotations>
    </ScalarVariable>
  </ModelVariables>
  <ModelStructure>
    <Outputs>
      <Unknown index="4"/>
      <Unknown index="5"/>
      <Unknown index="6"/>
    </Outputs>
  </ModelStructure>
</fmiModelDescription>
```

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
$ cd osi-sensor-model-packaging/examples
$ git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git
$ mkdir -p build
$ cd build
$ cmake ..
$ make
$ sudo make install
```