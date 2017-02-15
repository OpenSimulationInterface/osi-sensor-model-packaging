OSI Sensor Model Packaging
==========================

[![Build Status](https://travis-ci.org/OpenSimulationInterface/osi-sensor-model-packaging.svg?branch=master)](https://travis-ci.org/OpenSimulationInterface/osi-sensor-model-packaging)

This document specifies the ways in which sensor models using the
[Open Simulation Interface][] are to be packaged for use in simulation
environments using FMI 2.0.

This is version 0.2.0 Draft of this document. The version number is
to be interpreted according to the [Semantic Versioning Specification
(SemVer) 2.0.0][SemVer2.0.0].

The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT",
"SHOULD", "SHOULD NOT", "RECOMMENDED",  "MAY", and "OPTIONAL" in this
document are to be interpreted as described in [RFC 2119][].

[Open Simulation Interface]: https://github.com/OpenSimulationInterface/open-simulation-interface
[SemVer2.0.0]: http://semver.org/spec/v2.0.0.html
[RFC 2119]: https://www.ietf.org/rfc/rfc2119.txt

## FMI 2.0

The sensor model MUST be packaged as a valid [FMI][] 2.0 FMU for
Co-Simulation, as specified in the [FMI 2.0 standard][]. Unless
otherwise noted in this document all specifications in the FMI
2.0 standard apply as-is.

[FMI]: https://www.fmi-standard.org/ 
[FMI 2.0 standard]: https://svn.modelica.org/fmi/branches/public/specifications/v2.0/FMI_for_ModelExchange_and_CoSimulation_v2.0.pdf

## Basic Conventions

The following basic conventions apply:

-   In order to mark the FMU as being conformant to this version
    of the specification the following annotation MUST be placed
    into the `VendorAnnotations` element of the `modelDescription.xml`:

    ```XML
    <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp version="0.2.0" osi-version="2.0.0"/></Tool>
    ```

    where osi-version MUST contain the major, minor and patch
    version number of the open simulation interface specification
    that this model was compiled against.  This is to ensure that
    the importing environment can determine which OSI version to
    use prior to communicating with the FMU, which might be
    impossible in cases of major version changes.

-   The variable naming convention of the FMU MUST be `structured`.

-   The default experiment step size SHOULD be supplied and SHOULD 
    indicate the actual sensor refresh rate (for the input side)
    in seconds, i.e. it is OK for the simulation to only call the
    sensor step routine at this implied rate.  If it is not supplied
    the configuration of the sensor model communication rate has to
    be performed manually.

-   Besides the sensor data inputs and outputs specified below
    the sensor model can have additional inputs, outputs, and
    parameters (i.e. all kinds of variables as specified in the
    FMI 2.0 standard), as long as the sensor model can be run
    correctly with all of those variables left unconnected and
    at their default values.  The sensor model MUST NOT rely on
    other connections (beside the sensor data connections) being
    made.

## Sensor Data Inputs

-   Sensor data inputs MUST be named with the prefix `OSMPSensorDataIn`.
    If more than one sensor data input is to be configured, the prefix
    MUST be extended by an array index designator, i.e. two inputs
    will use the prefixes `OSMPSensorDataIn[1]` and `OSMPSensorDataIn[2]`.
    The indices MUST start at 1 and MUST be consecutive.  If only one
    sensor data input is needed the prefix MUST be just `OSMPSensorDataIn`.

-   For each input three discrete input variables MUST be defined
    using the prefix:

    -   `<prefix>.base.lo` (Discrete Integer Input)
      
        This is the lower (i.e. least significant) 32bit address
        of the OSI sensor data protocol buffer to be passed into
        the sensor model, cast into a signed 32bit integer (without
        changing bit values, i.e. as by reinterpret_cast in C++).

    -   `<prefix>.base.hi` (Discrete Integer Input)

        This is the higher (i.e. most significant) 32bit address
        of the OSI sensor data protocol buffer to be passed into 
        the sensor model, cast into a signed 32bit integer (without
        changing bit values, i.e. as by reinterpret_cast in C++).
      
        Note that this variable is only used for 64bit platforms,
        for 32bit platforms it will always be 0, but MUST still be
        present (in order to support FMUs including both 32bit and
        64bit implementations).

    -   `<prefix>.size` (Discrete Integer Input)

        This is the size of the OSI sensor data protocol buffer to 
        be passed into the sensor model as a signed 32bit integer
        (thus restricting the maximum size of OSI protocol buffers
        being passed around to < 2GB of size).
  
-   All sensor data input variables MUST be declared with
    `causality="input"` and `variability="discrete"`.
    
-   All sensor data input variables MUST have a default value of 0,
    indicating no valid sensor data protocol buffer available.

    Sensor model FMUs MUST interpret values of 0 for either the base
    address (merged from lo and hi for 64bit) or the size to indicate
    no valid sensor data protocol buffer is available and must handle
    this case safely.

    Sensor model FMUs MUST interpret values of 0 for either the base
    address (merged from lo and hi for 64bit) or the size to indicate
    no valid sensor data protocol buffer available and must handle this
    case safely.

-   The guaranteed lifetime of the sensor data protocol buffer pointer
    provided as input to the FMU MUST be from the time of the call to 
    `fmi2SetInteger` that provides those values until the end of the
    following `fmi2DoStep` call, i.e. the sensor model can rely on the
    provided buffer remaining valid from the moment it is passed in
    until the end of the corresponding calculation, and thus does not
    need to copy the contents in that case (zero copy input).

-   The sensor data MUST be encoded as osi::SensorData (see the OSI
    specification documentation for more details).

## Sensor Data Output

-   Sensor data output MUST be named with the prefix `OSMPSensorDataOut`.
    Currently no scenario is envisaged where more than one sensor
    data output is needed, however if such a need ever arises the
    same rules as for multiple sensor data inputs are going to
    apply, hence prefixes of the form `OSMPSensorDataOut[1]`,
    `OSMPSensorDataOut[2]`, etc., are reserved for the time being.

-   For each output three discrete output variables MUST be defined
    using the prefix:

    -   `<prefix>.base.lo` (Discrete Integer Output)
      
        This is the lower (i.e. least significant) 32bit address
        of the OSI sensor data protocol buffer to be passed out of
        the sensor model, cast into a signed 32bit integer (without
        changing bit values, i.e. as by reinterpret_cast in C++).

    -   `<prefix>.base.hi` (Discrete Integer Output)

        This is the higher (i.e. most significant) 32bit address
        of the OSI sensor data protocol buffer to be passed out of
        the sensor model, cast into a signed 32bit integer (without
        changing bit values, i.e. as by reinterpret_cast in C++).
      
        Note that this variable is only used for 64bit platforms,
        for 32bit platforms it will always be 0, but must still be
        present (in order to support FMUs including both 32bit and
        64bit implementations).

    -   `<prefix>.size` (Discrete Integer Output)

        This is the size of the OSI sensor data protocol buffer to
        be passed out of the sensor model as a signed 32bit integer
        (thus restricting the maximum size of OSI protocol buffers
        being passed around to < 2GB of size).
  
-   All sensor data output variables MUST be declared with
    `causality="output"` and `variability="discrete"`.
    
-   All sensor data output variables MUST have a default value of 0,
    indicating no valid sensor data protocol buffer available.

    Simulations MUST interpret values of 0 for either the base address
    (merged from lo and hi for 64bit) or the size to indicate no valid
    sensor data protocol buffer is available and must handle this case
    safely.

-   The guaranteed lifetime of the sensor data protocol buffer pointer
    provided as output by the FMU MUST be from the end of the call to
    `fmi2DoStep` that calculated this buffer until the beginning of the
    **second** `fmi2DoStep` call after that, i.e. the simulation engine
    can rely on the provided buffer remaining valid from the moment it
    is passed out until the end of the next Co-Simulation calculation
    cycle, and thus does not need to copy the contents in that case
    (zero copy output for the simulation engine, at the cost of double
    buffering for the sensor model).

    This arrangement (and hence the need for double buffering) is
    required to support use of the sensor model FMUs in simulation
    engines that have no special support for the protocol buffer
    pointers, i.e. using this convention it is possible to daisy
    chain FMUs with protocol buffer inputs/outputs in a normal
    simulation engine like e.g. MATLAB/Simulink, and get valid
    results.

-   The sensor data MUST be encoded as osi::SensorData (see the OSI
    specification documentation for more details).

## Examples

An example dummy sensor model implementation is provided in the
OSMPDummySensor sub-directory of the examples directory of this
repository.  Below you can find an example modelDescription.xml
file that would satisfy the requirements of this document for a sensor
model FMU with one input and output and no additional features:

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
    <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp version="0.1"/></Tool>
  </VendorAnnotations>
  <ModelVariables>
    <ScalarVariable name="OSMPSensorDataIn.base.lo" valueReference="0" causality="input" variability="discrete">
      <Integer start="0"/>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorDataIn.base.hi" valueReference="1" causality="input" variability="discrete">
      <Integer start="0"/>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorDataIn.size" valueReference="2" causality="input" variability="discrete">
      <Integer start="0"/>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorDataOut.base.lo" valueReference="3" causality="output" variability="discrete" initial="exact">
      <Integer start="0"/>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorDataOut.base.hi" valueReference="4" causality="output" variability="discrete" initial="exact">
      <Integer start="0"/>
    </ScalarVariable>
    <ScalarVariable name="OSMPSensorDataOut.size" valueReference="5" causality="output" variability="discrete" initial="exact">
      <Integer start="0"/>
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

## TODOs

-   Define auto configuration mechanism, so that sensor models can
    automatically provide relevant parameters for the environment
    simulation (e.g. position and field of view of the sensor, etc.)
    to the simulation, e.g. via computed parameters that provide a
    protocol buffer configuration record, or through direct FMI 
    constants.

-   Support various raw data streams, like e.g. video streams, or raw
    reflection lists, in the same vein as the current object list
    sensor data.  This is likely to be achieved using the same
    basic mechanism, but including a string constant per input/output
    indicating the MIME type of the transported data (or some other
    naming scheme for the type), so that the variables can be
    identified properly.  Can be merged with the above mechanism for
    auto configuration, if employed.

## Future Evolution

In the future an extension to the FMI standard that directly
supported opaque binary data (e.g. a binary data type that
is defined in the same way as the current string data type,
but length terminated instead of zero-terminated) would allow
migration of sensor models using the current convention to
one where the sensor data input and output variables could be
mapped into just one such input and output variable each, of
the relevant binary type.  The object lifetimes might need to
be adjusted in such a case to match the FMI standard extension,
depending on the form that is going to take.
