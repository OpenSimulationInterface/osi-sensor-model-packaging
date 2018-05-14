OSI Sensor Model Packaging
==========================

[![Build Status](https://travis-ci.org/OpenSimulationInterface/osi-sensor-model-packaging.svg?branch=master)](https://travis-ci.org/OpenSimulationInterface/osi-sensor-model-packaging)

This document specifies the ways in which sensor models, logical models
and environmental effect models using the [Open Simulation Interface][]
are to be packaged for use in simulation environments using FMI 2.0.

This is version 0.3.0 Draft of this document. The version number is
to be interpreted according to the [Semantic Versioning Specification
(SemVer) 2.0.0][SemVer2.0.0].

The key words "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT",
"SHOULD", "SHOULD NOT", "RECOMMENDED",  "MAY", and "OPTIONAL" in this
document are to be interpreted as described in [RFC 2119][].

[Open Simulation Interface]: https://github.com/OpenSimulationInterface/open-simulation-interface
[SemVer2.0.0]: http://semver.org/spec/v2.0.0.html
[RFC 2119]: https://www.ietf.org/rfc/rfc2119.txt

## Kinds of Models

The current specification supports the following kinds of models, that
can be packaged as FMUs:

-   Sensor models, which consume osi::SensorView as input and generate
    osi::SensorData as output,

-   Logical models, like e.g. sensor fusion models, which consume
    osi::SensorData as input and produce osi::SensorData as output, and

-   Environmental effect models, which consume osi::SensorView as input
    and produce osi::SensorView as output.

Additionally complex models that combine various aspects of the model kinds
above are possible, however configuration and setup of such FMUs will require
manual intervention in those cases.

## FMI 2.0

The model MUST be packaged as a valid [FMI][] 2.0 FMU for
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
    <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp version="0.3.0" osi-version="3.0.0"/></Tool>
    ```

    where osi-version MUST contain the major, minor and patch
    version number of the open simulation interface specification
    that this model was compiled against.  This is to ensure that
    the importing environment can determine which OSI version to
    use prior to communicating with the FMU, which might be
    impossible in cases of major version changes.

-   The variable naming convention of the FMU MUST be `structured`.

-   The default experiment step size SHOULD be supplied and SHOULD
    indicate the actual model refresh rate (for the input side)
    in seconds, i.e. it is OK for the simulation to only call the
    FMU fmi2DoStep routine at this implied rate.  If it is not
    supplied the configuration of the model communication rate is
    determined from any input configuration data the model provides
    (see below) or has to be performed manually.

-   Besides the model parameters, inputs and outputs specified below
    the model can have additional inputs, outputs, and parameters
    (i.e. all kinds of variables as specified in the FMI 2.0 standard),
    as long as the sensor model can be run correctly with all of those
    variables left unconnected and at their default values.  The sensor
    model MUST NOT rely on other connections (beside the specified data
    connections) being made.

## Definition of binary variables

In order to support the efficient exchange of binary data, especially
binary data as provided for by the OSI ProtocolBuffer definitions, the
following convention is used to define such variables for FMI 2.0:

-   For a notional binary variable of a name given by `<prefix>`,
    which MUST be a valid structured name according to FMI 2.0,
    three actual FMU integer variables MUST be defined:

    -   `<prefix>.base.lo` (Integer)

        This is the lower (i.e. least significant) 32bit address
        of the binary data buffer to be passed into or out of the
        model, cast into a signed 32bit integer (without changing
        bit values, i.e. as by reinterpret_cast in C++).

    -   `<prefix>.base.hi` (Integer)

        This is the higher (i.e. most significant) 32bit address
        of the binary data buffer to be passed into or out of the
        model, cast into a signed 32bit integer (without changing
        bit values, i.e. as by reinterpret_cast in C++).

        Note that this variable is only used for 64bit platforms,
        for 32bit platforms it will always be 0, but MUST still be
        present (in order to support FMUs including both 32bit and
        64bit implementations).

    -   `<prefix>.size` (Integer)

        This is the size of the binary data buffer to be passed into
        or out of the model as a signed 32bit integer (restricting
        the maximum size of binary data buffers being passed around
        to < 2GB of size).

-   The three actual variables MUST have matching causality and
    variability, which will be the causality and variability of
    the notional binary variable.

-   Unless the causality and variability combination of the actual
    variables precludes this (i.e. for fixed or tunable
    calculatedParameters), the variables MUST have a start value of 0,
    indicating that no valid binary data buffer is available.

    Model FMUs MUST interpret values of 0 for either the base address
    (merged from lo and hi for 64bit) or the size to indicate that no
    valid binary data buffer is available and must handle this case
    safely.

-   The three actual variables MUST contain an annotation of the
    following form in the `Annotations` child element of their
    `ScalarVariable` element of the `modelDescription.xml`:

    ```XML
    <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp-binary-variable name="<prefix>" role="<role>" mime-type="<mime-type>"/></Tool>
    ```

    where `<prefix>` is the prefix as defined above, and `<role>` is either
    `base.lo`, `base.hi` or `size`, depending on the variable.

    This annotation marks the variable as belonging to a notional binary
    variable named `<prefix>`, with the given variable having the specified
    `<role>`, and the transported binary content being specified by the
    `mime-type` attribute, as given by `<mime-type>`, which MUST be a valid
    MIME type specification.

    In the case of OSI-specified data, the MIME type MUST be of the form
    `application/x-open-simulation-interface; type=SensorView; version=3.0.0`
    indicating that the binary content is conformant to a given OSI
    version (3.0.0 in this example), containing a message of the type
    given in the `type` parameter (`osi::SensorView` in this example).

    The version parameter given for the MIME type
    `application/x-open-simulation-interface` must concur with the version
    specified as part of the top-level `osmp:osmp` annotation, and will
    default to this value if left unspecified.

    It is an error if the mime-type specified in the annotations for one
    notional binary variable (i.e. with identical name attribute) differ,
    or if there is not exactly one variable of each role for the same
    name.

-   The FMU MUST NOT contain any variable that is named `<prefix>`: This
    restriction ensures that there is no conflict between the notional
    binary variable defined and another variable.

-   The guaranteed lifetime of the binary data buffer pointer transported
    through the actual variables is defined for each kind of variable
    specified below.

## Sensor View Inputs

-   Sensor view inputs MUST be named with the prefix `OSMPSensorViewIn`.
    If more than one sensor view input is to be configured, the prefix
    MUST be extended by an array index designator, i.e. two inputs
    will use the prefixes `OSMPSensorViewIn[1]` and `OSMPSensorViewIn[2]`.
    The indices MUST start at 1 and MUST be consecutive.  If only one
    sensor view input is needed the prefix MUST be just `OSMPSensorViewIn`.

-   Each sensor view input MUST be define as a notional discrete binary
    input variable, as specified above, with `causality="input"` and
    `variability="discrete"`.

-   The MIME type of the variable MUST specify the `type=SensorView`, e.g.
    `application/x-open-simulation-interface; type=SensorView; version=3.0.0`.

-   The sensor view MUST be encoded as osi::SensorView (see the OSI
    specification documentation for more details).

-   The guaranteed lifetime of the sensor view protocol buffer pointer
    provided as input to the FMU MUST be from the time of the call to
    `fmi2SetInteger` that provides those values until the end of the
    following `fmi2DoStep` call, i.e. the sensor model can rely on the
    provided buffer remaining valid from the moment it is passed in
    until the end of the corresponding calculation, and thus does not
    need to copy the contents in that case (zero copy input).

-   The sensor view passed to the model must contain data as specified
    by the corresponding `OSMPSensorViewInConfiguration` parameter.

## Sensor View Input Configuration

-   For each notional sensor view input variable (named with the base
    prefix `OSMPSensorViewIn`) a corresponding calculatedParameter
    (named with base prefix `OSMPSensorViewInConfigRequest`) and a
    parameter (named with base prefix `OSMPSensorViewInConfig`) CAN
    exist. If the calculatedParameter exists, then the corresponding
    parameter MUST exist.

-   If the calculatedParameter exists it MUST be named with the prefix
    `OSMPSensorViewInConfigRequest`, and MUST have a `causality` of
    `calculatedParameter` and a variability of either `fixed` or
    `tunable`.

-   If the parameter exists it MUST be named with the prefix
    `OSMPSensorViewInConfig`, and MUST have a `causality` of
    `parameter` and a variability of either `fixed` or `tunable`,
    where the variability MUST match the variability of the
    corresponding calculatedParameter.

-   The MIME type of both variables MUST specify the
    `type=SensorViewConfiguration`, e.g.
    `application/x-open-simulation-interface; type=SensorViewConfiguration; version=3.0.0`.

-   The variables values MUST be encoded as osi::SensorViewConfiguration
    (see the OSI specification documentation for more details).

-   As long as no non-zero value has been assigned to the corresponding
    `OSMPSensorViewInConfig` parameter, the calculated parameter value
    MUST be the desired sensor view configuration for the corresponding
    `OSMPSensorViewIn` variable, based on model internal requirements
    and any other parameters on which this calculated parameter depends.

    Once a non-zero value has been assigned to the corresponding
    `OSMPSensorViewInConfig` parameter, the value of this calculated
    parameter MUST be an encoded OSI protocol buffer containing the
    same data as the parameter.

-   The simulation environment SHOULD, during FMI initialization mode,
    query the `OSMPSensorViewInConfigRequest` calculatedParameter value,
    and, taking this value into account, determine a suitable and supported
    SensorView configuration. The simulation environment MUST set this
    configuration using the corresponding `OSMPSensorViewInConfig` parameter
    before exiting initialization mode.

## Sensor Data Outputs

-   Sensor data outputs MUST be named with the prefix `OSMPSensorDataOut`.
    If more than one sensor data output is to be provided, the prefix
    MUST be extended by an array index designator, i.e. two outputs
    will use the prefixes `OSMPSensorDataOut[1]` and `OSMPSensorDataOut[2]`.
    The indices MUST start at 1 and MUST be consecutive.  If only one
    sensor data output is needed the prefix MUST be just `OSMPSensorDataOut`.

-   Each sensor data output MUST be define as a notional discrete binary
    output variable, as specified above, with `causality="output"` and
    `variability="discrete"`.

-   The MIME type of the variable MUST specify the `type=SensorData`, e.g.
    `application/x-open-simulation-interface; type=SensorData; version=3.0.0`.

-   The sensor data MUST be encoded as osi::SensorData (see the OSI
    specification documentation for more details).

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
    <Tool name="net.pmsf.osmp" xmlns:osmp="http://xsd.pmsf.net/OSISensorModelPackaging"><osmp:osmp version="0.3.0" osi-version="3.0.0"/></Tool>
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

## TODOs

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
