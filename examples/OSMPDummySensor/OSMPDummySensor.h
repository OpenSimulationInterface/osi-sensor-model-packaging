/*
 * PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2017 PMSF IT Consulting Pierre R. Mai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

using namespace std;

#include "fmi2Functions.h"

/* Enable Logging to File for Debug Builds */
#ifndef NDEBUG
#ifdef _WIN32
#define PRIVATE_LOG_PATH "C:\\TEMP\\OSMPDummySensorLog.log"
#else
#define PRIVATE_LOG_PATH "/tmp/OSMPDummySensorLog.log"
#endif
#endif

/*
 * Variable Definitions
 *
 * Define FMI_*_LAST_IDX to the zero-based index of the last variable
 * of the given type (0 if no variables of the type exist).  This
 * ensures proper space allocation, initialisation and handling of
 * the given variables in the template code.  Optionally you can
 * define FMI_TYPENAME_VARNAME_IDX definitions (e.g. FMI_REAL_MYVAR_IDX)
 * to refer to individual variables inside your code, or for example
 * FMI_REAL_MYARRAY_OFFSET and FMI_REAL_MYARRAY_SIZE definitions for
 * array variables.
 */

/* Boolean Variables */
#define FMI_BOOLEAN_SOURCE_IDX 0
#define FMI_BOOLEAN_VALID_IDX 1
#define FMI_BOOLEAN_LAST_IDX FMI_BOOLEAN_VALID_IDX
#define FMI_BOOLEAN_VARS (FMI_BOOLEAN_LAST_IDX+1)

/* Integer Variables */
#define FMI_INTEGER_SENSORDATA_IN_BASELO_IDX 0
#define FMI_INTEGER_SENSORDATA_IN_BASEHI_IDX 1
#define FMI_INTEGER_SENSORDATA_IN_SIZE_IDX 2
#define FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX 3
#define FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX 4
#define FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX 5
#define FMI_INTEGER_COUNT_IDX 6
#define FMI_INTEGER_LAST_IDX FMI_INTEGER_COUNT_IDX
#define FMI_INTEGER_VARS (FMI_INTEGER_LAST_IDX+1)

/* Real Variables */
#define FMI_REAL_LAST_IDX 0
#define FMI_REAL_VARS (FMI_REAL_LAST_IDX+1)

/* String Variables */
#define FMI_STRING_LAST_IDX 0
#define FMI_STRING_VARS (FMI_STRING_LAST_IDX+1)

#include <iostream>
#include <fstream>
#include <string>
#include <cstdarg>

#undef min
#undef max
#include "osi_sensordata.pb.h"

/* FMU Class */
class COSMPDummySensor {
public:
    /* FMI2 Interface mapped to C++ */
    COSMPDummySensor(fmi2String theinstanceName, fmi2Type thefmuType, fmi2String thefmuGUID, fmi2String thefmuResourceLocation, const fmi2CallbackFunctions* thefunctions, fmi2Boolean thevisible, fmi2Boolean theloggingOn);
    ~COSMPDummySensor();
    fmi2Status SetDebugLogging(fmi2Boolean theloggingOn,size_t nCategories, const fmi2String categories[]);
    static fmi2Component Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID, fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions, fmi2Boolean visible, fmi2Boolean loggingOn);
    fmi2Status SetupExperiment(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime);
    fmi2Status EnterInitializationMode();
    fmi2Status ExitInitializationMode();
    fmi2Status DoStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component);
    fmi2Status Terminate();
    fmi2Status Reset();
    void FreeInstance();
    fmi2Status GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]);
    fmi2Status GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]);
    fmi2Status GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]);
    fmi2Status GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[]);
    fmi2Status SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]);
    fmi2Status SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]);
    fmi2Status SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]);
    fmi2Status SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]);

protected:
    /* Internal Implementation */
    fmi2Status doInit();
    fmi2Status doStart(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime);
    fmi2Status doEnterInitializationMode();
    fmi2Status doExitInitializationMode();
    fmi2Status doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component);
    fmi2Status doTerm();
    void doFree();

protected:
    /* Private File-based Logging just for Debugging */
#ifdef PRIVATE_LOG_PATH
    static ofstream private_log_file;
#endif

    static void private_log_global(const char* format, ...) {
#ifdef PRIVATE_LOG_PATH
        va_list ap;
        va_start(ap, format);
        char buffer[1024];
        if (!private_log_file.is_open())
            private_log_file.open(PRIVATE_LOG_PATH, ios::out | ios::app);
        if (private_log_file.is_open()) {
#ifdef _WIN32
            vsnprintf_s(buffer, 1024, format, ap);
#else
            vsnprintf(buffer, 1024, format, ap);
#endif
            private_log_file << "OSMPDummySensor" << "::Global: " << buffer << endl;
            private_log_file.flush();
        }
#endif
    }
    void private_log(const char* format, ...) {
#ifdef PRIVATE_LOG_PATH
        va_list ap;
        va_start(ap, format);
        char buffer[1024];
        if (!private_log_file.is_open())
            private_log_file.open(PRIVATE_LOG_PATH, ios::out | ios::app);
        if (private_log_file.is_open()) {
#ifdef _WIN32
            vsnprintf_s(buffer, 1024, format, ap);
#else
            vsnprintf(buffer, 1024, format, ap);
#endif
            private_log_file << "OSMPDummySensor" << "::" << instanceName << "<" << ((void*)this) << ">: " << buffer << endl;
            private_log_file.flush();
        }
#endif
    }

protected:
    /* Members */
    string instanceName;
    fmi2Type fmuType;
    string fmuGUID;
    string fmuResourceLocation;
    bool visible;
    bool loggingOn;
    fmi2CallbackFunctions functions;
    fmi2Boolean boolean_vars[FMI_BOOLEAN_VARS];
    fmi2Integer integer_vars[FMI_INTEGER_VARS];
    fmi2Real real_vars[FMI_REAL_VARS];
    string string_vars[FMI_STRING_VARS];
    double last_time;
    string currentBuffer;
    string lastBuffer;

    /* Simple Accessors */
    fmi2Boolean fmi_source() { return boolean_vars[FMI_BOOLEAN_SOURCE_IDX]; }
    void set_fmi_source(fmi2Boolean value) { boolean_vars[FMI_BOOLEAN_SOURCE_IDX]=value; }
    fmi2Boolean fmi_valid() { return boolean_vars[FMI_BOOLEAN_VALID_IDX]; }
    void set_fmi_valid(fmi2Boolean value) { boolean_vars[FMI_BOOLEAN_VALID_IDX]=value; }
    fmi2Integer fmi_count() { return integer_vars[FMI_INTEGER_COUNT_IDX]; }
    void set_fmi_count(fmi2Integer value) { integer_vars[FMI_INTEGER_COUNT_IDX]=value; }

    /* Protocol Buffer Accessors */
    bool get_fmi_sensor_data_in(osi::SensorData& data);
    void set_fmi_sensor_data_out(const osi::SensorData& data);
    void reset_fmi_sensor_data_out();
};
