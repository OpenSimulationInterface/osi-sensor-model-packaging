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

extern "C" {
#define MODEL_IDENTIFIER OSMPDummySensor10
#include "fmiFunctions.h"
}
/* Enable Logging to File for Debug Builds */
#ifndef NDEBUG
#ifdef _WIN32
#define PRIVATE_LOG_PATH "C:\\TEMP\\OSMPDummySensor10Log.log"
#else
#define PRIVATE_LOG_PATH "/tmp/OSMPDummySensorLog10.log"
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
class COSMPDummySensor10 {
public:
    /* FMI Interface mapped to C++ */
    COSMPDummySensor10(fmiString theinstanceName, fmiString thefmuGUID, fmiString thefmuLocation, fmiString themimeType, fmiReal thetimeout, fmiBoolean thevisible, fmiBoolean theinteractive, fmiCallbackFunctions thefunctions, fmiBoolean theloggingOn);
    ~COSMPDummySensor10();
    fmiStatus SetDebugLogging(fmiBoolean theloggingOn);
    static fmiComponent Instantiate(fmiString theinstanceName, fmiString thefmuGUID, fmiString thefmuLocation, fmiString themimeType, fmiReal thetimeout, fmiBoolean thevisible, fmiBoolean theinteractive, fmiCallbackFunctions thefunctions, fmiBoolean theloggingOn);
    fmiStatus InitializeSlave(fmiReal startTime, fmiBoolean stopTimeDefined, fmiReal stopTime);
    fmiStatus DoStep(fmiReal currentCommunicationPoint, fmiReal communicationStepSize, fmiBoolean newStep);
    fmiStatus Terminate();
    fmiStatus Reset();
    void FreeInstance();
    fmiStatus GetReal(const fmiValueReference vr[], size_t nvr, fmiReal value[]);
    fmiStatus GetInteger(const fmiValueReference vr[], size_t nvr, fmiInteger value[]);
    fmiStatus GetBoolean(const fmiValueReference vr[], size_t nvr, fmiBoolean value[]);
    fmiStatus GetString(const fmiValueReference vr[], size_t nvr, fmiString value[]);
    fmiStatus SetReal(const fmiValueReference vr[], size_t nvr, const fmiReal value[]);
    fmiStatus SetInteger(const fmiValueReference vr[], size_t nvr, const fmiInteger value[]);
    fmiStatus SetBoolean(const fmiValueReference vr[], size_t nvr, const fmiBoolean value[]);
    fmiStatus SetString(const fmiValueReference vr[], size_t nvr, const fmiString value[]);

protected:
    /* Internal Implementation */
    fmiStatus doInit();
    fmiStatus doStart(fmiReal startTime, fmiBoolean stopTimeDefined, fmiReal stopTime);
    fmiStatus doCalc(fmiReal currentCommunicationPoint, fmiReal communicationStepSize, fmiBoolean newStep);
    fmiStatus doTerm();
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
            private_log_file << "OSMPDummySensor10" << "::Global: " << buffer << endl;
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
            private_log_file << "OSMPDummySensor10" << "::" << instanceName << "<" << ((void*)this) << ">: " << buffer << endl;
            private_log_file.flush();
        }
#endif
    }

protected:
    /* Members */
    string instanceName;
    string fmuGUID;
	string mimeType;
    string fmuLocation;
	double timeout;
    bool visible;
	bool interactive;
    bool loggingOn;
    fmiCallbackFunctions functions;
    fmiBoolean boolean_vars[FMI_BOOLEAN_VARS];
    fmiInteger integer_vars[FMI_INTEGER_VARS];
    fmiReal real_vars[FMI_REAL_VARS];
    string string_vars[FMI_STRING_VARS];
    double last_time;
    string currentBuffer;
    string lastBuffer;

    /* Simple Accessors */
    fmiBoolean fmi_source() { return boolean_vars[FMI_BOOLEAN_SOURCE_IDX]; }
    void set_fmi_source(fmiBoolean value) { boolean_vars[FMI_BOOLEAN_SOURCE_IDX]=value; }
    fmiBoolean fmi_valid() { return boolean_vars[FMI_BOOLEAN_VALID_IDX]; }
    void set_fmi_valid(fmiBoolean value) { boolean_vars[FMI_BOOLEAN_VALID_IDX]=value; }
    fmiInteger fmi_count() { return integer_vars[FMI_INTEGER_COUNT_IDX]; }
    void set_fmi_count(fmiInteger value) { integer_vars[FMI_INTEGER_COUNT_IDX]=value; }

    /* Protocol Buffer Accessors */
    bool get_fmi_sensor_data_in(osi::SensorData& data);
    void set_fmi_sensor_data_out(const osi::SensorData& data);
    void reset_fmi_sensor_data_out();
};
