/*
 * PMSF FMU Framework for FMI 1.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2017 PMSF IT Consulting Pierre R. Mai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "OSMPDummySensor10.h"

/*
 * Debug Breaks
 *
 * If you define DEBUGBREAKS the DLL will automatically break
 * into an attached Debugger on all major computation functions.
 * Note that the DLL is likely to break all environments if no
 * Debugger is actually attached when the breaks are triggered.
 */
#ifndef NDEBUG
#ifdef DEBUGBREAKS
#include <intrin.h>
#define DEBUGBREAK() __debugbreak()
#else
#define DEBUGBREAK()
#endif
#else
#define DEBUGBREAK()
#endif

#include <iostream>
#include <string>
#include <algorithm>
#include <cstdint>

using namespace std;

#ifdef PRIVATE_LOG_PATH
ofstream COSMPDummySensor10::private_log_file;
#endif

/*
 * ProtocolBuffer Accessors
 */

void* decode_integer_to_pointer(fmiInteger hi,fmiInteger lo)
{
#if PTRDIFF_MAX == INT64_MAX
    union addrconv {
        struct {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr;
    myaddr.base.lo=lo;
    myaddr.base.hi=hi;
    return reinterpret_cast<void*>(myaddr.address);
#elif PTRDIFF_MAX == INT32_MAX
    return reinterpret_cast<void*>(lo);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

void encode_pointer_to_integer(const void* ptr,fmiInteger& hi,fmiInteger& lo)
{
#if PTRDIFF_MAX == INT64_MAX
    union addrconv {
        struct {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr;
    myaddr.address=reinterpret_cast<unsigned long long>(ptr);
    hi=myaddr.base.hi;
    lo=myaddr.base.lo;
#elif PTRDIFF_MAX == INT32_MAX
    hi=0;
    lo=reinterpret_cast<int>(ptr);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

bool COSMPDummySensor10::get_fmi_sensor_data_in(osi::SensorData& data)
{
    if (integer_vars[FMI_INTEGER_SENSORDATA_IN_SIZE_IDX] > 0) {
        void* buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORDATA_IN_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_IN_BASELO_IDX]);
        private_log("Got %08X %08X, reading from %p ...",integer_vars[FMI_INTEGER_SENSORDATA_IN_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_IN_BASELO_IDX],buffer);
        data.ParseFromArray(buffer,integer_vars[FMI_INTEGER_SENSORDATA_IN_SIZE_IDX]);
        return true;
    } else {
        return false;
    }
}

void COSMPDummySensor10::set_fmi_sensor_data_out(const osi::SensorData& data)
{
    data.SerializeToString(&currentBuffer);
    encode_pointer_to_integer(currentBuffer.data(),integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]);
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX]=(fmiInteger)currentBuffer.length();
    private_log("Providing %08X %08X, writing from %p ...",integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX],currentBuffer.data());
    swap(currentBuffer,lastBuffer);
}

void COSMPDummySensor10::reset_fmi_sensor_data_out()
{
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]=0;
}

/*
 * Actual Core Content
 */

fmiStatus COSMPDummySensor10::doInit()
{
    DEBUGBREAK();

    /* Booleans */
    for (int i = 0; i<FMI_BOOLEAN_VARS; i++)
        boolean_vars[i] = fmiFalse;

    /* Integers */
    for (int i = 0; i<FMI_INTEGER_VARS; i++)
        integer_vars[i] = 0;

    /* Reals */
    for (int i = 0; i<FMI_REAL_VARS; i++)
        real_vars[i] = 0.0;

    /* Strings */
    for (int i = 0; i<FMI_STRING_VARS; i++)
        string_vars[i] = "";

    return fmiOK;
}

fmiStatus COSMPDummySensor10::doStart(fmiReal startTime, fmiBoolean stopTimeDefined, fmiReal stopTime)
{
    DEBUGBREAK();
    last_time = startTime;
    return fmiOK;
}

fmiStatus COSMPDummySensor10::doCalc(fmiReal currentCommunicationPoint, fmiReal communicationStepSize, fmiBoolean newStep)
{
    DEBUGBREAK();
    osi::SensorData currentIn,currentOut;
    if (fmi_source()) {
        /* We act as SensorData Source, so ignore inputs */
        currentOut.Clear();
        currentOut.mutable_timestamp()->set_seconds((long long int)floor(currentCommunicationPoint));
        currentOut.mutable_timestamp()->set_nanos((int)((currentCommunicationPoint - floor(currentCommunicationPoint))*1000000000.0));
        for (unsigned int i=0;i<10;i++) {
            osi::DetectedObject *obj = currentOut.add_object();
            obj->mutable_tracking_id()->set_value(i);
            obj->add_ground_truth_id()->set_value(i);
            obj->set_existence_probability(0.5);
        }
        set_fmi_sensor_data_out(currentOut);
        set_fmi_valid(true);
        set_fmi_count(currentOut.object_size());
    } else if (get_fmi_sensor_data_in(currentIn)) {
        /* Clear Output */
        currentOut.Clear();
        /* Copy Everything */
        currentOut.MergeFrom(currentIn);
        /* Adjust Timestamps and Ids */
        currentOut.mutable_timestamp()->set_seconds((long long int)floor(currentCommunicationPoint));
        currentOut.mutable_timestamp()->set_nanos((int)((currentCommunicationPoint - floor(currentCommunicationPoint))*1000000000.0));
        for_each(currentOut.mutable_object()->begin(),currentOut.mutable_object()->end(),
            [this](osi::DetectedObject& obj) {
                obj.mutable_tracking_id()->set_value(obj.tracking_id().value()+10);
                obj.set_existence_probability(obj.existence_probability()*0.9);
            });
        /* Serialize */
        set_fmi_sensor_data_out(currentOut);
        set_fmi_valid(true);
        set_fmi_count(currentOut.object_size());
    } else {
        /* We have no valid input, so no valid output */
        reset_fmi_sensor_data_out();
        set_fmi_valid(false);
        set_fmi_count(0);
    }
    return fmiOK;
}

fmiStatus COSMPDummySensor10::doTerm()
{
    DEBUGBREAK();
    return fmiOK;
}

void COSMPDummySensor10::doFree()
{
    DEBUGBREAK();
}

/*
 * Generic C++ Wrapper Code
 */

COSMPDummySensor10::COSMPDummySensor10(fmiString theinstanceName, fmiString thefmuGUID, fmiString thefmuLocation, fmiString themimeType, fmiReal thetimeout, fmiBoolean thevisible, fmiBoolean theinteractive, fmiCallbackFunctions thefunctions, fmiBoolean theloggingOn)
    : instanceName(theinstanceName),
    fmuGUID(thefmuGUID),
    fmuLocation(thefmuLocation),
	mimeType(themimeType),
	timeout(thetimeout),
    visible(!!thevisible),
    interactive(!!theinteractive),
    functions(thefunctions),
    loggingOn(!!theloggingOn),
    last_time(0.0)
{

}

COSMPDummySensor10::~COSMPDummySensor10()
{

}


fmiStatus COSMPDummySensor10::SetDebugLogging(fmiBoolean theloggingOn)
{
    private_log("fmiSetDebugLogging(%s)", theloggingOn ? "true" : "false");
    loggingOn = theloggingOn ? true : false;
    return fmiOK;
}

fmiComponent COSMPDummySensor10::Instantiate(fmiString instanceName, fmiString fmuGUID, fmiString fmuLocation, fmiString mimeType, fmiReal timeout, fmiBoolean visible, fmiBoolean interactive, fmiCallbackFunctions functions, fmiBoolean loggingOn)
{
    COSMPDummySensor10* myc = new COSMPDummySensor10(instanceName,fmuGUID,fmuLocation,mimeType,timeout,visible,interactive,functions,loggingOn);
    if (myc == NULL) {
        private_log_global("fmiInstantiate(\"%s\",\"%s\",\"%s\",\"%s\",%f,%d,%d,\"%s\",%d) = NULL (alloc failure)",
            (instanceName != NULL) ? instanceName : "<NULL>",
            (fmuGUID != NULL) ? fmuGUID : "<NULL>",
            (fmuLocation != NULL) ? fmuLocation : "<NULL>",
			(mimeType != NULL) ? mimeType : "<NULL>",
			timeout, visible, interactive, "FUNCTIONS", loggingOn);
        return NULL;
    }

    if (myc->doInit() != fmiOK) {
        private_log_global("fmiInstantiate(\"%s\",\"%s\",\"%s\",\"%s\",%f,%d,%d,\"%s\",%d) = NULL (doInit failure)",
            (instanceName != NULL) ? instanceName : "<NULL>",
            (fmuGUID != NULL) ? fmuGUID : "<NULL>",
            (fmuLocation != NULL) ? fmuLocation : "<NULL>",
			(mimeType != NULL) ? mimeType : "<NULL>",
			timeout, visible, interactive, "FUNCTIONS", loggingOn);
        delete myc;
        return NULL;
    }
    else {
        private_log_global("fmiInstantiate(\"%s\",\"%s\",\"%s\",\"%s\",%f,%d,%d,\"%s\",%d) = %p",
            (instanceName != NULL) ? instanceName : "<NULL>",
            (fmuGUID != NULL) ? fmuGUID : "<NULL>",
            (fmuLocation != NULL) ? fmuLocation : "<NULL>",
			(mimeType != NULL) ? mimeType : "<NULL>",
			timeout, visible, interactive, "FUNCTIONS", loggingOn, myc);
        return (fmiComponent)myc;
    }
}

fmiStatus COSMPDummySensor10::InitializeSlave(fmiReal startTime, fmiBoolean stopTimeDefined, fmiReal stopTime)
{
    private_log("fmiInitializeSlave(%g,%d,%g)", startTime, stopTimeDefined, stopTime);
    return doStart(startTime, stopTimeDefined, stopTime);
}

fmiStatus COSMPDummySensor10::DoStep(fmiReal currentCommunicationPoint, fmiReal communicationStepSize, fmiBoolean newStep)
{
    private_log("fmiDoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize, newStep);
    return doCalc(currentCommunicationPoint, communicationStepSize, newStep);
}

fmiStatus COSMPDummySensor10::Terminate()
{
    private_log("fmiTerminate()");
    return doTerm();
}

fmiStatus COSMPDummySensor10::Reset()
{
    private_log("fmiReset()");

    doFree();
    return doInit();
}

void COSMPDummySensor10::FreeInstance()
{
    private_log("fmiFreeInstance()");
    doFree();
}

fmiStatus COSMPDummySensor10::GetReal(const fmiValueReference vr[], size_t nvr, fmiReal value[])
{
    private_log("fmiGetReal(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_REAL_VARS)
            value[i] = real_vars[vr[i]];
        else
            return fmiError;
    }
    return fmiOK;
}

fmiStatus COSMPDummySensor10::GetInteger(const fmiValueReference vr[], size_t nvr, fmiInteger value[])
{
    private_log("fmiGetInteger(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_INTEGER_VARS)
            value[i] = integer_vars[vr[i]];
        else
            return fmiError;
    }
    return fmiOK;
}

fmiStatus COSMPDummySensor10::GetBoolean(const fmiValueReference vr[], size_t nvr, fmiBoolean value[])
{
    private_log("fmiGetBoolean(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_BOOLEAN_VARS)
            value[i] = boolean_vars[vr[i]];
        else
            return fmiError;
    }
    return fmiOK;
}

fmiStatus COSMPDummySensor10::GetString(const fmiValueReference vr[], size_t nvr, fmiString value[])
{
    private_log("fmiGetString(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_STRING_VARS)
            value[i] = string_vars[vr[i]].c_str();
        else
            return fmiError;
    }
    return fmiOK;
}

fmiStatus COSMPDummySensor10::SetReal(const fmiValueReference vr[], size_t nvr, const fmiReal value[])
{
    private_log("fmiSetReal(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_REAL_VARS)
            real_vars[vr[i]] = value[i];
        else
            return fmiError;
    }
    return fmiOK;
}

fmiStatus COSMPDummySensor10::SetInteger(const fmiValueReference vr[], size_t nvr, const fmiInteger value[])
{
    private_log("fmiSetInteger(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_INTEGER_VARS)
            integer_vars[vr[i]] = value[i];
        else
            return fmiError;
    }
    return fmiOK;
}

fmiStatus COSMPDummySensor10::SetBoolean(const fmiValueReference vr[], size_t nvr, const fmiBoolean value[])
{
    private_log("fmiSetBoolean(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_BOOLEAN_VARS)
            boolean_vars[vr[i]] = value[i];
        else
            return fmiError;
    }
    return fmiOK;
}

fmiStatus COSMPDummySensor10::SetString(const fmiValueReference vr[], size_t nvr, const fmiString value[])
{
    private_log("fmiSetString(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_STRING_VARS)
            string_vars[vr[i]] = value[i];
        else
            return fmiError;
    }
    return fmiOK;
}

/*
 * FMI 1.0 Co-Simulation Interface API
 */

extern "C" {

    DllExport const char* fmiGetTypesPlatform()
    {
        return fmiPlatform;
    }

    DllExport const char* fmiGetVersion()
    {
        return fmiVersion;
    }

    DllExport fmiStatus fmiSetDebugLogging(fmiComponent c, fmiBoolean loggingOn)
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->SetDebugLogging(loggingOn);
    }

    /*
    * Functions for Co-Simulation
    */
    DllExport fmiComponent fmiInstantiateSlave(fmiString instanceName,
        fmiString fmuGUID,
        fmiString fmuLocation,
        fmiString mimeType,
		fmiReal timeout,
        fmiBoolean visible,
		fmiBoolean interactive,
        fmiCallbackFunctions functions,
        fmiBoolean loggingOn)
    {
        return COSMPDummySensor10::Instantiate(instanceName, fmuGUID, fmuLocation, mimeType, timeout, visible, interactive, functions, loggingOn);
    }

    DllExport fmiStatus fmiInitializeSlave(fmiComponent c,
        fmiReal startTime,
        fmiBoolean stopTimeDefined,
        fmiReal stopTime)
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->InitializeSlave(startTime, stopTimeDefined, stopTime);
    }

    DllExport fmiStatus fmiDoStep(fmiComponent c,
        fmiReal currentCommunicationPoint,
        fmiReal communicationStepSize,
        fmiBoolean newStep)
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->DoStep(currentCommunicationPoint, communicationStepSize, newStep);
    }

    DllExport fmiStatus fmiTerminateSlave(fmiComponent c)
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->Terminate();
    }

    DllExport fmiStatus fmiResetSlave(fmiComponent c)
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->Reset();
    }

    DllExport void fmiFreeSlaveInstance(fmiComponent c)
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        myc->FreeInstance();
        delete myc;
    }

    /*
     * Data Exchange Functions
     */
    DllExport fmiStatus fmiGetReal(fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiReal value[])
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->GetReal(vr, nvr, value);
    }

    DllExport fmiStatus fmiGetInteger(fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiInteger value[])
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->GetInteger(vr, nvr, value);
    }

    DllExport fmiStatus fmiGetBoolean(fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiBoolean value[])
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->GetBoolean(vr, nvr, value);
    }

    DllExport fmiStatus fmiGetString(fmiComponent c, const fmiValueReference vr[], size_t nvr, fmiString value[])
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->GetString(vr, nvr, value);
    }

    DllExport fmiStatus fmiSetReal(fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiReal value[])
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->SetReal(vr, nvr, value);
    }

    DllExport fmiStatus fmiSetInteger(fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiInteger value[])
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->SetInteger(vr, nvr, value);
    }

    DllExport fmiStatus fmiSetBoolean(fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiBoolean value[])
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->SetBoolean(vr, nvr, value);
    }

    DllExport fmiStatus fmiSetString(fmiComponent c, const fmiValueReference vr[], size_t nvr, const fmiString value[])
    {
        COSMPDummySensor10* myc = (COSMPDummySensor10*)c;
        return myc->SetString(vr, nvr, value);
    }

    /*
     * Unsupported Features (FMUState, Derivatives, Async DoStep, Status Enquiries)
     */
    DllExport fmiStatus fmiSetRealInputDerivatives(fmiComponent c,
        const  fmiValueReference vr[],
        size_t nvr,
        const  fmiInteger order[],
        const  fmiReal value[])
    {
        return fmiError;
    }

    DllExport fmiStatus fmiGetRealOutputDerivatives(fmiComponent c,
        const   fmiValueReference vr[],
        size_t  nvr,
        const   fmiInteger order[],
        fmiReal value[])
    {
        return fmiError;
    }

    DllExport fmiStatus fmiCancelStep(fmiComponent c)
    {
        return fmiOK;
    }

    DllExport fmiStatus fmiGetStatus(fmiComponent c, const fmiStatusKind s, fmiStatus* value)
    {
        return fmiDiscard;
    }

    DllExport fmiStatus fmiGetRealStatus(fmiComponent c, const fmiStatusKind s, fmiReal* value)
    {
        return fmiDiscard;
    }

    DllExport fmiStatus fmiGetIntegerStatus(fmiComponent c, const fmiStatusKind s, fmiInteger* value)
    {
        return fmiDiscard;
    }

    DllExport fmiStatus fmiGetBooleanStatus(fmiComponent c, const fmiStatusKind s, fmiBoolean* value)
    {
        return fmiDiscard;
    }

    DllExport fmiStatus fmiGetStringStatus(fmiComponent c, const fmiStatusKind s, fmiString* value)
    {
        return fmiDiscard;
    }

}