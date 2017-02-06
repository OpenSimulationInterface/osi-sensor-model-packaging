/*
 * PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2017 PMSF IT Consulting Pierre R. Mai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "OSMPDummySensor.h"

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
ofstream COSMPDummySensor::private_log_file;
#endif

/*
 * ProtocolBuffer Accessors
 */

void* decode_integer_to_pointer(fmi2Integer hi,fmi2Integer lo)
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

void encode_pointer_to_integer(const void* ptr,fmi2Integer& hi,fmi2Integer& lo)
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

bool COSMPDummySensor::get_fmi_sensor_data_in(osi::SensorData& data)
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

void COSMPDummySensor::set_fmi_sensor_data_out(const osi::SensorData& data)
{
    data.SerializeToString(&currentBuffer);
    encode_pointer_to_integer(currentBuffer.data(),integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]);
	integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX]=(fmi2Integer)currentBuffer.length();
	private_log("Providing %08X %08X, writing from %p ...",integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX],currentBuffer.data());
	swap(currentBuffer,lastBuffer);
}

void COSMPDummySensor::reset_fmi_sensor_data_out()
{
	integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX]=0;
	integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]=0;
}

/*
 * Actual Core Content
 */

fmi2Status COSMPDummySensor::doInit()
{
	DEBUGBREAK();

	/* Booleans */
	for (int i = 0; i<FMI_BOOLEAN_VARS; i++)
		boolean_vars[i] = fmi2False;

	/* Integers */
	for (int i = 0; i<FMI_INTEGER_VARS; i++)
		integer_vars[i] = 0;

	/* Reals */
	for (int i = 0; i<FMI_REAL_VARS; i++)
		real_vars[i] = 0.0;

	/* Strings */
	for (int i = 0; i<FMI_STRING_VARS; i++)
		string_vars[i] = "";

	return fmi2OK;
}

fmi2Status COSMPDummySensor::doStart(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
	DEBUGBREAK();
	last_time = startTime;
	return fmi2OK;
}

fmi2Status COSMPDummySensor::doEnterInitializationMode()
{
	return fmi2OK;
}

fmi2Status COSMPDummySensor::doExitInitializationMode()
{
	return fmi2OK;
}

fmi2Status COSMPDummySensor::doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
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
	return fmi2OK;
}

fmi2Status COSMPDummySensor::doTerm()
{
	DEBUGBREAK();
	return fmi2OK;
}

void COSMPDummySensor::doFree()
{
	DEBUGBREAK();
}

/*
 * Generic C++ Wrapper Code
 */

COSMPDummySensor::COSMPDummySensor(fmi2String theinstanceName, fmi2Type thefmuType, fmi2String thefmuGUID, fmi2String thefmuResourceLocation, const fmi2CallbackFunctions* thefunctions, fmi2Boolean thevisible, fmi2Boolean theloggingOn)
	: instanceName(theinstanceName),
	fmuType(thefmuType),
	fmuGUID(thefmuGUID),
	fmuResourceLocation(thefmuResourceLocation),
	functions(*thefunctions),
	visible(!!thevisible),
	loggingOn(!!theloggingOn),
	last_time(0.0)
{

}

COSMPDummySensor::~COSMPDummySensor()
{

}


fmi2Status COSMPDummySensor::SetDebugLogging(fmi2Boolean theloggingOn,size_t nCategories, const fmi2String categories[])
{
	private_log("fmi2SetDebugLogging(%s)", theloggingOn ? "true" : "false");
	loggingOn = theloggingOn ? true : false;
	return fmi2OK;
}

fmi2Component COSMPDummySensor::Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID, fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions, fmi2Boolean visible, fmi2Boolean loggingOn)
{
	COSMPDummySensor* myc = new COSMPDummySensor(instanceName,fmuType,fmuGUID,fmuResourceLocation,functions,visible,loggingOn);

	if (myc == NULL) {
		private_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (alloc failure)",
			instanceName, fmuType, fmuGUID,
			(fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
			"FUNCTIONS", visible, loggingOn);
		return NULL;
	}

	if (myc->doInit() != fmi2OK) {
		private_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (doInit failure)",
			instanceName, fmuType, fmuGUID,
			(fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
			"FUNCTIONS", visible, loggingOn);
		delete myc;
		return NULL;
	}
	else {
		private_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = %p",
			instanceName, fmuType, fmuGUID,
			(fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
			"FUNCTIONS", visible, loggingOn, myc);
		return (fmi2Component)myc;
	}
}

fmi2Status COSMPDummySensor::SetupExperiment(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
	private_log("fmi2SetupExperiment(%d,%g,%g,%d,%g)", toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
	return doStart(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
}

fmi2Status COSMPDummySensor::EnterInitializationMode()
{
	private_log("fmi2EnterInitializationMode()");
	return doEnterInitializationMode();
}

fmi2Status COSMPDummySensor::ExitInitializationMode()
{
	private_log("fmi2ExitInitializationMode()");
	return doExitInitializationMode();
}

fmi2Status COSMPDummySensor::DoStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
	private_log("fmi2DoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    return doCalc(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
}

fmi2Status COSMPDummySensor::Terminate()
{
	private_log("fmi2Terminate()");
	return doTerm();
}

fmi2Status COSMPDummySensor::Reset()
{
	private_log("fmi2Reset()");

	doFree();
	return doInit();
}

void COSMPDummySensor::FreeInstance()
{
	private_log("fmi2FreeInstance()");
	doFree();
}

fmi2Status COSMPDummySensor::GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
{
	private_log("fmi2GetReal(...)");
	for (size_t i = 0; i<nvr; i++) {
		if (vr[i]<FMI_REAL_VARS)
			value[i] = real_vars[vr[i]];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPDummySensor::GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
	private_log("fmi2GetInteger(...)");
	for (size_t i = 0; i<nvr; i++) {
		if (vr[i]<FMI_INTEGER_VARS)
			value[i] = integer_vars[vr[i]];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPDummySensor::GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
	private_log("fmi2GetBoolean(...)");
	for (size_t i = 0; i<nvr; i++) {
		if (vr[i]<FMI_BOOLEAN_VARS)
			value[i] = boolean_vars[vr[i]];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPDummySensor::GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
	private_log("fmi2GetString(...)");
	for (size_t i = 0; i<nvr; i++) {
		if (vr[i]<FMI_STRING_VARS)
			value[i] = string_vars[vr[i]].c_str();
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPDummySensor::SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
{
	private_log("fmi2SetReal(...)");
	for (size_t i = 0; i<nvr; i++) {
		if (vr[i]<FMI_REAL_VARS)
			real_vars[vr[i]] = value[i];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPDummySensor::SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
{
	private_log("fmi2SetInteger(...)");
	for (size_t i = 0; i<nvr; i++) {
		if (vr[i]<FMI_INTEGER_VARS)
			integer_vars[vr[i]] = value[i];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPDummySensor::SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
	private_log("fmi2SetBoolean(...)");
	for (size_t i = 0; i<nvr; i++) {
		if (vr[i]<FMI_BOOLEAN_VARS)
			boolean_vars[vr[i]] = value[i];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPDummySensor::SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
	private_log("fmi2SetString(...)");
	for (size_t i = 0; i<nvr; i++) {
		if (vr[i]<FMI_STRING_VARS)
			string_vars[vr[i]] = value[i];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

/*
 * FMI 2.0 Co-Simulation Interface API
 */

extern "C" {

	FMI2_Export const char* fmi2GetTypesPlatform()
	{
		return fmi2TypesPlatform;
	}

	FMI2_Export const char* fmi2GetVersion()
	{
		return fmi2Version;
	}

	FMI2_Export fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories, const fmi2String categories[])
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->SetDebugLogging(loggingOn, nCategories, categories);
	}

	/*
	* Functions for Co-Simulation
	*/
	FMI2_Export fmi2Component fmi2Instantiate(fmi2String instanceName,
        fmi2Type fmuType,
        fmi2String fmuGUID,
        fmi2String fmuResourceLocation,
        const fmi2CallbackFunctions* functions,
        fmi2Boolean visible,
        fmi2Boolean loggingOn)
	{
		return COSMPDummySensor::Instantiate(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
	}

	FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c,
        fmi2Boolean toleranceDefined,
        fmi2Real tolerance,
        fmi2Real startTime,
        fmi2Boolean stopTimeDefined,
        fmi2Real stopTime)
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->SetupExperiment(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
	}

	FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c)
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->EnterInitializationMode();
	}

	FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c)
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->ExitInitializationMode();
	}

	FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,
        fmi2Real currentCommunicationPoint,
        fmi2Real communicationStepSize,
        fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->DoStep(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
	}

	FMI2_Export fmi2Status fmi2Terminate(fmi2Component c)
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->Terminate();
	}

	FMI2_Export fmi2Status fmi2Reset(fmi2Component c)
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->Reset();
	}

	FMI2_Export void fmi2FreeInstance(fmi2Component c)
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		myc->FreeInstance();
		delete myc;
	}

	/*
	 * Data Exchange Functions
	 */
	FMI2_Export fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->GetReal(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->GetInteger(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->GetBoolean(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->GetString(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->SetReal(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->SetInteger(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->SetBoolean(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
	{
		COSMPDummySensor* myc = (COSMPDummySensor*)c;
		return myc->SetString(vr, nvr, value);
	}

	/*
	 * Unsupported Features (FMUState, Derivatives, Async DoStep, Status Enquiries)
	 */
	FMI2_Export fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate)
	{
		return fmi2Error;
	}

    FMI2_Export fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate FMUstate, size_t *size)
	{
		return fmi2Error;
	}

    FMI2_Export fmi2Status fmi2SerializeFMUstate (fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size)
	{
		return fmi2Error;
	}

    FMI2_Export fmi2Status fmi2DeSerializeFMUstate (fmi2Component c, const fmi2Byte serializedState[], size_t size, fmi2FMUstate* FMUstate)
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2GetDirectionalDerivative(fmi2Component c,
        const fmi2ValueReference vUnknown_ref[], size_t nUnknown,
        const fmi2ValueReference vKnown_ref[] , size_t nKnown,
        const fmi2Real dvKnown[],
        fmi2Real dvUnknown[])
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2SetRealInputDerivatives(fmi2Component c,
		const  fmi2ValueReference vr[],
		size_t nvr,
		const  fmi2Integer order[],
		const  fmi2Real value[])
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c,
		const   fmi2ValueReference vr[],
		size_t  nvr,
		const   fmi2Integer order[],
		fmi2Real value[])
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2CancelStep(fmi2Component c)
	{
		return fmi2OK;
	}

	FMI2_Export fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value)
	{
		return fmi2Discard;
	}

	FMI2_Export fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value)
	{
		return fmi2Discard;
	}

	FMI2_Export fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer* value)
	{
		return fmi2Discard;
	}

	FMI2_Export fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean* value)
	{
		return fmi2Discard;
	}

	FMI2_Export fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value)
	{
		return fmi2Discard;
	}

}