/*
 * PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2017 PMSF IT Consulting Pierre R. Mai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "OSMPNetworkProxy.h"

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
#include <iomanip>
#include <string>
#include <algorithm>
#include <sstream>
#include <cstdint>
#include <cstring>
#include <cerrno>

using namespace std;

#ifdef PRIVATE_LOG_PATH
ofstream COSMPNetworkProxy::private_log_file;
#endif

/*
 * Binary Data Accessors
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


/*
 * TCP Proxy Communication
 */

int COSMPNetworkProxy::ensure_tcp_proxy_connection()
{
    struct addrinfo hints;
    struct addrinfo *result;
    int rc;
    
    if (tcp_proxy_socket != INVALID_SOCKET)
        return 1;
    
    std::memset(&hints,0,sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_NUMERICHOST;
    hints.ai_addrlen=0;
    hints.ai_canonname=0;
    hints.ai_addr=0;
    hints.ai_next=0;

    normal_log("NET","Connecting to %s:%s",fmi_address().c_str(),fmi_port().c_str());
    rc=getaddrinfo(fmi_address().c_str(),fmi_port().c_str(),&hints,&result);
    if (rc!=0 || !result) {
#ifdef _WIN32
        normal_log("NET","Error getting destination address: %d",WSAGetLastError());
#else
        normal_log("NET","Error getting destination address: %d (%s)",rc,gai_strerror(rc));
#endif
        return 0;
    }

    tcp_proxy_socket = socket(result->ai_family,SOCK_STREAM,IPPROTO_TCP);
    if (tcp_proxy_socket == INVALID_SOCKET) {
#ifdef _WIN32
        normal_log("NET","Error setting up Socket: %d",WSAGetLastError());
#else
        normal_log("NET","Error setting up Socket: %d (%s)",errno,strerror(errno));
#endif
        freeaddrinfo(result);
        return 0;
    }

    rc = connect(tcp_proxy_socket,result->ai_addr,result->ai_addrlen);

    if (rc != 0) {
#ifdef _WIN32
        normal_log("NET","Error setting up Socket Connection: %d",WSAGetLastError());
#else
        normal_log("NET","Error setting up Socket Connection: %d (%s)",errno,strerror(errno));
#endif
#ifdef _WIN32
        closesocket(tcp_proxy_socket);
#else
        close(tcp_proxy_socket);
#endif
        tcp_proxy_socket=INVALID_SOCKET;
        freeaddrinfo(result);
        return 0;
    }

    freeaddrinfo(result);
    return 1;
}

void COSMPNetworkProxy::close_tcp_proxy_connection()
{
    if (tcp_proxy_socket!=INVALID_SOCKET) {
#ifdef _WIN32
        closesocket(tcp_proxy_socket);
#else
        close(tcp_proxy_socket);
#endif
        tcp_proxy_socket=INVALID_SOCKET;
    }
}

/*
 * 0MQ Proxy Communication
 */

#ifdef WITH_ZMQ
int COSMPNetworkProxy::ensure_zmq_proxy_connection()
{
    if (zmq_proxy_socket != nullptr)
        return 1;

    zmq_proxy_socket = new zmq::socket_t(zmq_proxy_context, fmi_pubsub()?ZMQ_PUB:ZMQ_PAIR);
    try {
        if (fmi_pubsub()) {
            std::stringstream addrstream;
            addrstream << "tcp://*:" << fmi_port();
            string addr = addrstream.str();
            zmq_proxy_socket->bind(addr);
            normal_log("NET","Successfully bound to %s",addr.c_str());
        } else {
            std::stringstream addrstream;
            addrstream << "tcp://" << fmi_address() << ":" << fmi_port();
            string addr = addrstream.str();
            zmq_proxy_socket->connect(addr);
            normal_log("NET","Successfully connected to %s",addr.c_str());
        }
    } catch(const zmq::error_t& e) {
        normal_log("NET","Failed to set up 0MQ connection: %s",e.what());
        delete zmq_proxy_socket;
        zmq_proxy_socket=nullptr;
        return 0;
    }

    return 1;
}

void COSMPNetworkProxy::close_zmq_proxy_connection()
{
    if (zmq_proxy_socket!=nullptr) {
        zmq_proxy_socket->close();
        delete zmq_proxy_socket;
        zmq_proxy_socket=nullptr;
    }
}
#endif

/*
 * Actual Core Content
 */

fmi2Status COSMPNetworkProxy::doInit()
{
    DEBUGBREAK();

    /* Booleans */
    for (int i = 0; i<FMI_BOOLEAN_VARS; i++)
        boolean_vars[i] = fmi2False;

#ifdef WITH_ZMQ
    set_fmi_zmq(fmi2True);
    set_fmi_pubsub(fmi2True);
#endif

    /* Integers */
    for (int i = 0; i<FMI_INTEGER_VARS; i++)
        integer_vars[i] = 0;

    /* Reals */
    for (int i = 0; i<FMI_REAL_VARS; i++)
        real_vars[i] = 0.0;

    /* Strings */
    for (int i = 0; i<FMI_STRING_VARS; i++)
        string_vars[i] = "";

    set_fmi_address("127.0.0.1");
    set_fmi_port("3456");

    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::doStart(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    DEBUGBREAK();
    last_time = startTime;
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::doEnterInitializationMode()
{
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::doExitInitializationMode()
{
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
    DEBUGBREAK();

    set_fmi_valid(false);
    set_fmi_sent(false);

    if (integer_vars[FMI_INTEGER_SENSORDATA_IN_SIZE_IDX] > 0) {
        void* buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORDATA_IN_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_IN_BASELO_IDX]);
        int buffersize = integer_vars[FMI_INTEGER_SENSORDATA_IN_SIZE_IDX];
        normal_log("OSMP","Got %08X %08X LEN %08X, reading from %p (length %i)...",integer_vars[FMI_INTEGER_SENSORDATA_IN_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_IN_BASELO_IDX],buffersize,buffer,buffersize);
        if (fmi_datalog()) {
            for (int i=0;i<buffersize;i+=16) {
                std::stringstream hexline;
                hexline << std::hex << std::setfill('0') << std::setw(2);
                for (int j=0;((i+j)<buffersize) && (j<16);j++) {
                    hexline << " " << (int)((unsigned char*)buffer)[i+j];
                }
                normal_log("OSMP","       %s",hexline.str().c_str());
            }
        }
        set_fmi_valid(true);

        if (!fmi_dummy()) {
#ifdef WITH_ZMQ
            if (fmi_zmq()) {
                if (ensure_zmq_proxy_connection()) {
                    try {
                        if (!zmq_proxy_socket->send(buffer,buffersize)) {
                            normal_log("NET","Failed to send message with size %d.",buffersize);
                        } else {
                            normal_log("NET","Successfully sent zmq message with size %d.",buffersize);
                            set_fmi_sent(true);
                        }
                    } catch(const zmq::error_t& e) {
                        normal_log("NET","Failed to send message with size %d: %s",buffersize,e.what());
                    }
                }
            } else {
#endif
                if (ensure_tcp_proxy_connection()) {
                    int sendval=send(tcp_proxy_socket,(char*)&buffersize,sizeof(buffersize),0);
                    if (sendval!=sizeof(buffersize)) {
#ifdef _WIN32
                        normal_log("NET","Failed to send message size (%d): %d",buffersize,WSAGetLastError());
#else
                        normal_log("NET","Failed to send message size (%d): %d (%s)",buffersize,errno,strerror(errno));
#endif
                        close_tcp_proxy_connection();
                    } else {
                        sendval=send(tcp_proxy_socket,(char*)buffer,buffersize,0);
                        if (sendval!=buffersize) {
#ifdef _WIN32
                            normal_log("NET","Failed to send message itself with size %d: %d",buffersize,WSAGetLastError());
#else
                            normal_log("NET","Failed to send message itself with size %d: %d (%s)",buffersize,errno,strerror(errno));
#endif
                            close_tcp_proxy_connection();
                        } else {
                            normal_log("NET","Successfully sent tcp message with size %d.",buffersize);
                            set_fmi_sent(true);
                        }
                    }
                }
#ifdef WITH_ZMQ
            }
#endif
        }
    }

    last_time=currentCommunicationPoint+communicationStepSize;
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::doTerm()
{
    DEBUGBREAK();
    return fmi2OK;
}

void COSMPNetworkProxy::doFree()
{
    DEBUGBREAK();
}

/*
 * Generic C++ Wrapper Code
 */

COSMPNetworkProxy::COSMPNetworkProxy(fmi2String theinstanceName, fmi2Type thefmuType, fmi2String thefmuGUID, fmi2String thefmuResourceLocation, const fmi2CallbackFunctions* thefunctions, fmi2Boolean thevisible, fmi2Boolean theloggingOn)
    : instanceName(theinstanceName),
    fmuType(thefmuType),
    fmuGUID(thefmuGUID),
    fmuResourceLocation(thefmuResourceLocation),
    functions(*thefunctions),
    visible(!!thevisible),
    loggingOn(!!theloggingOn),
    last_time(0.0),
    tcp_proxy_socket(INVALID_SOCKET)
#ifdef WITH_ZMQ
    ,zmq_proxy_context(),
    zmq_proxy_socket(nullptr)
#endif
{
    loggingCategories.clear();
    loggingCategories.insert("FMI");
    loggingCategories.insert("OSMP");
    loggingCategories.insert("NET");
#ifdef _WIN32
    long rc;
    WSADATA WsaDat;
    if ((rc=WSAStartup(MAKEWORD(2,2),&WsaDat)) != 0) {
        normal_log("NET","Error %d setting up Windows Socket Communications.",rc);
        WSACleanup();
    }
#endif
}

COSMPNetworkProxy::~COSMPNetworkProxy()
{
#ifdef WITH_ZMQ
    close_zmq_proxy_connection();
#endif
    close_tcp_proxy_connection();
#ifdef _WIN32
    WSACleanup();
#endif
}

fmi2Status COSMPNetworkProxy::SetDebugLogging(fmi2Boolean theloggingOn,size_t nCategories, const fmi2String categories[])
{
    fmi_verbose_log("fmi2SetDebugLogging(%s)", theloggingOn ? "true" : "false");
    loggingOn = theloggingOn ? true : false;
    if (categories && (nCategories > 0)) {
        loggingCategories.clear();
        for (size_t i=0;i<nCategories;i++) {
            if (categories[i] == "FMI")
                loggingCategories.insert("FMI");
            else if (categories[i] == "OSMP")
                loggingCategories.insert("OSMP");
            else if (categories[i] == "NET")
                loggingCategories.insert("NET");
        }
    } else {
        loggingCategories.clear();
        loggingCategories.insert("FMI");
        loggingCategories.insert("OSMP");
        loggingCategories.insert("NET");
    }
    return fmi2OK;
}

fmi2Component COSMPNetworkProxy::Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID, fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions, fmi2Boolean visible, fmi2Boolean loggingOn)
{
    COSMPNetworkProxy* myc = nullptr;
    try {
        myc = new COSMPNetworkProxy(instanceName,fmuType,fmuGUID,fmuResourceLocation,functions,visible,loggingOn);

        if (myc == nullptr) {
            fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (alloc failure)",
                instanceName, fmuType, fmuGUID,
                (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
                "FUNCTIONS", visible, loggingOn);
            return NULL;
        }

        if (myc->doInit() != fmi2OK) {
            fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (doInit failure)",
                instanceName, fmuType, fmuGUID,
                (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
                "FUNCTIONS", visible, loggingOn);
            delete myc;
            return NULL;
        } else {
            fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = %p",
                instanceName, fmuType, fmuGUID,
                (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
                "FUNCTIONS", visible, loggingOn, myc);
            return (fmi2Component)myc;
        }
    } catch (std::exception &e) {
        try {
            fmi_verbose_log_global("Unhandled exception in fmi2Instantiate: %s",e.what());
            if (myc!=nullptr)
                delete myc;
        } catch (...) {
        }
        return NULL;
    }
}

fmi2Status COSMPNetworkProxy::SetupExperiment(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    fmi_verbose_log("fmi2SetupExperiment(%d,%g,%g,%d,%g)", toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    try {
        return doStart(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    } catch (std::exception &e) {
        try {
            fmi_verbose_log("Unhandled exception in fmi2SetupExperiment: %s",e.what());
        } catch (...) {
        }
        return fmi2Error;
    }
}

fmi2Status COSMPNetworkProxy::EnterInitializationMode()
{
    fmi_verbose_log("fmi2EnterInitializationMode()");
    try {
        return doEnterInitializationMode();
    } catch (std::exception &e) {
        try {
            fmi_verbose_log("Unhandled exception in fmi2EnterInitializationMode: %s",e.what());
        } catch (...) {
        }
        return fmi2Error;
    }
}

fmi2Status COSMPNetworkProxy::ExitInitializationMode()
{
    fmi_verbose_log("fmi2ExitInitializationMode()");
    try {
        return doExitInitializationMode();
    } catch (std::exception &e) {
        try {
            fmi_verbose_log("Unhandled exception in fmi2ExitInitializationMode: %s",e.what());
        } catch (...) {
        }
        return fmi2Error;
    }
}

fmi2Status COSMPNetworkProxy::DoStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
    fmi_verbose_log("fmi2DoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    try {
        return doCalc(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    } catch (std::exception &e) {
        try {
            fmi_verbose_log("Unhandled exception in fmi2DoStep: %s",e.what());
        } catch (...) {
        }
        return fmi2Error;
    }
}

fmi2Status COSMPNetworkProxy::Terminate()
{
    fmi_verbose_log("fmi2Terminate()");
    try {
        return doTerm();
    } catch (std::exception &e) {
        try {
            fmi_verbose_log("Unhandled exception in fmi2Terminate: %s",e.what());
        } catch (...) {
        }
        return fmi2Error;
    }
}

fmi2Status COSMPNetworkProxy::Reset()
{
    fmi_verbose_log("fmi2Reset()");
    try {
        doFree();
        return doInit();
    } catch (std::exception &e) {
        try {
            fmi_verbose_log("Unhandled exception in fmi2Reset: %s",e.what());
        } catch (...) {
        }
        return fmi2Error;
    }
}

void COSMPNetworkProxy::FreeInstance()
{
    fmi_verbose_log("fmi2FreeInstance()");
    try {
        doFree();
    } catch (std::exception &e) {
        try {
            fmi_verbose_log("Unhandled exception in fmi2FreeInstance: %s",e.what());
        } catch (...) {
        }
    }
}

fmi2Status COSMPNetworkProxy::GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
{
    fmi_verbose_log("fmi2GetReal(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_REAL_VARS)
            value[i] = real_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
    fmi_verbose_log("fmi2GetInteger(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_INTEGER_VARS)
            value[i] = integer_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
    fmi_verbose_log("fmi2GetBoolean(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_BOOLEAN_VARS)
            value[i] = boolean_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
    fmi_verbose_log("fmi2GetString(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_STRING_VARS)
            value[i] = string_vars[vr[i]].c_str();
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
{
    fmi_verbose_log("fmi2SetReal(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_REAL_VARS)
            real_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
{
    fmi_verbose_log("fmi2SetInteger(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_INTEGER_VARS)
            integer_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
    fmi_verbose_log("fmi2SetBoolean(...)");
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_BOOLEAN_VARS)
            boolean_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPNetworkProxy::SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
    fmi_verbose_log("fmi2SetString(...)");
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
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
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
        return COSMPNetworkProxy::Instantiate(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
    }

    FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c,
        fmi2Boolean toleranceDefined,
        fmi2Real tolerance,
        fmi2Real startTime,
        fmi2Boolean stopTimeDefined,
        fmi2Real stopTime)
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->SetupExperiment(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    }

    FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c)
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->EnterInitializationMode();
    }

    FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c)
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->ExitInitializationMode();
    }

    FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,
        fmi2Real currentCommunicationPoint,
        fmi2Real communicationStepSize,
        fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->DoStep(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    }

    FMI2_Export fmi2Status fmi2Terminate(fmi2Component c)
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->Terminate();
    }

    FMI2_Export fmi2Status fmi2Reset(fmi2Component c)
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->Reset();
    }

    FMI2_Export void fmi2FreeInstance(fmi2Component c)
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        myc->FreeInstance();
        delete myc;
    }

    /*
     * Data Exchange Functions
     */
    FMI2_Export fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->GetReal(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->GetInteger(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->GetBoolean(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->GetString(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->SetReal(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->SetInteger(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
        return myc->SetBoolean(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
    {
        COSMPNetworkProxy* myc = (COSMPNetworkProxy*)c;
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
