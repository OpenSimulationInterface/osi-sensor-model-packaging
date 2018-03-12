/*
 * PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2017 PMSF IT Consulting Pierre R. Mai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "OSMPCNetworkProxy.h"

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

#include <stdio.h>


#ifdef PRIVATE_LOG_PATH
FILE* OSMPCNetworkProxy_private_log_file;
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
    return (void*)(myaddr.address);
#elif PTRDIFF_MAX == INT32_MAX
    return (void*)(lo);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

void encode_pointer_to_integer(const void* ptr,fmi2Integer* hi,fmi2Integer* lo)
{
#if PTRDIFF_MAX == INT64_MAX
    union addrconv {
        struct {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr;
    myaddr.address=(unsigned long long)(ptr);
    *hi=myaddr.base.hi;
    *lo=myaddr.base.lo;
#elif PTRDIFF_MAX == INT32_MAX
    *hi=0;
    *lo=(int)(ptr);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}


/*
 * TCP Proxy Communication
 */

int ensure_tcp_proxy_connection(OSMPCNetworkProxy component)
{
    struct addrinfo hints;
    struct addrinfo *result;
    int rc;
    
    if (component->tcp_proxy_socket != INVALID_SOCKET)
        return 1;
    
    memset(&hints,0,sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_NUMERICHOST;
    hints.ai_addrlen=0;
    hints.ai_canonname=0;
    hints.ai_addr=0;
    hints.ai_next=0;

    normal_log(component,"NET","Connecting to %s:%s",component->string_vars[FMI_STRING_ADDRESS_IDX],component->string_vars[FMI_STRING_PORT_IDX]);
    rc=getaddrinfo(component->string_vars[FMI_STRING_ADDRESS_IDX],component->string_vars[FMI_STRING_PORT_IDX],&hints,&result);
    if (rc!=0 || !result) {
#ifdef _WIN32
        normal_log(component,"NET","Error getting destination address: %d",WSAGetLastError());
#else
        normal_log(component,"NET","Error getting destination address: %d (%s)",rc,gai_strerror(rc));
#endif
        return 0;
    }

    component->tcp_proxy_socket = socket(result->ai_family,SOCK_STREAM,IPPROTO_TCP);
    if (component->tcp_proxy_socket == INVALID_SOCKET) {
#ifdef _WIN32
        normal_log(component,"NET","Error setting up Socket: %d",WSAGetLastError());
#else
        normal_log(component,"NET","Error setting up Socket: %d (%s)",errno,strerror(errno));
#endif
        freeaddrinfo(result);
        return 0;
    }

    rc = connect(component->tcp_proxy_socket,result->ai_addr,result->ai_addrlen);

    if (rc != 0) {
#ifdef _WIN32
        normal_log(component,"NET","Error setting up Socket Connection: %d",WSAGetLastError());
#else
        normal_log(component,"NET","Error setting up Socket Connection: %d (%s)",errno,strerror(errno));
#endif
#ifdef _WIN32
        closesocket(component->tcp_proxy_socket);
#else
        close(component->tcp_proxy_socket);
#endif
        component->tcp_proxy_socket=INVALID_SOCKET;
        freeaddrinfo(result);
        return 0;
    }

    freeaddrinfo(result);
    return 1;
}

void close_tcp_proxy_connection(OSMPCNetworkProxy component)
{
    if (component->tcp_proxy_socket!=INVALID_SOCKET) {
#ifdef _WIN32
        closesocket(component->tcp_proxy_socket);
#else
        close(component->tcp_proxy_socket);
#endif
        component->tcp_proxy_socket=INVALID_SOCKET;
    }
}

/*
 * Actual Core Content
 */

fmi2Status doInit(OSMPCNetworkProxy component)
{
    int i;
    
    DEBUGBREAK();

    /* Booleans */
    for (i = 0; i<FMI_BOOLEAN_VARS; i++)
        component->boolean_vars[i] = fmi2False;

    component->boolean_vars[FMI_BOOLEAN_SENDER_IDX] = fmi2True;
    component->boolean_vars[FMI_BOOLEAN_RECEIVER_IDX] = fmi2True;

    /* Integers */
    for (i = 0; i<FMI_INTEGER_VARS; i++)
        component->integer_vars[i] = 0;

    /* Reals */
    for (i = 0; i<FMI_REAL_VARS; i++)
        component->real_vars[i] = 0.0;

    /* Strings */
    component->string_vars[FMI_STRING_ADDRESS_IDX]=strdup(FMU_DEFAULT_ADDRESS);
    component->string_vars[FMI_STRING_PORT_IDX]=strdup(FMU_DEFAULT_PORT);

    return fmi2OK;
}

fmi2Status doStart(OSMPCNetworkProxy component,fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    DEBUGBREAK();
    component->last_time = startTime;
    return fmi2OK;
}

fmi2Status doEnterInitializationMode(OSMPCNetworkProxy component)
{
    return fmi2OK;
}

fmi2Status doExitInitializationMode(OSMPCNetworkProxy component)
{
    return fmi2OK;
}

fmi2Status doCalc(OSMPCNetworkProxy component, fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
    void* buffer=NULL;
    int buffersize=0;

    DEBUGBREAK();

    component->boolean_vars[FMI_BOOLEAN_INPUT_VALID_IDX]=fmi2False;
    component->boolean_vars[FMI_BOOLEAN_INPUT_SENT_IDX]=fmi2False;

    buffer = decode_integer_to_pointer(component->integer_vars[FMI_INTEGER_SENSORDATA_IN_BASEHI_IDX],component->integer_vars[FMI_INTEGER_SENSORDATA_IN_BASELO_IDX]);
    buffersize = component->integer_vars[FMI_INTEGER_SENSORDATA_IN_SIZE_IDX];

    if (component->integer_vars[FMI_INTEGER_SENSORDATA_IN_SIZE_IDX] > 0) {
        normal_log(component,"OSMP","Got %08X %08X LEN %08X, reading from %p (length %i)...",component->integer_vars[FMI_INTEGER_SENSORDATA_IN_BASEHI_IDX],component->integer_vars[FMI_INTEGER_SENSORDATA_IN_BASELO_IDX],buffersize,buffer,buffersize);
        if (component->boolean_vars[FMI_BOOLEAN_LOG_DATA_IDX]) {
            int i=0;
            for (i=0;i<buffersize;i+=16) {
                static char hexmap[]="0123456789ABCDEF";
                char hexline[50];
                char *ptr;
                int j=0;
                for (j=0,ptr=hexline;((i+j)<buffersize) && (j<16);j++) {
                    unsigned char byte = ((unsigned char*)buffer)[i+j];
                    *ptr++=' ';
                    *ptr++=hexmap[byte>>4];
                    *ptr++=hexmap[byte&0xF];
                }
                *ptr='\0';
                normal_log(component,"OSMP","       %s",hexline);
            }
        }
        component->boolean_vars[FMI_BOOLEAN_INPUT_VALID_IDX]=fmi2True;
    }

    if (!component->boolean_vars[FMI_BOOLEAN_DUMMY_IDX] && component->boolean_vars[FMI_BOOLEAN_SENDER_IDX]) {
        if (ensure_tcp_proxy_connection(component)) {
            int sendval=send(component->tcp_proxy_socket,(char*)&buffersize,sizeof(buffersize),0);
            if (sendval!=sizeof(buffersize)) {
#ifdef _WIN32
                normal_log(component,"NET","Failed to send message size (%d): %d",buffersize,WSAGetLastError());
#else
                normal_log(component,"NET","Failed to send message size (%d): %d (%s)",buffersize,errno,strerror(errno));
#endif
                close_tcp_proxy_connection(component);
            } else {
                if (buffersize > 0) {
                    sendval=send(component->tcp_proxy_socket,(char*)buffer,buffersize,0);
                    if (sendval!=buffersize) {
    #ifdef _WIN32
                        normal_log(component,"NET","Failed to send message itself with size %d: %d",buffersize,WSAGetLastError());
    #else
                        normal_log(component,"NET","Failed to send message itself with size %d: %d (%s)",buffersize,errno,strerror(errno));
    #endif
                        close_tcp_proxy_connection(component);
                    } else {
                        normal_log(component,"NET","Successfully sent tcp message with size %d.",buffersize);
                        component->boolean_vars[FMI_BOOLEAN_INPUT_SENT_IDX]=fmi2True;
                    }
                } else {
                    normal_log(component,"NET","Successfully sent empty tcp message with size %d.",buffersize);
                    component->boolean_vars[FMI_BOOLEAN_INPUT_SENT_IDX]=fmi2True;
                }
            }
        }
    }

    if (!component->boolean_vars[FMI_BOOLEAN_DUMMY_IDX] && component->boolean_vars[FMI_BOOLEAN_RECEIVER_IDX]) {
        /* Switch Buffers */
        if (component->prev_output_buffer_ptr != NULL) {
            free(component->prev_output_buffer_ptr);
            component->prev_output_buffer_ptr=NULL;
            component->prev_output_buffer_size=0;
        }
        component->prev_output_buffer_ptr = component->output_buffer_ptr;
        component->prev_output_buffer_size = component->output_buffer_size;
        component->output_buffer_ptr = NULL;
        component->output_buffer_size = 0;
        component->integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX] = 0;
        component->integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX] = 0;
        component->integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX] = 0;

        if (ensure_tcp_proxy_connection(component)) {
            int recv_buffer_size=0;
            char* recv_buffer_ptr=NULL;
            int recvval=0;
            recvval=recv(component->tcp_proxy_socket,(char*)&(recv_buffer_size),sizeof(recv_buffer_size),MSG_WAITALL);
            if (recvval!=sizeof(recv_buffer_size)) {
#ifdef _WIN32
                normal_log(component,"NET","Failed to recv message size (%d): %d",sizeof(recv_buffer_size),WSAGetLastError());
#else
                normal_log(component,"NET","Failed to recv message size (%d): %d (%s)",sizeof(recv_buffer_size),errno,strerror(errno));
#endif
                close_tcp_proxy_connection(component);
            } else if (recv_buffer_size == 0) {
                normal_log(component,"NET","Successfully recv empty tcp message with size %d.",recv_buffer_size);
                component->boolean_vars[FMI_BOOLEAN_OUTPUT_RECEIVED_IDX] = fmi2True;
                component->output_buffer_ptr = NULL;
                component->output_buffer_size = recv_buffer_size;
                component->integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX] = 0;
                component->integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX] = 0;
                component->integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX] = recv_buffer_size;
                component->boolean_vars[FMI_BOOLEAN_OUTPUT_VALID_IDX] = fmi2False;
            } else {
                recv_buffer_ptr = calloc(recv_buffer_size,1);
                if (recv_buffer_ptr == NULL) {
                    normal_log(component,"NET","Failed to allocated recv message buffer of size (%d)",recv_buffer_size);
                    close_tcp_proxy_connection(component);
                } else {
                    recvval=recv(component->tcp_proxy_socket,recv_buffer_ptr,recv_buffer_size,MSG_WAITALL);
                    if (recvval!=recv_buffer_size) {
#ifdef _WIN32
                        normal_log(component,"NET","Failed to recv message itself with size %d: %d",recv_buffer_size,WSAGetLastError());
#else
                        normal_log(component,"NET","Failed to recv message itself with size %d: %d (%s)",recv_buffer_size,errno,strerror(errno));
#endif
                        close_tcp_proxy_connection(component);
                    } else {
                        normal_log(component,"NET","Successfully recv tcp message with size %d.",recv_buffer_size);
                        component->boolean_vars[FMI_BOOLEAN_OUTPUT_RECEIVED_IDX] = fmi2True;
                        component->output_buffer_ptr = recv_buffer_ptr;
                        component->output_buffer_size = recv_buffer_size;
                        encode_pointer_to_integer(recv_buffer_ptr,&(component->integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX]),&(component->integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]));
                        component->integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX] = recv_buffer_size;
                        component->boolean_vars[FMI_BOOLEAN_OUTPUT_VALID_IDX] = fmi2True;
                    }
                }
            }
        }
    }

    component->last_time=currentCommunicationPoint+communicationStepSize;
    return fmi2OK;
}

fmi2Status doTerm(OSMPCNetworkProxy component)
{
    DEBUGBREAK();
    return fmi2OK;
}

void doFree(OSMPCNetworkProxy component)
{
    DEBUGBREAK();
    if (component->prev_output_buffer_ptr!=NULL) {
        free(component->prev_output_buffer_ptr);
        component->prev_output_buffer_ptr=NULL;
        component->prev_output_buffer_size=0;
    }
    if (component->output_buffer_ptr!=NULL) {
        free(component->output_buffer_ptr);
        component->output_buffer_ptr=NULL;
        component->output_buffer_size=0;
    }
}

/*
 * FMI 2.0 Co-Simulation Interface API
 */

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
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    fmi_verbose_log(myc,"fmi2SetDebugLogging(%s)", loggingOn ? "true" : "false");
    myc->loggingOn = loggingOn ? 1 : 0;
    
    for (;myc->loggingCategories!=NULL && myc->nCategories>0;) free(myc->loggingCategories[--(myc->nCategories)]);
    free(myc->loggingCategories);
    myc->loggingCategories = NULL;
    myc->nCategories = 0;

    if (categories && (nCategories > 0)) {
        myc->loggingCategories = calloc(nCategories,sizeof(char*));
        if (myc->loggingCategories != NULL) {
            size_t i;
            myc->nCategories = nCategories;
            for (i=0;i<nCategories;i++) myc->loggingCategories[i]=strdup(categories[i]);
        }
    } else {
        myc->loggingCategories = calloc(3,sizeof(char*));
        if (myc->loggingCategories != NULL) {
            myc->nCategories = 3;
            myc->loggingCategories[0] = strdup("FMI");
            myc->loggingCategories[1] = strdup("OSMP");
            myc->loggingCategories[2] = strdup("NET");
        }
    }
    
    return fmi2OK;
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
#ifdef _WIN32
    long rc;
    WSADATA WsaDat;
#endif
    OSMPCNetworkProxy myc = NULL;
    
#ifdef FMU_GUID
    if (fmuGUID!=NULL && 0!=strcmp(fmuGUID,FMU_GUID)) {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (GUID mismatch, expected %s)",
            instanceName, fmuType, fmuGUID,
            (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
            "FUNCTIONS", visible, loggingOn, FMU_GUID);
        return NULL;
    }
#endif
    
    myc = calloc(1,sizeof(struct OSMPCNetworkProxy));
    
    if (myc == NULL) {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (alloc failure)",
            instanceName, fmuType, fmuGUID,
            (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
            "FUNCTIONS", visible, loggingOn);
        return NULL;
    }
    
    myc->instanceName=strdup(instanceName);
    myc->fmuType=fmuType;
    myc->fmuGUID=strdup(fmuGUID);
    myc->fmuResourceLocation=strdup(fmuResourceLocation);
    myc->functions.logger=functions->logger;
    myc->functions.allocateMemory=functions->allocateMemory;
    myc->functions.freeMemory=functions->freeMemory;
    myc->functions.stepFinished=functions->stepFinished;
    myc->functions.componentEnvironment=functions->componentEnvironment;
    myc->visible=visible;
    myc->loggingOn=loggingOn;
    myc->last_time=0.0;
    myc->tcp_proxy_socket=INVALID_SOCKET;
    myc->output_buffer_ptr=NULL;
    myc->output_buffer_size=0;
    myc->prev_output_buffer_ptr=NULL;
    myc->prev_output_buffer_size=0;

    myc->loggingCategories = calloc(3,sizeof(char*));
    if (myc->loggingCategories != NULL) {
        myc->nCategories = 3;
        myc->loggingCategories[0] = strdup("FMI");
        myc->loggingCategories[1] = strdup("OSMP");
        myc->loggingCategories[2] = strdup("NET");
    }
    
#ifdef _WIN32
    if ((rc=WSAStartup(MAKEWORD(2,2),&WsaDat)) != 0) {
        normal_log(myc,"NET","Error %d setting up Windows Socket Communications.",rc);
        WSACleanup();
    }
#endif
    
    if (doInit(myc) != fmi2OK) {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (doInit failure)",
            instanceName, fmuType, fmuGUID,
            (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
            "FUNCTIONS", visible, loggingOn);
        free(myc->fmuResourceLocation);
        free(myc->fmuGUID);
        free(myc->instanceName);
        for (;myc->loggingCategories!=NULL && myc->nCategories>0;) free(myc->loggingCategories[--(myc->nCategories)]);
        free(myc->loggingCategories);
        free(myc);
        return NULL;
    }
    fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = %p",
        instanceName, fmuType, fmuGUID,
        (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
        "FUNCTIONS", visible, loggingOn, myc);
    return (fmi2Component)myc;
}
    
FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c,
    fmi2Boolean toleranceDefined,
    fmi2Real tolerance,
    fmi2Real startTime,
    fmi2Boolean stopTimeDefined,
    fmi2Real stopTime)
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    fmi_verbose_log(myc,"fmi2SetupExperiment(%d,%g,%g,%d,%g)", toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    return doStart(myc,toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
}

FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c)
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    fmi_verbose_log(myc,"fmi2EnterInitializationMode()");
    return doEnterInitializationMode(myc);
}

FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c)
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    fmi_verbose_log(myc,"fmi2ExitInitializationMode()");
    return doExitInitializationMode(myc);
}

FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,
    fmi2Real currentCommunicationPoint,
    fmi2Real communicationStepSize,
    fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    fmi_verbose_log(myc,"fmi2DoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    return doCalc(myc,currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
}

FMI2_Export fmi2Status fmi2Terminate(fmi2Component c)
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    fmi_verbose_log(myc,"fmi2Terminate()");
    return doTerm(myc);
}

FMI2_Export fmi2Status fmi2Reset(fmi2Component c)
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    fmi_verbose_log(myc,"fmi2Reset()");
    doFree(myc);
    return doInit(myc);
}

FMI2_Export void fmi2FreeInstance(fmi2Component c)
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    fmi_verbose_log(myc,"fmi2FreeInstance()");
    doFree(myc);

    close_tcp_proxy_connection(myc);
#ifdef _WIN32
    WSACleanup();
#endif

    free(myc->fmuResourceLocation);
    free(myc->fmuGUID);
    free(myc->instanceName);
    for (;myc->loggingCategories!=NULL && myc->nCategories>0;) free(myc->loggingCategories[--(myc->nCategories)]);
    free(myc->loggingCategories);
    free(myc);
}

/*
 * Data Exchange Functions
 */
FMI2_Export fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    size_t i;
    fmi_verbose_log(myc,"fmi2GetReal(...)");
    for (i = 0; i<nvr; i++) {
        if (vr[i]<FMI_REAL_VARS)
            value[i] = myc->real_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    size_t i;
    fmi_verbose_log(myc,"fmi2GetInteger(...)");
    for (i = 0; i<nvr; i++) {
        if (vr[i]<FMI_INTEGER_VARS)
            value[i] = myc->integer_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    size_t i;
    fmi_verbose_log(myc,"fmi2GetBoolean(...)");
    for (i = 0; i<nvr; i++) {
        if (vr[i]<FMI_BOOLEAN_VARS)
            value[i] = myc->boolean_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    size_t i;
    fmi_verbose_log(myc,"fmi2GetString(...)");
    for (i = 0; i<nvr; i++) {
        if (vr[i]<FMI_STRING_VARS)
            value[i] = myc->string_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    size_t i;
    fmi_verbose_log(myc,"fmi2SetReal(...)");
    for (i = 0; i<nvr; i++) {
        if (vr[i]<FMI_REAL_VARS)
            myc->real_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    size_t i;
    fmi_verbose_log(myc,"fmi2SetInteger(...)");
    for (i = 0; i<nvr; i++) {
        if (vr[i]<FMI_INTEGER_VARS)
            myc->integer_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    size_t i;
    fmi_verbose_log(myc,"fmi2SetBoolean(...)");
    for (i = 0; i<nvr; i++) {
        if (vr[i]<FMI_BOOLEAN_VARS)
            myc->boolean_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
    OSMPCNetworkProxy myc = (OSMPCNetworkProxy)c;
    size_t i;
    fmi_verbose_log(myc,"fmi2SetString(...)");
    for (i = 0; i<nvr; i++) {
        if (vr[i]<FMI_STRING_VARS) {
            if (myc->string_vars[vr[i]])
                free(myc->string_vars[vr[i]]);
            myc->string_vars[vr[i]] = strdup(value[i]);
        } else
            return fmi2Error;
    }
    return fmi2OK;
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
