/*
 * PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "OSMPCNetworkProxyConfig.h"
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#if defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING)
#include <stdio.h>
#endif

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
typedef int SOCKET;
#endif
#ifndef INVALID_SOCKET
#define INVALID_SOCKET -1
#endif

#ifndef FMU_SHARED_OBJECT
#define FMI2_FUNCTION_PREFIX OSMPCNetworkProxy_
#endif
#include "fmi2Functions.h"

/*
 * Logging Control
 *
 * Logging is controlled via three definitions:
 *
 * - If PRIVATE_LOG_PATH is defined it gives the name of a file
 *   that is to be used as a private log file.
 * - If PUBLIC_LOGGING is defined then we will (also) log to
 *   the FMI logging facility where appropriate.
 * - If VERBOSE_FMI_LOGGING is defined then logging of basic
 *   FMI calls is enabled, which can get very verbose.
 */

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
#define FMI_BOOLEAN_DUMMY_IDX 0
#define FMI_BOOLEAN_SENDER_IDX 1
#define FMI_BOOLEAN_RECEIVER_IDX 2
#define FMI_BOOLEAN_ZMQ_IDX 3
#define FMI_BOOLEAN_PUBSUB_IDX 4
#define FMI_BOOLEAN_LOG_DATA_IDX 5
#define FMI_BOOLEAN_INPUT_VALID_IDX 6
#define FMI_BOOLEAN_INPUT_SENT_IDX 7
#define FMI_BOOLEAN_OUTPUT_RECEIVED_IDX 8
#define FMI_BOOLEAN_OUTPUT_VALID_IDX 9
#define FMI_BOOLEAN_LAST_IDX FMI_BOOLEAN_OUTPUT_VALID_IDX
#define FMI_BOOLEAN_VARS (FMI_BOOLEAN_LAST_IDX+1)

/* Integer Variables */
#define FMI_INTEGER_SENSORDATA_IN_BASELO_IDX 0
#define FMI_INTEGER_SENSORDATA_IN_BASEHI_IDX 1
#define FMI_INTEGER_SENSORDATA_IN_SIZE_IDX 2
#define FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX 3
#define FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX 4
#define FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX 5
#define FMI_INTEGER_LAST_IDX FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX
#define FMI_INTEGER_VARS (FMI_INTEGER_LAST_IDX+1)

/* Real Variables */
#define FMI_REAL_LAST_IDX 0
#define FMI_REAL_VARS (FMI_REAL_LAST_IDX+1)

/* String Variables */
#define FMI_STRING_ADDRESS_IDX 0
#define FMI_STRING_PORT_IDX 1
#define FMI_STRING_LAST_IDX FMI_STRING_PORT_IDX
#define FMI_STRING_VARS (FMI_STRING_LAST_IDX+1)

/* Callbacks without const */
typedef struct {
   fmi2CallbackLogger         logger;
   fmi2CallbackAllocateMemory allocateMemory;
   fmi2CallbackFreeMemory     freeMemory;
   fmi2StepFinished           stepFinished;
   fmi2ComponentEnvironment   componentEnvironment;
} fmi2CallbackFunctionsVar;

/* FMU Instance */
typedef struct OSMPCNetworkProxy {
    /* Members */
    char* instanceName;
    fmi2Type fmuType;
    char* fmuGUID;
    char* fmuResourceLocation;
    int visible;
    int loggingOn;
    size_t nCategories;
    char** loggingCategories;
    fmi2CallbackFunctionsVar functions;
    fmi2Boolean boolean_vars[FMI_BOOLEAN_VARS];
    fmi2Integer integer_vars[FMI_INTEGER_VARS];
    fmi2Real real_vars[FMI_REAL_VARS];
    char* string_vars[FMI_STRING_VARS];
    double last_time;

    /* Proxy Connections */
    #ifdef FMU_LISTEN
    SOCKET tcp_proxy_listen_socket;
    #endif
    SOCKET tcp_proxy_socket;

    /* Buffering */
    size_t output_buffer_size, prev_output_buffer_size;
    char *output_buffer_ptr, *prev_output_buffer_ptr;
} *OSMPCNetworkProxy;

/* Private File-based Logging just for Debugging */
#ifdef PRIVATE_LOG_PATH
static FILE* private_log_file = NULL;
#endif

void fmi_verbose_log_global(const char* format, ...)
{
#ifdef VERBOSE_FMI_LOGGING
#ifdef PRIVATE_LOG_PATH
    va_list ap;
    va_start(ap, format);
    if (private_log_file == NULL)
        private_log_file = fopen(PRIVATE_LOG_PATH,"a");
    if (private_log_file != NULL) {
        fprintf(private_log_file,"OSMPCNetworkProxy::Global: ");
        vfprintf(private_log_file, format, ap);
        fputc('\n',private_log_file);
        fflush(private_log_file);
    }
#endif
#endif
}

void internal_log(OSMPCNetworkProxy component,const char* category, const char* format, va_list arg)
{
#if defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING)
    char buffer[1024];
#ifdef _WIN32
    vsnprintf_s(buffer, 1024, _TRUNCATE, format, arg);
#else
    vsnprintf(buffer, 1024, format, arg);
    buffer[1023]='\0';
#endif
#ifdef PRIVATE_LOG_PATH
    if (private_log_file == NULL)
        private_log_file = fopen(PRIVATE_LOG_PATH,"a");
    if (private_log_file != NULL) {
        fprintf(private_log_file,"OSMPCNetworkProxy::%s<%p>: %s\n",component->instanceName,component,buffer);
        fflush(private_log_file);
    }
#endif
#ifdef PUBLIC_LOGGING
    if (component->loggingOn) {
        size_t i;
        int active = component->nCategories == 0;
        for (i=0;i<component->nCategories;i++) {
            if (0==strcmp(category,component->loggingCategories[i])) {
                active = 1;
                break;
            }
        }
        if (active)
            component->functions.logger(component->functions.componentEnvironment,component->instanceName,fmi2OK,category,buffer);
    }
#endif
#endif
}

void fmi_verbose_log(OSMPCNetworkProxy component,const char* format, ...)
{
#if defined(VERBOSE_FMI_LOGGING) && (defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING))
    va_list ap;
    va_start(ap, format);
    internal_log(component,"FMI",format,ap);
    va_end(ap);
#endif
}

/* Normal Logging */
void normal_log(OSMPCNetworkProxy component, const char* category, const char* format, ...) {
#if defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING)
    va_list ap;
    va_start(ap, format);
    internal_log(component,category,format,ap);
    va_end(ap);
#endif
}
