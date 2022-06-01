/*
 * PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "OSMPDummySource.h"

#define NO_LIDAR_REFLECTIONS
#define LIDAR_NUM_LAYERS 32
#define OBJECTS_MULT 1

/*
 * Debug Breaks
 *
 * If you define DEBUG_BREAKS the FMU will automatically break
 * into an attached Debugger on all major computation functions.
 * Note that the FMU is likely to break all environments if no
 * Debugger is actually attached when the breaks are triggered.
 */
#if defined(DEBUG_BREAKS) && !defined(NDEBUG)
#if defined(__has_builtin) && !defined(__ibmxl__)
#if __has_builtin(__builtin_debugtrap)
#define DEBUGBREAK() __builtin_debugtrap()
#elif __has_builtin(__debugbreak)
#define DEBUGBREAK() __debugbreak()
#endif
#endif
#if !defined(DEBUGBREAK)
#if defined(_MSC_VER) || defined(__INTEL_COMPILER)
#include <intrin.h>
#define DEBUGBREAK() __debugbreak()
#else
#include <signal.h>
#if defined(SIGTRAP)
#define DEBUGBREAK() raise(SIGTRAP)
#else
#define DEBUGBREAK() raise(SIGABRT)
#endif
#endif
#endif
#else
#define DEBUGBREAK()
#endif

#include <iostream>
//#include <string>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>  //included for windows compatibility
#include <memory>

using namespace std;

#ifdef PRIVATE_LOG_PATH
ofstream COSMPDummySource::private_log_file;
#endif

#ifdef _WIN32
std::string fileName = "C:/tmp/OSMPDummySource_flatbuf_timing";
#else
std::string fileName = "/tmp/OSMPDummySource_flatbuf_timing";
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

void COSMPDummySource::set_fmi_sensor_view_out()
{
    encode_pointer_to_integer(currentBuffer.data(), integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASELO_IDX]);
    integer_vars[FMI_INTEGER_SENSORVIEW_OUT_SIZE_IDX]=(fmi2Integer)currentBuffer.length();
    normal_log("OSMP","Providing %08X %08X, writing from %p ...",integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASELO_IDX],currentBuffer.data());
    std::printf("Providing %08X %08X, writing from %p ...\n",integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASELO_IDX],currentBuffer.data());
    swap(currentBuffer,lastBuffer);
}

void COSMPDummySource::reset_fmi_sensor_view_out()
{
    integer_vars[FMI_INTEGER_SENSORVIEW_OUT_SIZE_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASEHI_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASELO_IDX]=0;
}

/*
 * Actual Core Content
 */

fmi2Status COSMPDummySource::doInit()
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

fmi2Status COSMPDummySource::doStart(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    DEBUGBREAK();

    // initialize data structure
    // 1) sensor and ego IDs, timestamps
    auto sensor_id = std::unique_ptr<osi3::IdentifierT>(new osi3::IdentifierT());
    sensor_id->value = 1000;
    sensorViewOut.sensor_id = std::move(sensor_id);
    auto host_vehicle_id = std::unique_ptr<osi3::IdentifierT>(new osi3::IdentifierT());
    host_vehicle_id->value = 14;
    sensorViewOut.host_vehicle_id = std::move(host_vehicle_id);

    auto timestamp = std::unique_ptr<osi3::TimestampT>(new osi3::TimestampT());
    timestamp->seconds = 0;
    timestamp->nanos = 0;
    sensorViewOut.timestamp = std::move(timestamp);

    // ground truth
    auto currentGT = std::unique_ptr<osi3::GroundTruthT>(new osi3::GroundTruthT());
    sensorViewOut.global_ground_truth = std::move(currentGT);
    auto gt_host_vehicle_id = std::unique_ptr<osi3::IdentifierT>(new osi3::IdentifierT());
    gt_host_vehicle_id->value = 14;
    sensorViewOut.global_ground_truth->host_vehicle_id = std::move(gt_host_vehicle_id);

    auto gt_timestamp = std::unique_ptr<osi3::TimestampT>(new osi3::TimestampT());
    gt_timestamp->seconds = 0;
    gt_timestamp->nanos = 0;
    sensorViewOut.global_ground_truth->timestamp = std::move(gt_timestamp);

    // initialize Moving Objects
    for (unsigned int i = 0; i < (10 * OBJECTS_MULT); i++)
    {
        auto veh = std::unique_ptr<osi3::MovingObjectT>(new osi3::MovingObjectT());
        auto veh_id = std::unique_ptr<osi3::IdentifierT>(new osi3::IdentifierT());
        veh_id->value = 10 + i;
        veh->id = std::move(veh_id);
        //veh.type = osi3::MovingObject_::Type::TYPE_VEHICLE;     //todo: vehicle types are wrong in headers due to namespace conflict -> stationary and moving are confused
        //veh.vehicle_classification->type = source_veh_types[i]; //todo: vehicle classifications are wrong in headers due to namespace conflict -> confused with moving object type
        //veh.vehicle_classification->light_state->indicator_state = osi3::MovingObject_::VehicleClassification_::LightState_::IndicatorState::INDICATOR_STATE_OFF;
        //veh.vehicle_classification->light_state->brake_light_state = osi3::MovingObject_::VehicleClassification_::LightState_::BrakeLightState::BRAKE_LIGHT_STATE_OFF;
        auto vehicle_attributes = std::unique_ptr<osi3::MovingObject_::VehicleAttributesT>(new osi3::MovingObject_::VehicleAttributesT());
        auto bbcenter_to_rear = std::unique_ptr<osi3::Vector3dT>(new osi3::Vector3dT());
        bbcenter_to_rear->x = 1;
        bbcenter_to_rear->y = 0;
        bbcenter_to_rear->x = -0.3;
        vehicle_attributes->bbcenter_to_rear = std::move(bbcenter_to_rear);
        veh->vehicle_attributes = std::move(vehicle_attributes);
        auto base = std::unique_ptr<osi3::BaseMovingT>(new osi3::BaseMovingT());
        auto dimension = std::unique_ptr<osi3::Dimension3dT>(new osi3::Dimension3dT());
        dimension->height = 1.5;
        dimension->width = 2.0;
        dimension->length = 5.0;
        base->dimension = std::move(dimension);
        const auto x_speed = source_x_speeds[i % 10];
        auto base_position = std::unique_ptr<osi3::Vector3dT>(new osi3::Vector3dT());
        base_position->x = source_x_offsets[i % 10];
        base_position->y = source_y_offsets[i % 10];
        base_position->z = 0.0;
        base->position = std::move(base_position);
        auto velocity = std::unique_ptr<osi3::Vector3dT>(new osi3::Vector3dT());
        velocity->x = x_speed;
        velocity->y = 0.25 / x_speed;
        velocity->z = 0.0;
        base->velocity = std::move(velocity);
        auto acceleration = std::unique_ptr<osi3::Vector3dT>(new osi3::Vector3dT());
        acceleration->x = 0.0;
        acceleration->y = 0.0;
        acceleration->z = 0.0;
        base->acceleration = std::move(acceleration);
        auto orientation = std::unique_ptr<osi3::Orientation3dT>(new osi3::Orientation3dT());
        orientation->pitch = 0.0;
        orientation->roll = 0.0;
        orientation->yaw = 0.0;
        base->orientation = std::move(orientation);
        auto orientation_rate = std::unique_ptr<osi3::Orientation3dT>(new osi3::Orientation3dT());
        orientation_rate->pitch = 0.0;
        orientation_rate->roll = 0.0;
        orientation_rate->yaw = 0.0;
        base->orientation_rate = std::move(orientation_rate);
        veh->base = std::move(base);
        normal_log("OSI","GT: Adding Vehicle %d[%llu] Absolute Position: %f,%f,%f Velocity (%f,%f,%f)",i,veh->id->value,veh->base->position->x,veh->base->position->y,veh->base->position->z,veh->base->velocity->x,veh->base->velocity->y,veh->base->velocity->z);
        sensorViewOut.global_ground_truth->moving_object.push_back(std::move(veh));
    }

    return fmi2OK;
}

fmi2Status COSMPDummySource::doEnterInitializationMode()
{
    DEBUGBREAK();

    return fmi2OK;
}

fmi2Status COSMPDummySource::doExitInitializationMode()
{
    DEBUGBREAK();

    return fmi2OK;
}

void rotatePoint(double x, double y, double z,double yaw,double pitch,double roll,double &rx,double &ry,double &rz)
{
    double matrix[3][3];
    double cos_yaw = cos(yaw);
    double cos_pitch = cos(pitch);
    double cos_roll = cos(roll);
    double sin_yaw = sin(yaw);
    double sin_pitch = sin(pitch);
    double sin_roll = sin(roll);

    matrix[0][0] = cos_yaw*cos_pitch;  matrix[0][1]=cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll; matrix[0][2]=cos_yaw*sin_pitch*cos_roll + sin_yaw*sin_roll;
    matrix[1][0] = sin_yaw*cos_pitch;  matrix[1][1]=sin_yaw*sin_pitch*sin_roll + cos_yaw*cos_roll; matrix[1][2]=sin_yaw*sin_pitch*cos_roll - cos_yaw*sin_roll;
    matrix[2][0] = -sin_pitch;         matrix[2][1]=cos_pitch*sin_roll;                            matrix[2][2]=cos_pitch*cos_roll;

    rx = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z;
    ry = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z;
    rz = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z;
}

fmi2Status COSMPDummySource::doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint)
{
    DEBUGBREAK();
    auto start_source_calc = std::chrono::duration_cast< std::chrono::microseconds >(std::chrono::system_clock::now().time_since_epoch());

    flatbuffers::FlatBufferBuilder builder(1024);
    
    double time = currentCommunicationPoint+communicationStepSize;

    normal_log("OSI","Calculating SensorView at %f for %f (step size %f)",currentCommunicationPoint,time,communicationStepSize);

    // Update time
    sensorViewOut.timestamp->seconds = (long long int)floor(time);
    sensorViewOut.timestamp->nanos = (int)((time - floor(time))*1000000000.0);
    
    sensorViewOut.global_ground_truth->timestamp->seconds = (long long int)floor(time);
    sensorViewOut.global_ground_truth->timestamp->nanos = (int)((time - floor(time))*1000000000.0);

    // Lidar Reflections
#ifndef NO_LIDAR_REFLECTIONS
    auto lidar_sensor_view = std::unique_ptr<osi3::LidarSensorViewT>(new osi3::LidarSensorViewT());
    auto view_configuration = std::unique_ptr<osi3::LidarSensorViewConfigurationT>(new osi3::LidarSensorViewConfigurationT());
    view_configuration->field_of_view_horizontal = 145.0 / 180 * M_PI;
    view_configuration->field_of_view_vertical = 3.2 / 180 * M_PI;
    auto mounting_position = std::unique_ptr<osi3::MountingPositionT>(new osi3::MountingPositionT());
    auto position = std::unique_ptr<osi3::Vector3dT>(new osi3::Vector3dT());
    position->x = 1.5205; // from middle of rear axle, in m // 1.3705 + 0.15
    position->y = 0.0; // from middle of rear axle, in m
    position->z = 1.232;  // from middle of rear axle, in m // 0.382 + 0.85
    mounting_position->position = std::move(position);
    view_configuration->mounting_position = std::move(mounting_position);
    lidar_sensor_view->view_configuration = std::move(view_configuration);

    int no_of_layers = LIDAR_NUM_LAYERS;    // the number of layers of every lidar front-end
    double azimuth_fov = 360.0;             // Azimuth angle FoV in °
    int rays_per_beam_vertical = 3;         // vertical super-sampling factor
    int rays_per_beam_horizontal = 6;       // horizontal super-sampling factor
    double beam_step_azimuth = 0.2;         // horizontal step-size per beam in degrees of VLP32 at 600 rpm (10 Hz) with VLP32's fixed firing_cycle of 55.296e^(-6) s
    double max_emitted_signal_strength_in_dB = 10 * std::log10(0.5); // maximal emitted signal strength in dB
    int rays_per_beam = rays_per_beam_vertical * rays_per_beam_horizontal;
    double const_distance = 10.0;
    double speed_of_light = 299792458.0;

    // Simulation of lower number of rays because of SET Level improvements
    for (int elevation_idx = 0; elevation_idx < no_of_layers * rays_per_beam_vertical; elevation_idx++) {
        for (int azimuth_idx = 0; azimuth_idx < azimuth_fov/beam_step_azimuth*rays_per_beam_horizontal*0.8; azimuth_idx++) {    //assume 80% of rays generate reflection
            double attenuation = 1.0;
            auto ray = std::unique_ptr<osi3::LidarSensorView_::ReflectionT>(new osi3::LidarSensorView_::ReflectionT());
            ray->time_of_flight = const_distance * 2.0 / speed_of_light;
            ray->signal_strength = max_emitted_signal_strength_in_dB + 10 * std::log10(attenuation) - 10 * std::log10(rays_per_beam); // assuming equal distribution of beam power per ray
            lidar_sensor_view->reflection.push_back(std::move(ray));
        }
    }
    //currentOut.lidar_sensor_view.push_back(std::unique_ptr<osi3::LidarSensorViewT>(&lidar_sensor_view));
    currentOut.lidar_sensor_view.push_back(std::move(lidar_sensor_view));
#endif

    // Moving Objects
    for (unsigned int i = 0; i < (10 * OBJECTS_MULT); i++)
    {
        const auto x_speed = source_x_speeds[i % 10];
        auto base = sensorViewOut.global_ground_truth->moving_object[i]->base.get();

        base->position->x = source_x_offsets[i % 10] + time * x_speed;
        base->position->y = source_y_offsets[i % 10] + sin(time / x_speed) * 0.25;
        //base->position->z = 0.0;

        base->velocity->x = x_speed;
        base->velocity->y = cos(time / x_speed) * 0.25 / x_speed;
        //base->velocity->z = 0.0;

        //base->acceleration->x = 0.0;
        base->acceleration->y = -sin(time / x_speed) * 0.25 / (x_speed * x_speed);
        //base->acceleration->z = 0.0;

        normal_log("OSI","GT: updating Vehicle %d[%llu] Absolute Position: %f,%f,%f Velocity (%f,%f,%f)", i, 10 + i, base->position->x, base->position->y, base->position->z, base->velocity->x, base->velocity->y, base->velocity->z);
    }
    
    std::cout << "DummySource time doCalc(): " << time << std::endl;

    //check if file exists
    std::ifstream f(fileName.c_str());
    bool fileExists = f.is_open();
    f.close();

    std::ofstream logFile;
    if(!fileExists) {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream time_string;
        time_string << std::put_time(std::localtime(&in_time_t), "_%H-%M-%S.json");
        fileName += time_string.str();
        logFile.open (fileName, std::ios_base::app);
        logFile << "{" << std::endl;
        logFile << "\t\"Header\": {" << std::endl;
        logFile << "\t\t\"OsiMessages\": [\"osi3::SensorView\", \"osi3::SensorData\"]," << std::endl;
        logFile << "\t\t\"EventFields\": [\"EventId\", \"GlobalTime\", \"SimulationTime\", \"MessageId\", \"SizeValueReference\", \"MessageSize\"]," << std::endl;
        logFile << "\t\t\"EventTypes\": [\"StartSourceCalc\", \"StartOSISerialize\", \"StopOSISerialize\", \"StartOSIDeserialize\", \"StopOSIDeserialize\"]," << std::endl;
        logFile << "\t\t\"FormatVersion\": {" << std::endl;
        logFile << "\t\t\t\"Major\": 1," << std::endl;
        logFile << "\t\t\t\"Minor\": 0," << std::endl;
        logFile << "\t\t\t\"Patch\": 0," << std::endl;
        logFile << "\t\t\t\"PreRelease\": \"beta\"" << std::endl;
        logFile << "\t\t}" << std::endl;
        logFile << "\t}," << std::endl;
        logFile << "\t\"Data\": [" << std::endl;
        logFile << "\t\t{" << std::endl;
        logFile << "\t\t\t\"Instance\": {" << std::endl;
        logFile << "\t\t\t\t\"ModelIdentity\": " << "\"OSMPDummySource Flatbuf\"" << std::endl;
        /*logFile << "\t\t\t\t\"ModelIdentity\": " << "\"OSMPDummySource Protobuf\"" << "," << std::endl;
        logFile << "\t\t\t\t\"OsiVersion\": {" << std::endl;
        logFile << "\t\t\t\t\t\"version_major\": "<< currentOut.version->version_major() << "," << std::endl;
        logFile << "\t\t\t\t\t\"version_minor\": "<< currentOut.version->version_minor() << "," << std::endl;
        logFile << "\t\t\t\t\t\"version_patch\": "<< currentOut.version->version_patch() << std::endl;
        logFile << "\t\t\t\t}" << std::endl;*/
        logFile << "\t\t\t}," << std::endl;
        logFile << "\t\t\t\"OsiEvents\": [" << std::endl;
    } else {
        logFile.open (fileName, std::ios_base::app);
    }

    float osiSimTime = sensorViewOut.global_ground_truth->timestamp->seconds + (float)sensorViewOut.global_ground_truth->timestamp->nanos * 0.000000001;

    auto startOSISerialize = std::chrono::duration_cast< std::chrono::microseconds >(std::chrono::system_clock::now().time_since_epoch());

    builder.Finish(osi3::SensorView::Pack(builder, &sensorViewOut));
    auto uint8_buffer = builder.GetBufferPointer();
    auto size = builder.GetSize();
    std::string tmp_buffer(reinterpret_cast<char const*>(uint8_buffer), size);
    currentBuffer = tmp_buffer;

    set_fmi_sensor_view_out();
    set_fmi_valid(true);
    set_fmi_count((int)sensorViewOut.global_ground_truth->moving_object.size());

    auto stopOSISerialize = std::chrono::duration_cast< std::chrono::microseconds >(std::chrono::system_clock::now().time_since_epoch());

    if(fileExists) {
        logFile << "," <<  std::endl;
    }
    int sensorViewSize = size;
    logFile << "\t\t\t\t[" << "0" << ", " << std::setprecision(16) << (double)start_source_calc.count() << ", " <<  osiSimTime << ", " << "0" <<  ", " << "5" << ", " << sensorViewSize << "]," <<  std::endl;
    logFile << "\t\t\t\t[" << "1" << ", " << std::setprecision(16) << (double)startOSISerialize.count() << ", " << osiSimTime << ", " << "0" <<  ", " << "5" << ", " << sensorViewSize << "]," <<  std::endl;
    logFile << "\t\t\t\t[" << "2" << ", " << std::setprecision(16) << (double)stopOSISerialize.count() << ", " <<  osiSimTime << ", " << "0" <<  ", " << "5" << ", " << sensorViewSize << "]";
    logFile.close();

    return fmi2OK;
}

fmi2Status COSMPDummySource::doTerm()
{
    DEBUGBREAK();
    return fmi2OK;
}

void COSMPDummySource::doFree()
{
    DEBUGBREAK();
}

/*
 * Generic C++ Wrapper Code
 */

COSMPDummySource::COSMPDummySource(fmi2String theinstanceName, fmi2Type thefmuType, fmi2String thefmuGUID, fmi2String thefmuResourceLocation, const fmi2CallbackFunctions* thefunctions, fmi2Boolean thevisible, fmi2Boolean theloggingOn)
    : instanceName(theinstanceName),
    fmuType(thefmuType),
    fmuGUID(thefmuGUID),
    fmuResourceLocation(thefmuResourceLocation),
    functions(*thefunctions),
    visible(!!thevisible),
    loggingOn(!!theloggingOn)
{
    //currentBuffer = new string();
    //lastBuffer = new string();
    loggingCategories.clear();
    loggingCategories.insert("FMI");
    loggingCategories.insert("OSMP");
    loggingCategories.insert("OSI");
}

COSMPDummySource::~COSMPDummySource()
{
    //delete currentBuffer;
    //delete lastBuffer;
}

fmi2Status COSMPDummySource::SetDebugLogging(fmi2Boolean theloggingOn, size_t nCategories, const fmi2String categories[])
{
    fmi_verbose_log("fmi2SetDebugLogging(%s)", theloggingOn ? "true" : "false");
    loggingOn = theloggingOn ? true : false;
    if (categories && (nCategories > 0)) {
        loggingCategories.clear();
        for (size_t i=0;i<nCategories;i++) {
            if (0==strcmp(categories[i],"FMI"))
                loggingCategories.insert("FMI");
            else if (0==strcmp(categories[i],"OSMP"))
                loggingCategories.insert("OSMP");
            else if (0==strcmp(categories[i],"OSI"))
                loggingCategories.insert("OSI");
        }
    } else {
        loggingCategories.clear();
        loggingCategories.insert("FMI");
        loggingCategories.insert("OSMP");
        loggingCategories.insert("OSI");
    }
    return fmi2OK;
}

fmi2Component COSMPDummySource::Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID, fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions, fmi2Boolean visible, fmi2Boolean loggingOn)
{
    COSMPDummySource* myc = new COSMPDummySource(instanceName,fmuType,fmuGUID,fmuResourceLocation,functions,visible,loggingOn);

    if (myc == NULL) {
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
    }
    else {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = %p",
            instanceName, fmuType, fmuGUID,
            (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
            "FUNCTIONS", visible, loggingOn, myc);
        return (fmi2Component)myc;
    }
}

fmi2Status COSMPDummySource::SetupExperiment(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    fmi_verbose_log("fmi2SetupExperiment(%d,%g,%g,%d,%g)", toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    return doStart(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
}

fmi2Status COSMPDummySource::EnterInitializationMode()
{
    fmi_verbose_log("fmi2EnterInitializationMode()");
    return doEnterInitializationMode();
}

fmi2Status COSMPDummySource::ExitInitializationMode()
{
    fmi_verbose_log("fmi2ExitInitializationMode()");
    return doExitInitializationMode();
}

fmi2Status COSMPDummySource::DoStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
    fmi_verbose_log("fmi2DoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    return doCalc(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
}

fmi2Status COSMPDummySource::Terminate()
{
    fmi_verbose_log("fmi2Terminate()");
    return doTerm();
}

fmi2Status COSMPDummySource::Reset()
{
    fmi_verbose_log("fmi2Reset()");

    doFree();
    return doInit();
}

void COSMPDummySource::FreeInstance()
{
    fmi_verbose_log("fmi2FreeInstance()");
    doFree();
}

fmi2Status COSMPDummySource::GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
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

fmi2Status COSMPDummySource::GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
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

fmi2Status COSMPDummySource::GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
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

fmi2Status COSMPDummySource::GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
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

fmi2Status COSMPDummySource::SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
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

fmi2Status COSMPDummySource::SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
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

fmi2Status COSMPDummySource::SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
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

fmi2Status COSMPDummySource::SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
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
        COSMPDummySource* myc = (COSMPDummySource*)c;
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
        return COSMPDummySource::Instantiate(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
    }

    FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c,
        fmi2Boolean toleranceDefined,
        fmi2Real tolerance,
        fmi2Real startTime,
        fmi2Boolean stopTimeDefined,
        fmi2Real stopTime)
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->SetupExperiment(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    }

    FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c)
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->EnterInitializationMode();
    }

    FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c)
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->ExitInitializationMode();
    }

    FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,
        fmi2Real currentCommunicationPoint,
        fmi2Real communicationStepSize,
        fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->DoStep(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    }

    FMI2_Export fmi2Status fmi2Terminate(fmi2Component c)
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        std::ofstream logFile;
        logFile.open(fileName, std::ios_base::app);
        logFile << std::endl << "\t\t\t]" << std::endl;
        logFile << "\t\t}" << std::endl;
        logFile << "\t]" << std::endl;
        logFile << "}" << std::endl;
        logFile.close();

        return myc->Terminate();
    }

    FMI2_Export fmi2Status fmi2Reset(fmi2Component c)
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->Reset();
    }

    FMI2_Export void fmi2FreeInstance(fmi2Component c)
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        myc->FreeInstance();
        delete myc;
    }

    /*
     * Data Exchange Functions
     */
    FMI2_Export fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->GetReal(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->GetInteger(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->GetBoolean(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->GetString(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->SetReal(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->SetInteger(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
        return myc->SetBoolean(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
    {
        COSMPDummySource* myc = (COSMPDummySource*)c;
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
