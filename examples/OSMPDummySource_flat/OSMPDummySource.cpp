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
    std::chrono::milliseconds start_source_calc = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());

    flatbuffers::FlatBufferBuilder builder(1024);
    double time = currentCommunicationPoint+communicationStepSize;

    normal_log("OSI","Calculating SensorView at %f for %f (step size %f)",currentCommunicationPoint,time,communicationStepSize);

    /* We act as GroundTruth Source */

    //// Lidar Reflections
    auto mounting_pos_position = osi3::CreateVector3d(builder, 1.5205, 0.0, 1.232);
    osi3::MountingPositionBuilder mounting_position_builder(builder);
    mounting_position_builder.add_position(mounting_pos_position);
    auto mounting_position = mounting_position_builder.Finish();

    osi3::LidarSensorViewConfigurationBuilder lidar_sensor_view_configuration_builder(builder);
    lidar_sensor_view_configuration_builder.add_field_of_view_horizontal(145.0 / 180 * M_PI);
    lidar_sensor_view_configuration_builder.add_field_of_view_vertical(3.2 / 180 * M_PI);
    lidar_sensor_view_configuration_builder.add_mounting_position(mounting_position);
    auto lidar_sensor_view_configuration = lidar_sensor_view_configuration_builder.Finish();

    int no_of_layers = 32;                  // the number of layers of every lidar front-end
    double azimuth_fov = 360.0;             // Azimuth angle FoV in °
    int rays_per_beam_vertical = 3;         // vertical super-sampling factor
    int rays_per_beam_horizontal = 6;       // horizontal super-sampling factor
    double beam_step_azimuth = 0.2;         // horizontal step-size per beam in degrees of VLP32 at 600 rpm (10 Hz) with VLP32's fixed firing_cycle of 55.296e^(-6) s
    double max_emitted_signal_strength_in_dB = 10 * std::log10(0.5); // maximal emitted signal strength in dB
    int rays_per_beam = rays_per_beam_vertical * rays_per_beam_horizontal;
    double const_distance = 10.0;
    double speed_of_light = 299792458.0;

    std::vector<flatbuffers::Offset<osi3::LidarSensorView_::Reflection>> reflection_vector;
    // Simulation of lower number of rays because of SET Level improvements
    for (int elevation_idx = 0; elevation_idx < no_of_layers * rays_per_beam_vertical; elevation_idx++) {
        for (int azimuth_idx = 0; azimuth_idx < azimuth_fov/beam_step_azimuth*rays_per_beam_horizontal*0.8; azimuth_idx++) {    //assume 80% of rays generate reflection
            double attenuation = 1.0;
            osi3::LidarSensorView_::ReflectionBuilder reflection_builder(builder);
            reflection_builder.add_time_of_flight(const_distance * 2.0 / speed_of_light);
            reflection_builder.add_signal_strength(max_emitted_signal_strength_in_dB + 10 * std::log10(attenuation) - 10 * std::log10(rays_per_beam));  // assuming equal distribution of beam power per ray
            auto ray = reflection_builder.Finish();
            reflection_vector.push_back(ray);
        }
    }

    auto reflection_flat_vector = builder.CreateVector(reflection_vector);
    osi3::LidarSensorViewBuilder lidar_sensor_view_builder(builder);
    lidar_sensor_view_builder.add_view_configuration(lidar_sensor_view_configuration);
    lidar_sensor_view_builder.add_reflection(reflection_flat_vector);
    auto lidar_sensor_view = lidar_sensor_view_builder.Finish();
    std::vector<flatbuffers::Offset<osi3::LidarSensorView>> lidar_sensor_view_vector;
    lidar_sensor_view_vector.push_back(lidar_sensor_view);
    auto lidar_sensor_view_flatvector = builder.CreateVector(lidar_sensor_view_vector);

    //// Moving Objects
    static double source_y_offsets[10] = { 3.0, 3.0, 3.0, 0.25, 0, -0.25, -3.0, -3.0, -3.0, -3.0 };
    static double source_x_offsets[10] = { 0.0, 40.0, 100.0, 100.0, 0.0, 150.0, 5.0, 45.0, 85.0, 125.0 };
    static double source_x_speeds[10] = { 29.0, 30.0, 31.0, 25.0, 26.0, 28.0, 20.0, 22.0, 22.5, 23.0 };
    static osi3::MovingObject_::VehicleClassification_::Type source_veh_types[10] = {
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_MEDIUM_CAR,
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_SMALL_CAR,
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_COMPACT_CAR,
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_DELIVERY_VAN,
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_LUXURY_CAR,
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_MEDIUM_CAR,
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_COMPACT_CAR,
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_SMALL_CAR,
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_MOTORBIKE,
            osi3::MovingObject_::VehicleClassification_::Type::TYPE_BUS };

    std::vector<flatbuffers::Offset<osi3::MovingObject>> moving_object_vector;
    for (unsigned int i=0;i<10;i++) {
        osi3::IdentifierBuilder id_builder(builder);
        id_builder.add_value(10+i);
        auto moving_obj_id = id_builder.Finish();

        osi3::MovingObject_::VehicleClassificationBuilder vehicle_classification_builder(builder);
        //vehicle_classification_builder.add_type(source_veh_types[i]);     //todo: vehicle classifications are wrong in headers due to namespace conflict -> confused with moving object type
        auto vehicle_classification = vehicle_classification_builder.Finish();

        auto bbcenter_to_rear = osi3::CreateVector3d(builder, 1.0, 0.0, -0.3);
        osi3::MovingObject_::VehicleAttributesBuilder vehicle_attributes_builder(builder);
        vehicle_attributes_builder.add_bbcenter_to_rear(bbcenter_to_rear);
        auto vehicle_attributes = vehicle_attributes_builder.Finish();

        auto dimension = osi3::CreateDimension3d(builder, 5.0, 2.0, 1.5);
        auto position = osi3::CreateVector3d(builder, source_x_offsets[i]+time*source_x_speeds[i], source_y_offsets[i]+sin(time/source_x_speeds[i])*0.25, 0.0);
        auto velocity = osi3::CreateVector3d(builder, source_x_speeds[i], cos(time/source_x_speeds[i])*0.25/source_x_speeds[i], 0.0);
        auto acceleration = osi3::CreateVector3d(builder, 0.0, -sin(time/source_x_speeds[i])*0.25/(source_x_speeds[i]*source_x_speeds[i]), 0.0);
        auto orientation = osi3::CreateOrientation3d(builder, 0.0, 0.0, 0.0);
        auto orientation_rate = osi3::CreateOrientation3d(builder, 0.0, 0.0, 0.0);
        osi3::BaseMovingBuilder base_moving_builder(builder);
        base_moving_builder.add_dimension(dimension);
        base_moving_builder.add_position(position);
        base_moving_builder.add_velocity(velocity);
        base_moving_builder.add_acceleration(acceleration);
        base_moving_builder.add_orientation(orientation);
        base_moving_builder.add_orientation_rate(orientation_rate);
        auto base_moving = base_moving_builder.Finish();

        osi3::MovingObjectBuilder moving_object_builder(builder);
        moving_object_builder.add_id(moving_obj_id);
        //moving_object_builder.add_type(osi3::MovingObject_::Type::TYPE_VEHICLE);  //todo: vehicle types are wrong in headers due to namespace conflict -> stationary and moving are confused
        moving_object_builder.add_vehicle_classification(vehicle_classification);
        moving_object_builder.add_vehicle_attributes(vehicle_attributes);
        moving_object_builder.add_base(base_moving);
        auto current_moving_object = moving_object_builder.Finish();
        moving_object_vector.push_back(current_moving_object);

        auto moving_object = reinterpret_cast<osi3::MovingObject *>(builder.GetCurrentBufferPointer() + builder.GetSize() - current_moving_object.o);
        normal_log("OSI","GT: Adding Vehicle %d[%llu] Absolute Position: %f,%f,%f Velocity (%f,%f,%f)",i,moving_object->id()->value(),moving_object->base()->position()->x(),moving_object->base()->position()->y(),moving_object->base()->position()->z(),moving_object->base()->velocity()->x(),moving_object->base()->velocity()->y(),moving_object->base()->velocity()->z());
    }
    auto moving_object_flatvector = builder.CreateVector(moving_object_vector);

    auto timestamp = osi3::CreateTimestamp(builder, (int64_t)floor(time), (int)((time - floor(time))*1000000000.0));
    osi3::IdentifierBuilder host_vehicle_id_builder(builder);
    host_vehicle_id_builder.add_value(14);
    auto host_vehicle_id = host_vehicle_id_builder.Finish();
    osi3::GroundTruthBuilder ground_truth_builder(builder);
    ground_truth_builder.add_timestamp(timestamp);
    ground_truth_builder.add_host_vehicle_id(host_vehicle_id);
    ground_truth_builder.add_moving_object(moving_object_flatvector);
    auto ground_truth = ground_truth_builder.Finish();

    auto sensor_id = osi3::CreateIdentifier(builder, 10000);

    osi3::SensorViewBuilder sensor_view_builder(builder);
    //sensor_view_builder.add_version(osi3::InterfaceVersion::descriptor()->file()->options().GetExtension(osi3::current_interface_version));   //todo: the used Protobuf FileOptions do not exist in Flatbuffers
    sensor_view_builder.add_sensor_id(sensor_id);
    sensor_view_builder.add_host_vehicle_id(host_vehicle_id);
    sensor_view_builder.add_timestamp(timestamp);
    sensor_view_builder.add_global_ground_truth(ground_truth);
    sensor_view_builder.add_lidar_sensor_view(lidar_sensor_view_flatvector);
    auto sensor_view = sensor_view_builder.Finish();

    std::chrono::milliseconds startOSISerialize = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());

    builder.Finish(sensor_view);
    auto uint8_buffer = builder.GetBufferPointer();
    auto size = builder.GetSize();
    std::string tmp_buffer(reinterpret_cast<char const*>(uint8_buffer), size);
    currentBuffer = tmp_buffer;

    set_fmi_sensor_view_out();
    set_fmi_valid(true);
    set_fmi_count((int)moving_object_vector.size());

    std::chrono::milliseconds stopOSISerialize = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());

    //// Performance logging
    auto sensor_view_in = flatbuffers::GetRoot<osi3::SensorView>(uint8_buffer);
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
        /*logFile << "\t\t\t\t\"ModelIdentity\": " << "\"OSMPDummySource Flatbuf\"" << "," << std::endl;
        logFile << "\t\t\t\t\"OsiVersion\": {" << std::endl;
        logFile << "\t\t\t\t\t\"version_major\": " << sensor_view_in->version()->version_major() << "," << std::endl;
        logFile << "\t\t\t\t\t\"version_minor\": " << sensor_view_in->version()->version_minor() << "," << std::endl;
        logFile << "\t\t\t\t\t\"version_patch\": " << sensor_view_in->version()->version_patch() << std::endl;
        logFile << "\t\t\t\t}" << std::endl;*/
        logFile << "\t\t\t}," << std::endl;
        logFile << "\t\t\t\"OsiEvents\": [" << std::endl;
    } else {
        logFile.open (fileName, std::ios_base::app);
    }

    if(fileExists) {
        logFile << "," <<  std::endl;
    }
    size_t sensorViewSize = builder.GetSize();
    double osiSimTime = (double)sensor_view_in->global_ground_truth()->timestamp()->seconds() + (double)sensor_view_in->global_ground_truth()->timestamp()->nanos() * 0.000000001;
    logFile << "\t\t\t\t[" << "0" << ", " << std::setprecision(13) << (double)start_source_calc.count()/1000.0 << ", " <<  osiSimTime << ", " << "0" <<  ", " << "5" << ", " << sensorViewSize << "]," <<  std::endl;
    logFile << "\t\t\t\t[" << "1" << ", " << std::setprecision(13) << (double)startOSISerialize.count()/1000.0 << ", " << osiSimTime << ", " << "0" <<  ", " << "5" << ", " << sensorViewSize << "]," <<  std::endl;
    logFile << "\t\t\t\t[" << "2" << ", " << std::setprecision(13) << (double)stopOSISerialize.count()/1000.0 << ", " <<  osiSimTime << ", " << "0" <<  ", " << "5" << ", " << sensorViewSize << "]";
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
