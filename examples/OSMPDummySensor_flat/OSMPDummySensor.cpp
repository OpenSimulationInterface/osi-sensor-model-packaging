/*
 * PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
 *
 * (C) 2022 Persival GmbH Clemens Linnhoff
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "OSMPDummySensor.h"

#define NO_LIDAR_REFLECTIONS

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
#include <string>
#include <algorithm>
#include <cstdint>
#include <chrono>

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#endif

using namespace std;

#ifdef PRIVATE_LOG_PATH
ofstream COSMPDummySensor::private_log_file;
#endif

#ifdef _WIN32
std::string fileName = "C:/tmp/OSMPDummySensor_flatbuf_timing";
#else
std::string fileName = "/tmp/OSMPDummySensor_flatbuf_timing";
#endif


/*
 * Buffer Accessors
 */

void * decode_integer_to_pointer(fmi2Integer hi,fmi2Integer lo)
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

bool COSMPDummySensor::get_fmi_sensor_view_config(osi3::SensorViewConfiguration& data)
{
    //todo: sensor view config currently not implemented
    /*if (integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX] > 0) {
        void* buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASELO_IDX]);
        normal_log("OSMP","Got %08X %08X, reading from %p ...",integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASELO_IDX],buffer);
        data.ParseFromArray(buffer,integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX]);
        return true;
    } else {
        return false;
    }*/
    return false;
}

void COSMPDummySensor::set_fmi_sensor_view_config_request(const osi3::SensorViewConfiguration& data)
{
    //todo: sensor view config currently not implemented
    /*data.SerializeToString(currentConfigRequestBuffer);
    encode_pointer_to_integer(currentConfigRequestBuffer->data(),integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX]);
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX]=(fmi2Integer)currentConfigRequestBuffer->length();
    normal_log("OSMP","Providing %08X %08X, writing from %p ...",integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX],currentConfigRequestBuffer->data());
    swap(currentConfigRequestBuffer,lastConfigRequestBuffer);*/
}

void COSMPDummySensor::reset_fmi_sensor_view_config_request()
{
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX]=0;
}

const osi3::SensorView* COSMPDummySensor::get_fmi_sensor_view_in()
{
    if (integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX] > 0) {
        void* buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX]);
        normal_log("OSMP","Got %08X %08X, reading from %p ...",integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX],buffer);
        std::printf("Got %08X %08X, reading from %p ...\n",integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX],buffer);
        //data.ParseFromArray(buffer,integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX]);

        //Experimental: Read from file
        /*std::ifstream infile;
        infile.open("/tmp/data.bin", std::ios::binary | std::ios::in);
        infile.seekg(0, std::ios::end);
        int length = infile.tellg();
        infile.seekg(0, std::ios::beg);
        char* data = new char[length];
        infile.read(data, length);
        infile.close();*/

        auto sensor_view_in = flatbuffers::GetRoot<osi3::SensorView>(buffer);
        return sensor_view_in;
    } else {
        return nullptr;
    }
}

void COSMPDummySensor::set_fmi_sensor_data_out()
{
    encode_pointer_to_integer(currentOutputBuffer.data(),integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]);
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX]=(fmi2Integer)currentOutputBuffer.length();
    normal_log("OSMP","Providing %08X %08X, writing from %p ...",integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX],currentOutputBuffer.data());
    std::printf("Providing %08X %08X, writing from %p ...\n",integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX],currentOutputBuffer.data());
    swap(currentOutputBuffer,lastOutputBuffer);
}

void COSMPDummySensor::reset_fmi_sensor_data_out()
{
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX]=0;
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]=0;
}

void COSMPDummySensor::refresh_fmi_sensor_view_config_request()
{
    //todo: sensor view config currently not implemented
    /*osi3::SensorViewConfiguration config;
    if (get_fmi_sensor_view_config(config))
        set_fmi_sensor_view_config_request(config);
    else {
        config.Clear();
        config.mutable_version()->CopyFrom(osi3::InterfaceVersion::descriptor()->file()->options()->)GetExtension(osi3::current_interface_version));
        config.set_field_of_view_horizontal(3.14);
        config.set_field_of_view_vertical(3.14);
        config.set_range(fmi_nominal_range()*1.1);
        config.mutable_update_cycle_time()->set_seconds(0);
        config.mutable_update_cycle_time()->set_nanos(20000000);
        config.mutable_update_cycle_offset()->Clear();
        osi3::GenericSensorViewConfiguration* generic = config.add_generic_sensor_view_configuration();
        generic->set_field_of_view_horizontal(3.14);
        generic->set_field_of_view_vertical(3.14);
        set_fmi_sensor_view_config_request(config);
    }*/
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

    set_fmi_nominal_range(135.0);
    return fmi2OK;
}

fmi2Status COSMPDummySensor::doStart(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    DEBUGBREAK();

    return fmi2OK;
}

fmi2Status COSMPDummySensor::doEnterInitializationMode()
{
    DEBUGBREAK();

    return fmi2OK;
}

fmi2Status COSMPDummySensor::doExitInitializationMode()
{
    DEBUGBREAK();

    /*osi3::SensorViewConfiguration config;
    if (!get_fmi_sensor_view_config(config))
        normal_log("OSI","Received no valid SensorViewConfiguration from Simulation Environment, assuming everything checks out.");
    else {
        normal_log("OSI","Received SensorViewConfiguration for Sensor Id %llu",config.sensor_id()->)value());
        normal_log("OSI","SVC Ground Truth FoV Horizontal %f, FoV Vertical %f, Range %f",config.field_of_view_horizontal(),config.field_of_view_vertical(),config.range());
        normal_log("OSI","SVC Mounting Position: (%f, %f, %f)",config.mounting_position()->)position()->)x(),config.mounting_position()->)position()->)y(),config.mounting_position()->)position()->)z());
        normal_log("OSI","SVC Mounting Orientation: (%f, %f, %f)",config.mounting_position()->)orientation()->)roll(),config.mounting_position()->)orientation()->)pitch(),config.mounting_position()->)orientation()->)yaw());
    }*/

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

fmi2Status COSMPDummySensor::doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint)
{
    DEBUGBREAK();
    flatbuffers::FlatBufferBuilder builder(1024);
    double time = currentCommunicationPoint+communicationStepSize;
    auto startOSIDeserialize = std::chrono::duration_cast< std::chrono::microseconds >(std::chrono::system_clock::now().time_since_epoch());
    normal_log("OSI","Calculating Sensor at %f for %f (step size %f)",currentCommunicationPoint,time,communicationStepSize);
    const osi3::SensorView* sensor_view_in = get_fmi_sensor_view_in();
    auto stopOSIDeserialize = std::chrono::duration_cast< std::chrono::microseconds >(std::chrono::system_clock::now().time_since_epoch());

    if (sensor_view_in) {
        //// Lidar Detections
#ifndef NO_LIDAR_REFLECTIONS
        std::vector<flatbuffers::Offset<osi3::LidarDetection>> lidar_detection_vector;
        if (sensor_view_in->lidar_sensor_view()) {
            double azimuth_fov = 360.0;             // Azimuth angle FoV in °
            int rays_per_beam_horizontal = 6;       // horizontal super-sampling factor
            double beam_step_azimuth = 0.2;         // horizontal step-size per beam in degrees of VLP32 at 600 rpm (10 Hz) with VLP32's fixed firing_cycle of 55.296e^(-6) s
            double beam_step_elevation = 0.3;       // simplified equidistant beam spacing
            double max_emitted_signal_strength_in_dB = 10 * std::log10(0.5); // maximal emitted signal strength in dB
            int rays_per_layer = azimuth_fov/beam_step_azimuth*rays_per_beam_horizontal*0.8;
            double const_distance = 10.0;
            double speed_of_light = 299792458.0;
            size_t num_reflections = sensor_view_in->lidar_sensor_view()->Get(0)->reflection()->size();
            int layer_idx = -1;
            for (size_t reflection_idx = 0; reflection_idx < num_reflections; reflection_idx++) {
                if ((reflection_idx % rays_per_layer) == 0) layer_idx++;
                auto current_reflection = sensor_view_in->lidar_sensor_view()->Get(0)->reflection()->Get(reflection_idx);
                if (reflection_idx % 18 == 0) {     //18 times super-sampling
                    //todo: generate lidar detection
                    double distance = current_reflection->time_of_flight() * speed_of_light / 2;
                    double azimuth_deg = double(reflection_idx % rays_per_layer) * beam_step_azimuth;
                    double elevation_deg = layer_idx * beam_step_elevation - 5;     //start at -5° for simplification
                    auto detection_position = osi3::CreateSpherical3d(builder, distance, azimuth_deg*M_PI/180, elevation_deg*M_PI/180);
                    osi3::LidarDetectionBuilder lidar_detection_builder(builder);
                    lidar_detection_builder.add_position(detection_position);
                    lidar_detection_vector.push_back(lidar_detection_builder.Finish());
                }
            }
        }
        auto lidar_detection_vector_flatvector = builder.CreateVector(lidar_detection_vector);
        auto lidar_sensor_builder = osi3::LidarDetectionDataBuilder(builder);
        lidar_sensor_builder.add_detection(lidar_detection_vector_flatvector);
        auto lidar_sensor = lidar_sensor_builder.Finish();
        std::vector<flatbuffers::Offset<osi3::LidarDetectionData>> lidar_sensor_vector;
        lidar_sensor_vector.push_back(lidar_sensor);
        auto lidar_sensor_vector_flatvector = builder.CreateVector(lidar_sensor_vector);
        auto feature_data_builder = osi3::FeatureDataBuilder(builder);
        feature_data_builder.add_lidar_sensor(lidar_sensor_vector_flatvector);
        auto feature_data = feature_data_builder.Finish();
#endif
        //// Moving Objects
        double ego_x=0, ego_y=0, ego_z=0;
        const osi3::Identifier* ego_id = sensor_view_in->global_ground_truth()->host_vehicle_id();
        normal_log("OSI","Looking for EgoVehicle with ID: %llu",ego_id->value());
        size_t num_moving_obj = sensor_view_in->global_ground_truth()->moving_object()->size();
        for (size_t obj_idx = 0; obj_idx < num_moving_obj; obj_idx++) {
            auto current_obj = sensor_view_in->global_ground_truth()->moving_object()->Get(obj_idx);
            normal_log("OSI", "MovingObject with ID %llu is EgoVehicle: %d", current_obj->id()->value(), current_obj->id()->value() == ego_id->value());
            if (current_obj->id()->value() == ego_id->value()) {
                normal_log("OSI", "Found EgoVehicle with ID: %llu", current_obj->id()->value());
                ego_x = current_obj->base()->position()->x();
                ego_y = current_obj->base()->position()->y();
                ego_z = current_obj->base()->position()->z();
            }
        }
        normal_log("OSI","Current Ego Position: %f,%f,%f", ego_x, ego_y, ego_z);

        std::vector<flatbuffers::Offset<osi3::DetectedMovingObject>> detected_moving_object_vector;
        int moving_obj_counter=0;
        double actual_range = fmi_nominal_range()*1.1;

        for (size_t obj_idx = 0; obj_idx < num_moving_obj; obj_idx++) {
            auto current_obj = sensor_view_in->global_ground_truth()->moving_object()->Get(obj_idx);
            if (current_obj->id()->value() != ego_id->value()) {
                // NOTE: We currently do not take sensor mounting position into account,
                // i.e. sensor-relative coordinates are relative to center of bounding box
                // of ego vehicle currently.
                double trans_x = current_obj->base()->position()->x() - ego_x;
                double trans_y = current_obj->base()->position()->y() - ego_y;
                double trans_z = current_obj->base()->position()->z() - ego_z;
                double rel_x,rel_y,rel_z;
                rotatePoint(trans_x, trans_y, trans_z, current_obj->base()->orientation()->yaw(), current_obj->base()->orientation()->pitch(), current_obj->base()->orientation()->roll(), rel_x, rel_y, rel_z);
                double distance = sqrt(rel_x*rel_x + rel_y*rel_y + rel_z*rel_z);
                if ((distance <= actual_range) && (rel_x/distance > 0.866025)) {
                    std::vector<flatbuffers::Offset<osi3::Identifier>> obj_gt_id;
                    obj_gt_id.push_back(osi3::CreateIdentifier(builder, current_obj->id()->value()));
                    auto obj_gt_id_flatvector = builder.CreateVector(obj_gt_id);
                    auto obj_tracking_id = osi3::CreateIdentifier(builder, moving_obj_counter);
                    std::vector<flatbuffers::Offset<osi3::Identifier>> sensor_id;
                    sensor_id.push_back(osi3::CreateIdentifier(builder, sensor_view_in->sensor_id()->value()));
                    auto sensor_id_flatvector = builder.CreateVector(sensor_id);
                    osi3::DetectedItemHeaderBuilder detected_item_header_builder(builder);
                    detected_item_header_builder.add_ground_truth_id(obj_gt_id_flatvector);
                    detected_item_header_builder.add_tracking_id(obj_tracking_id);
                    detected_item_header_builder.add_existence_probability(cos((2.0*distance-actual_range)/actual_range));
                    detected_item_header_builder.add_measurement_state(osi3::DetectedItemHeader_::MeasurementState::MEASUREMENT_STATE_MEASURED);
                    detected_item_header_builder.add_sensor_id(sensor_id_flatvector);
                    auto obj_header = detected_item_header_builder.Finish();


                    auto obj_position = osi3::CreateVector3d(builder, rel_x, rel_y, rel_z);
                    auto obj_dimension = osi3::CreateDimension3d(builder, current_obj->base()->dimension()->length(), current_obj->base()->dimension()->width(), current_obj->base()->dimension()->height());
                    osi3::BaseMovingBuilder base_moving_builder(builder);
                    base_moving_builder.add_position(obj_position);
                    base_moving_builder.add_dimension(obj_dimension);
                    auto base_moving = base_moving_builder.Finish();

                    std::vector<flatbuffers::Offset<osi3::DetectedMovingObject_::CandidateMovingObject>> candidate_vector;
                    osi3::DetectedMovingObject_::CandidateMovingObjectBuilder candidate_builder(builder);
                    //candidate_builder.add_type(veh->type());  //todo: vehicle types are wrong in headers due to namespace conflict -> stationary and moving are confused
                    candidate_builder.add_probability(1);
                    candidate_vector.push_back(candidate_builder.Finish());
                    auto candidate_flatvector = builder.CreateVector(candidate_vector);

                    osi3::DetectedMovingObjectBuilder detected_moving_object_builder(builder);
                    detected_moving_object_builder.add_header(obj_header);
                    detected_moving_object_builder.add_base(base_moving);
                    detected_moving_object_builder.add_candidate(candidate_flatvector);
                    auto detected_moving_object = detected_moving_object_builder.Finish();
                    detected_moving_object_vector.push_back(detected_moving_object);

                    auto obj = reinterpret_cast<osi3::DetectedMovingObject *>(builder.GetCurrentBufferPointer() + builder.GetSize() - detected_moving_object.o);

                    normal_log("OSI","Output Vehicle %d[%llu] Probability %f Relative Position: %f,%f,%f (%f,%f,%f)",obj_idx,current_obj->id()->value(),obj->header()->existence_probability(),rel_x,rel_y,rel_z,obj->base()->position()->x(),obj->base()->position()->y(),obj->base()->position()->z());
                    moving_obj_counter++;
                } else {
                    normal_log("OSI", "Ignoring Vehicle %d[%llu] Outside Sensor Scope Relative Position: %f,%f,%f (%f,%f,%f)", moving_obj_counter, current_obj->id()->value(), current_obj->base()->position()->x() - ego_x, current_obj->base()->position()->y() - ego_y, current_obj->base()->position()->z() - ego_z, current_obj->base()->position()->x(), current_obj->base()->position()->y(), current_obj->base()->position()->z());
                }
            }
            else
            {
                normal_log("OSI", "Ignoring EGO Vehicle %d[%llu] Relative Position: %f,%f,%f (%f,%f,%f)", moving_obj_counter, current_obj->id()->value(), current_obj->base()->position()->x() - ego_x, current_obj->base()->position()->y() - ego_y, current_obj->base()->position()->z() - ego_z, current_obj->base()->position()->x(), current_obj->base()->position()->y(), current_obj->base()->position()->z());
            }
        }
        auto detected_moving_object_flatvector = builder.CreateVector(detected_moving_object_vector);

        //auto interface_version = osi3::CreateInterfaceVersion(builder,sensor_view_in->version()->version_major(), sensor_view_in->version()->version_minor(), sensor_view_in->version()->version_patch());    //todo: not implemented in source
        auto timestamp = osi3::CreateTimestamp(builder, (int64_t)floor(time), (int)((time - floor(time))*1000000000.0));

        /* Copy SensorView */
        //currentOut.add_sensor_view()->CopyFrom(sensor_view_in);   //todo: copying a serialized buffer into another buffer is not that easy in Flatbuffers

        osi3::SensorDataBuilder sensor_data_builder(builder);
        sensor_data_builder.add_timestamp(timestamp);
        //sensor_data_builder.add_version(interface_version);
        sensor_data_builder.add_moving_object(detected_moving_object_flatvector);
#ifndef NO_LIDAR_REFLECTIONS
        if (sensor_view_in->lidar_sensor_view()) {
            sensor_data_builder.add_feature_data(feature_data);
        }
#endif
        auto sensor_data = sensor_data_builder.Finish();

        auto startOSISerialize = std::chrono::duration_cast< std::chrono::microseconds >(std::chrono::system_clock::now().time_since_epoch());

        builder.Finish(sensor_data);
        auto uint8_buffer = builder.GetBufferPointer();
        auto size = builder.GetSize();
        std::string tmp_buffer(reinterpret_cast<char const*>(uint8_buffer), size);
        currentOutputBuffer = tmp_buffer;

        normal_log("OSI", "Mapped %d vehicles to output", moving_obj_counter);

        set_fmi_sensor_data_out();
        set_fmi_valid(true);
        set_fmi_count(moving_obj_counter);

        auto stopOSISerialize = std::chrono::duration_cast< std::chrono::microseconds >(std::chrono::system_clock::now().time_since_epoch());

        //// Performance logging
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
            logFile << "\t\t\"EventTypes\": [\"StartOSISerialize\", \"StopOSISerialize\", \"StartOSIDeserialize\", \"StopOSIDeserialize\"]," << std::endl;
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
            logFile << "\t\t\t\t\"ModelIdentity\": " << "\"OSMPDummySensor Flatbuf\"" << std::endl;
            /*logFile << "\t\t\t\t\"ModelIdentity\": " << "\"OSMPDummySensor Flatbuf\"" << "," << std::endl;
            logFile << "\t\t\t\t\"OsiVersion\": {" << std::endl;
            logFile << "\t\t\t\t\t\"version_major\": " << sensor_view_in->version()->version_major() << "," << std::endl;
            logFile << "\t\t\t\t\t\"version_minor\": " << sensor_view_in->version()->version_minor() << "," << std::endl;
            logFile << "\t\t\t\t\t\"version_patch\": " << sensor_view_in->version()->version_patch() << std::endl;*
            logFile << "\t\t\t\t}" << std::endl;*/
            logFile << "\t\t\t}," << std::endl;
            logFile << "\t\t\t\"OsiEvents\": [" << std::endl;
        } else {
            logFile.open (fileName, std::ios_base::app);
        }

        if(fileExists) {
            logFile << "," <<  std::endl;
        }
        size_t sensorDataSize = builder.GetSize();
        double osiSimTime = (double)sensor_view_in->global_ground_truth()->timestamp()->seconds() + (double)sensor_view_in->global_ground_truth()->timestamp()->nanos() * 0.000000001;

        logFile << "\t\t\t\t[" << "2" << ", " << std::setprecision(16) << (double)startOSIDeserialize.count() << ", " << osiSimTime << ", " << "0" << ", " << "2" << ", " << sizeof(*sensor_view_in) << "]," << std::endl;
        logFile << "\t\t\t\t[" << "3" << ", " << std::setprecision(16) << (double)stopOSIDeserialize.count() << ", " <<  osiSimTime << ", " << "0" << ", " << "2" << ", " << sizeof(*sensor_view_in) << "]," << std::endl;
        logFile << "\t\t\t\t[" << "0" << ", " << std::setprecision(16) << (double)startOSISerialize.count() << ", " << osiSimTime << ", " << "1" <<  ", " << "5" << ", " << sensorDataSize << "]," <<  std::endl;
        logFile << "\t\t\t\t[" << "1" << ", " << std::setprecision(16) << (double)stopOSISerialize.count() << ", " <<  osiSimTime << ", " << "1" <<  ", " << "5" << ", " << sensorDataSize << "]";
        logFile.close();

    } else {
        /* We have no valid input, so no valid output */
        normal_log("OSI","No valid input, therefore providing no valid output.");
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
    simulation_started(false)
{
    /*currentOutputBuffer=new string();
    lastOutputBuffer=new string();
    currentConfigRequestBuffer=new string();
    lastConfigRequestBuffer=new string();*/
    loggingCategories.clear();
    loggingCategories.insert("FMI");
    loggingCategories.insert("OSMP");
    loggingCategories.insert("OSI");
}

COSMPDummySensor::~COSMPDummySensor()
{
    /*delete currentOutputBuffer;
    delete lastOutputBuffer;
    delete currentConfigRequestBuffer;
    delete lastConfigRequestBuffer;*/
}

fmi2Status COSMPDummySensor::SetDebugLogging(fmi2Boolean theloggingOn, size_t nCategories, const fmi2String categories[])
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

fmi2Component COSMPDummySensor::Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID, fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions, fmi2Boolean visible, fmi2Boolean loggingOn)
{
    COSMPDummySensor* myc = new COSMPDummySensor(instanceName,fmuType,fmuGUID,fmuResourceLocation,functions,visible,loggingOn);

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

fmi2Status COSMPDummySensor::SetupExperiment(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
    fmi_verbose_log("fmi2SetupExperiment(%d,%g,%g,%d,%g)", toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    return doStart(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
}

fmi2Status COSMPDummySensor::EnterInitializationMode()
{
    fmi_verbose_log("fmi2EnterInitializationMode()");
    return doEnterInitializationMode();
}

fmi2Status COSMPDummySensor::ExitInitializationMode()
{
    fmi_verbose_log("fmi2ExitInitializationMode()");
    simulation_started = true;
    return doExitInitializationMode();
}

fmi2Status COSMPDummySensor::DoStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
    fmi_verbose_log("fmi2DoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    return doCalc(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
}

fmi2Status COSMPDummySensor::Terminate()
{
    fmi_verbose_log("fmi2Terminate()");
    return doTerm();
}

fmi2Status COSMPDummySensor::Reset()
{
    fmi_verbose_log("fmi2Reset()");

    doFree();
    simulation_started = false;
    return doInit();
}

void COSMPDummySensor::FreeInstance()
{
    fmi_verbose_log("fmi2FreeInstance()");
    doFree();
}

fmi2Status COSMPDummySensor::GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
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

fmi2Status COSMPDummySensor::GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
    fmi_verbose_log("fmi2GetInteger(...)");
    bool need_refresh = !simulation_started;
    for (size_t i = 0; i<nvr; i++) {
        if (vr[i]<FMI_INTEGER_VARS) {
            if (need_refresh && (vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX || vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX || vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX)) {
                refresh_fmi_sensor_view_config_request();
                need_refresh = false;
            }
            value[i] = integer_vars[vr[i]];
        } else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status COSMPDummySensor::GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
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

fmi2Status COSMPDummySensor::GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
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

fmi2Status COSMPDummySensor::SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
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

fmi2Status COSMPDummySensor::SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
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

fmi2Status COSMPDummySensor::SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
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

fmi2Status COSMPDummySensor::SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
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