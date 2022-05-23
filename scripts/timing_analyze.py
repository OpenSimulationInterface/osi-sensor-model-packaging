import json
import numpy as np
import os

############### FLATB BUILDER ################
json_directory = '/home/martin/development/flatbuf_test_data/objects_10/flatb_builder'
#json_directory = '/home/martin/development/flatbuf_test_data/objects_200/flatb_builder'
#json_directory = '/home/martin/development/flatbuf_test_data/objects_500/flatb_builder'
#json_directory = '/home/martin/development/flatbuf_test_data/lidar_830k/flatb_builder'

############### FLATB OBJECT API #############
#json_directory = '/home/martin/development/flatbuf_test_data/objects_10/flatb_object_api'
#json_directory = '/home/martin/development/flatbuf_test_data/objects_200/flatb_object_api'
#json_directory = '/home/martin/development/flatbuf_test_data/objects_500/flatb_object_api'
#json_directory = '/home/martin/development/flatbuf_test_data/lidar_830k/flatb_object_api'

############### PROTOB #######################
#json_directory = '/home/martin/development/flatbuf_test_data/objects_10/protob'
#json_directory = '/home/martin/development/flatbuf_test_data/objects_200/protob'
#json_directory = '/home/martin/development/flatbuf_test_data/objects_500/protob'
#json_directory = '/home/martin/development/flatbuf_test_data/lidar_830k/protob'

def analyze_file(timing_json_path):
    # Opening JSON file
    f = open(timing_json_path)
    
    # returns JSON object as 
    # a dictionary
    data = json.load(f)

    if 'OSMPDummySensor' in data['Data'][0]['Instance']['ModelIdentity']:
        startEvent = 2
        nextEvent = 2
        eventCount = int(4)
        timeStrings = ['Model deserialize', 'Model calculation', 'Model serialize', 'Total']
    else:
        startEvent = 0
        nextEvent = 0
        eventCount = int(3)
        timeStrings = ['Source generate', 'Source serialize', 'Total']

    numTimeSteps = len(data['Data'][0]['OsiEvents']) // eventCount
    timingMat = np.zeros((numTimeSteps, eventCount - 1))
    matIdx = 0
    prevTimeStamp = 0

    # Iterating through the json
    # list
    for event in data['Data'][0]['OsiEvents']:
        if event[0] != nextEvent:
            print('Error in log event sequence')
            break
        curTimeStamp = event[1]
        if event[0] != startEvent:
            timingMat[np.unravel_index(matIdx, timingMat.shape)] = curTimeStamp - prevTimeStamp
            matIdx += 1

        nextEvent += 1
        nextEvent = nextEvent % eventCount
        prevTimeStamp = curTimeStamp

    timingAvg = timingMat.sum(axis = 0) / numTimeSteps
    timingAvg = np.append(timingAvg, timingAvg.sum())
    timingMin = timingMat.min(axis = 0)
    timingMax = timingMat.max(axis = 0)
    print(timeStrings)
    print(timingAvg)

    # Closing file
    f.close()

def main():
    json_files=os.listdir(json_directory)
    for json_file in json_files:
        json_path = os.path.join(json_directory, json_file)
        analyze_file(json_path)

if __name__ == "__main__":
    main()