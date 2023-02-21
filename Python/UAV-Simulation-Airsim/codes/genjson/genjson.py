import json
import os
import shutil

def fileWrite(fName, num):
    dict_data = {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",

        "Vehicles": {

        },
        
        "Recording": {
            "RecordOnMove": False,
            "RecordInterval": 0.05,
            "Folder": "",
            "Enabled": False,
            "Cameras": []
        },
        
        "OriginGeopoint": {
            "Latitude": 47.641468,
            "Longitude": -122.140165,
            "Altitude": 122
        }
    }

    # Add drone into json
    for ii in range(1, num+1):
        strIn = "Drone{}".format(ii)
        xVal = ii * 10
        dict_data["Vehicles"][strIn] = {"VehicleType": "SimpleFlight","X": xVal, "Y": 0, "Z": -2}

    # convert dictionary to json string
    json_data = json.dumps(dict_data)

    # write json string to a file
    with open(fName, 'w') as outfile:
        outfile.write(json_data)


def fileCopy(fName):
    # get absolute path
    p = os.path.abspath('')

    # get documents directory  
    # documents_dir = os.path.expanduser('~/Documents') # => LINUX
    documents_dir = os.path.join(os.environ['USERPROFILE'], 'Documents')

    # Get a list of files and directories in the Documents directory
    contents = os.listdir(documents_dir)

    # Print the contents
    for item in contents:
        if item == "AirSim":
            absolute_path = os.path.join(documents_dir, item)
            srcJSON = os.path.join(p.__str__(), fName)
            dstJSON = os.path.join(absolute_path, fName)
            # Copy the file
            shutil.copy(srcJSON, dstJSON)    
            # remove json file from this path
            os.remove(srcJSON)            
        else:
            pass

def runGenJson(sNum):
    fName = "settings.json"
    fileWrite(fName, sNum)
    fileCopy(fName)
    
