import json
import sys

emptyPath = {
 "name":"",
 "version":3,
 "snapshot":{
  "waypoints":[],
  "constraints":[],
  "targetDt":0.05
 },
 "params":{
  "waypoints":[],
  "constraints":[
    {"from":"first", "to":None, "data":{"type":"StopPoint", "props":{}}, "enabled":True},
    {"from":"last", "to":None, "data":{"type":"StopPoint", "props":{}}, "enabled":True},
    {"from":"first", "to":"last", "data":{"type":"KeepInRectangle", "props":{"x":{"exp":"0 m", "val":0.0}, "y":{"exp":"0 m", "val":0.0}, "w":{"exp":"16.541 m", "val":16.541}, "h":{"exp":"8.0692 m", "val":8.0692}}}, "enabled":False}],
  "targetDt":{
   "exp":"0.05 s",
   "val":0.05
  }
 },
 "trajectory":{
  "config":None,
  "sampleType":None,
  "waypoints":[],
  "samples":[],
  "splits":[]
 },
 "events":[]
}

def run(path: str, dir: str):
    print("Choreo Project: "+path)
    print("Destination Directory for Trajectory Files: "+dir)
    createPath("")

def createPath(startPose: str, endPose: str, route: str):
    print("Creating a path from "+startPose+" to "+endPose+" going through "+route)

run(sys.argv[1],sys.argv[2])