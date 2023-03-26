# API End Points #

### API Server URL ###
https://www.command-central.com
- It is currently in beta and available for free use to open source developers and researchers;
- There are no currently known limitations as to the number of workers, workstations and routings you can enable and it is designed to handle a high workload.

### Response object ###
All endpoints return the same JSON object.  Depending on the request, some fields may just be their default value.

`
    {
    status: '0', name: '',
    tools: [''], program: [''],
    poNum: '', partNumber: '', partSerialNumber: '',
    sourceName: '',
    sourcePosX: '-9999', sourcePosY: '-9999', sourcePosZ: '-9999',
    sourceOrientX: '-9999', sourceOrientY: '-9999', sourceOrientZ: '-9999', sourceOrientW: '-9999',
    destName: '',
    destPosX: '-9999', destPosY: '-9999', destPosZ: '-9999',
    destOrientX: '-9999', destOrientY: '-9999', destOrientZ: '-9999', destOrientW: '-9999'
    }
`

### /workerGetNextJob ###
curl: https://www.command-central.com/workerGetNextJob?key=your-robot-id-goes-here
- ROS Service Request 'function': NEXTJOB
- Description: This curl will check if there is an available job for the robot corresponding to the robot name.  If there is it will return the source and destination name and coordinates

### /workerActivateJob ###
curl:  https://www.command-central.com/workerActivateJob?name=your-robot-id-goes-here
- ROS Service Request 'function': ACTIVATEJOB
- Description: This curl will acknowledge the job is received and that the robot has started the job

### /workerLocation ###
curl:  https://www.command-central.com/workerLocation?name=your-robot-id-goes-here&location=source-or-desination-goes-here
- ROS Service Request 'function': MOVEWORKER
- Description: This curl will update the location of the worker to be either at the source or destination workstation
- Implementation: in the `location` field `source` is used to acknowledge worker is at the source workstation, `destination` is used to acknowlegde the worker is at the destination station

### /workerTakePart ###
curl:  https://www.command-central.com/workerTakePart?name=your-id-name-goes-here
- ROS Service Request 'function': TAKEPART
- Description: This curl will acknowledge the worker has retrieved the part from the `source` workstation

### /workerLoadPart ###
curl:  https://www.command-central.com/workerLoadPart?name=your-id-name-goes-here
- ROS Service Request 'function': LOADPART
- Description: This curl will acknowledge the worker has loaded the part into the `destination` workstation

### /workerArchiveJob ###
curl:  https://www.command-central.com/workerArchiveJob?name=your-id-name-goes-here
- ROS Service Request 'function': ARCHIVEJOB
- Description: This curl will clear the worker job and signal it is ready for the next job (call NEXTJOB)


# API Error Codes #

## Worker Route API: workerRoute.js ##

### /workerGetNextJob ###
- 10001: There is a problem with communication check message formatting, normally a worker id problem
- 10002: There are no active routings
- 10003: There are no tasks to do
- 10004: Worker already has a job in progress
- 10005: Worker does not exist

### /workerActivateJob ###
- 11001: There is a problem with communication check message formatting, normally a worker id problem
- 11002: Worker is not ACTIVE
- 11003: Problem activating job in worker
- 11004: General error activating job
- 11005: No job to activate

### /workerLocation ###
- 12001: There is a problem with communication check message formatting, normally a worker id problem
- 12002: Error in move logic
- 12003: General error updating location

### /workerTakePart ###
- 13001: There is a problem with communication check message formatting, normally a worker id problem
- 13002: Problem updating worker in database
- 13003: Distance was not 0 or station was already empty
- 13004: Error with logic updating worker
- 13005: Error updating when taking part

### /workerLoadWorkstation ###
- 14001: There is a problem with communication check message formatting, normally a worker id problem
- 14002: Error loading buffer station
- 14003: Error updating the workstation in the database
- 14004: There is already something in the workstation
- 14005: Error with logic of loading workstation
- 14006: Error not enough funds to perform work left in the routing
- 14007: Could not find worker owner
- 14008: Error processing payment to worker owner

### /workerArchiveJob ###
- 15001: There is a problem with communication check message formatting, normally a worker id problem
- 15002: Error with logic of archiving job
- 15003: The worker needs a job to be activated and complete before archiving
- 15004: The worker still has a job and needs to complete before archiving

### /workerRunTimeStatus ###
- 16001: Worker does not exist
- 16002: RunTime parameter is missing
- 16003: GridLocation parameter is missing
- 16004: General error updating run time status

## ROS Custom Service Documentation ##
The ROS service which implements the endpoints in ROS has the following server definition:
- name: [WebComm.srv](https://github.com/mukmalone/AdpativeGoalManagement/blob/master/examples/mir_robot/mir_agm/srv/WebComm.srv)
- ROS service: [web_comm_server_node.cpp](https://github.com/mukmalone/AdpativeGoalManagement/blob/master/examples/mir_robot/mir_agm/src/web_comm_server_node.cpp)
- example of using service to work through a sequence: [agm_worker_node.cpp](https://github.com/mukmalone/AdpativeGoalManagement/blob/master/examples/mir_robot/mir_agm/src/agm_worker_node.cpp)
- Request:
> - string function //name of function you want to call(defined below)
> - string key //_id of the robot making the request
> - string location //used when moving to source or destination locations
---
- Response:
> - uint32 status //status of the request.  PASS=1 Error Code is != 1
> - string name  //returns the name of the robot, if there is an error this will be the error description
> - string tools // returns the list of tools associated with the destination station
> - string program // returns the list of programs associated with the destination station
> - string poNum // returns the PO number of the part being run
> - string partNumber // returns the part number of the part being run
> - string partSerialNumber: // returns the serial number of the part being run
> - string sourceName //Source Workstation name
> - float64 sourcePosX //Source workstation parking X location
> - float64 sourcePosY //Source workstation parking Y location
> - float64 sourcePosZ //Source workstation parking Z location
> - float64 sourceOrientX //Source workstation parking X angle
> - float64 sourceOrientY //Source workstation parking Y angle
> - float64 sourceOrientZ //Source workstation parking Z angle
> - float64 sourceOrientW //Source workstation parking angle
> - string destinationName //Destination workstation name
> - float64 destPosX //Destination workstation parking X location
> - float64 destPosY //Destination workstation parking Y location
> - float64 destPosZ //Destination workstation parking Z location
> - float64 destOrientX //Destination workstation parking X angle
> - float64 destOrientY //Destination workstation parking Y angle
> - float64 destOrientZ //Destination workstation parking Z angle
> - float64 destOrientW //Destination workstation parking angle
