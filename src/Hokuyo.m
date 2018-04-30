classdef Hokuyo < handle
  
    properties
        firstHandle;
        secondHandle;
        firstPose;
        secondPose;
        range;
        hits;
        voidPoints;
    end
    
    methods
        function obj = Hokuyo(range)
            obj.range = range;
        end
        
        function init(obj, api, vrep)
            [res, obj.firstHandle] = api.simxGetObjectHandle(vrep, 'fastHokuyo_sensor1', api.simx_opmode_oneshot_wait); vrchk(api, res);
            [res, obj.secondHandle] = api.simxGetObjectHandle(vrep, 'fastHokuyo_sensor2', api.simx_opmode_oneshot_wait); vrchk(api, res);
            
            api.simxSetIntegerSignal(vrep, 'handle_xy_sensor', 2, api.simx_opmode_oneshot);
        end
    end
    
end

