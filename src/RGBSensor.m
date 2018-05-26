classdef RGBSensor < handle
  
    properties
        handle;
        pose;
        image;
    end
    
    methods
        function obj = RGBSensor()
        end
        
        function init(obj, api, vrep)
            [res, obj.handle] = api.simxGetObjectHandle(vrep, 'rgbSensor', api.simx_opmode_oneshot_wait); vrchk(api, res);
        end
    end
    
end
