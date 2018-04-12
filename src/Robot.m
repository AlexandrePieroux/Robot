classdef Robot < handle
    
    properties
        handle;
        pose;
        width;
        length;
        hokuyo;
        wheels;
    end
    
    methods
        function obj = Robot(range, maxlv, minav, maxav) 
            obj.hokuyo = Hokuyo(range);
            obj.wheels = Wheels(maxlv, minav, maxav);
        end
        
        function init(obj, api, vrep)
            [res, obj.handle] = api.simxGetObjectHandle(vrep, 'youBot_center', api.simx_opmode_oneshot_wait); vrchk(api, res);

            % Get the width of the youbot
            [res, vehicleControl] = api.simxGetObjectHandle(vrep, 'vehicleControl', api.simx_opmode_oneshot_wait); vrchk(api, res);
            [res, minX] = api.simxGetObjectFloatParameter(vrep, vehicleControl, 15, api.simx_opmode_oneshot_wait); vrchk(api, res);
            [res, maxX] = api.simxGetObjectFloatParameter(vrep, vehicleControl, 18, api.simx_opmode_oneshot_wait); vrchk(api, res);
            
            obj.width = maxX - minX;

            % Get the height of the youbot
            [res, minY] = api.simxGetObjectFloatParameter(vrep, vehicleControl, 16, api.simx_opmode_oneshot_wait); vrchk(api, res);
            [res, maxY] = api.simxGetObjectFloatParameter(vrep, vehicleControl, 19, api.simx_opmode_oneshot_wait); vrchk(api, res);
            
            obj.height = maxY - minY;

            %Init the components
            obj.hokuyo.init(api, vrep, obj.handle);
            obj.wheels.init(api, vrep);
            
            % Display the red laser beams of active sensors
            api.simxSetIntegerSignal(vrep, 'displaylasers', 1, api.simx_opmode_oneshot); 
        end
        
    end
    
end

