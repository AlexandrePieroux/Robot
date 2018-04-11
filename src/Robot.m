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
        function obj = Robot(width, length, range, maxlv, minav, maxav) 
            obj.width = width;
            obj.length = length;
            obj.hokuyo = Hokuyo(range);
            obj.wheels = Wheels(maxlv, minav, maxav);
        end
        
        function init(obj, api, vrep)
            [res, obj.handle] = api.simxGetObjectHandle(vrep, 'youBot_center', api.simx_opmode_oneshot_wait); vrchk(api, res);

            %Init the components
            obj.hokuyo.init(api, vrep, obj.handle);
            obj.wheels.init(api, vrep);
            
            % Display the red laser beams of active sensors
            api.simxSetIntegerSignal(vrep, 'displaylasers', 1, api.simx_opmode_oneshot); 
        end
        
        function drivePath(obj, path)
          % Mecanum wheels equations
          % maxav == max angular velocity ?
          frontLeftWheel = velocity * sin(-angle+pi/4) - obj.wheels.maxav
          frontRightWheel = velocity * cos(-angle+pi/4) + obj.wheels.maxav
          rearLeftWheel = velocity * cos(-angle+pi/4) - obj.wheels.maxav
          rearRightWheel = velocity * sin(-angle+pi/4) + obj.wheels.maxav
        end
    end
    
end

