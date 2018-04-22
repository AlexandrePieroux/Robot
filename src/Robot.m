classdef Robot < handle
    
    properties
        handle;
        pose;
        se2;
        width;
        length;
        hokuyo;
        wheels;
        
        % Kinematic information
        Ja;
        JaInv;
    end
    
    methods
        function obj = Robot(range, maxlv, minav, maxav) 
            obj.hokuyo = Hokuyo(range);
            obj.wheels = Wheels(maxlv, minav, maxav);
        end
        
        function init(obj, api, vrep)
            [res, obj.handle] = api.simxGetObjectHandle(vrep, 'youBot_center', api.simx_opmode_oneshot_wait); vrchk(api, res);

            % Get the handle of the vehicle control
            [res, vehicleControl] = api.simxGetObjectHandle(vrep, 'vehicleControl', api.simx_opmode_oneshot_wait); vrchk(api, res);
            
            % Get the width of the youbot
            [res, minX] = api.simxGetObjectFloatParameter(vrep, vehicleControl, 15, api.simx_opmode_oneshot_wait); vrchk(api, res);
            [res, maxX] = api.simxGetObjectFloatParameter(vrep, vehicleControl, 18, api.simx_opmode_oneshot_wait); vrchk(api, res);
            
            obj.width = maxX - minX;

            % Get the height of the youbot
            [res, minY] = api.simxGetObjectFloatParameter(vrep, vehicleControl, 16, api.simx_opmode_oneshot_wait); vrchk(api, res);
            [res, maxY] = api.simxGetObjectFloatParameter(vrep, vehicleControl, 19, api.simx_opmode_oneshot_wait); vrchk(api, res);
            
            obj.length = maxY - minY;
            
            % We know that the youbot in longer than wider
            if obj.length < obj.width
                tmp = obj.length;
                obj.length = obj.width;
                obj.width = tmp;
            end

            %Init the wheels and the sensors components
            obj.hokuyo.init(api, vrep, obj.handle);
            obj.wheels.init(api, vrep);
            
            % Get the kinematic information of the youbot
            [res, vector] = api.simxGetObjectPosition(vrep, obj.wheels.frHandle, obj.wheels.flHandle, api.simx_opmode_oneshot_wait); vrchk(api, res);
            a = vector(1)/2; % Distance from center of wheel to the mid width of the robot
            
            [res, vector] = api.simxGetObjectPosition(vrep, obj.wheels.frHandle, obj.wheels.rrHandle, api.simx_opmode_oneshot_wait); vrchk(api, res);
            b = vector(2)/2; % Distance from center of wheel to the length of the robot
            
            % Compute the jacobian augmented forwad kinematic matrix of the youbot
            r = obj.wheels.wheelRadius;
            obj.Ja = 1/r * [
                1   1   (a+b)   1;
                1  -1  -(a+b)   1;
                1   1  -(a+b)  -1;
                1  -1   (a+b)  -1
            ];
        
            % Compute the augmented inverse kinematics of the youbot
            obj.JaInv = r/4 * [
                1        1        1        1;
                1       -1        1       -1;
                1/(a+b) -1/(a+b) -1/(a+b)  1/(a+b);
                4/r      4/r     -4/r     -4/r
            ];
                        
            % Display the red laser beams of active sensors
            api.simxSetIntegerSignal(vrep, 'displaylasers', 1, api.simx_opmode_oneshot); 
        end
    end
end

