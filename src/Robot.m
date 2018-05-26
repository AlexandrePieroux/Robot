classdef Robot < handle
    
    properties
        % Api identifier
        handle;
        
        % Pose information
        pose;
        se2;
        se2Inv;
        
        % Physical information
        width;
        length;
        a;
        b;
        r;
        
        % Component objects
        hokuyo;
        wheels;
        rgbsensor;
        
        % Kinematic information
        maxv;  % Max speed on each axis [x y]
        maxav; % Max angular speed
        Ja;    % Augmented forward kinematic matrix
        JaInv; % Augmented inverse kinematic matrix
    end
    
    methods
        function obj = Robot(range, maxx, maxy, maxav) 
            obj.hokuyo = Hokuyo(range);
            obj.rgbsensor = RGBSensor();
            obj.wheels = Wheels();
            obj.maxv = [maxx maxy];
            obj.maxav = maxav;
        end
        
        function init(obj, api, vrep)
            [res, obj.handle] = api.simxGetObjectHandle(vrep, 'youBot_center', api.simx_opmode_oneshot_wait); vrchk(api, res);

            % Get the handle of the vehicle con trol
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
            obj.hokuyo.init(api, vrep);
            obj.rgbsensor.init(api, vrep);
            obj.wheels.init(api, vrep);
            
            % Get the kinematic information of the youbot
            [res, vector] = api.simxGetObjectPosition(vrep, obj.wheels.swFrHandle, obj.wheels.swFlHandle, api.simx_opmode_oneshot_wait); vrchk(api, res);
            obj.a = vector(1)/2; % Distance from center of wheel to the mid width of the robot
            
            [res, vector] = api.simxGetObjectPosition(vrep, obj.wheels.swFrHandle, obj.wheels.swRrHandle, api.simx_opmode_oneshot_wait); vrchk(api, res);
            obj.b = vector(2)/2; % Distance from center of wheel to the length of the robot
            
            % Compute the jacobian augmented forwad kinematic matrix of the youbot
            obj.r = obj.wheels.wheelRadius;
            obj.Ja = 1/obj.r * [
                1   1   (obj.a+obj.b)   1;
                1  -1  -(obj.a+obj.b)   1;
                1   1  -(obj.a+obj.b)  -1;
                1  -1   (obj.a+obj.b)  -1
            ];
        
            % Compute the augmented inverse kinematics of the youbot
            obj.JaInv = obj.r/4 * [
                1        1        1        1;
                1       -1        1       -1;
                1/(obj.a+obj.b) -1/(obj.a+obj.b) -1/(obj.a+obj.b)  1/(obj.a+obj.b);
                4/obj.r      4/obj.r     -4/obj.r     -4/obj.r
            ];                        
       end
    end
end

