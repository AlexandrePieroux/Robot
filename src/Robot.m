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
        
        % Kinematic information
        maxv;  % Max speed on each axis [x y]
        maxav; % Max angular speed
        Ja;    % Augmented forward kinematic matrix
        JaInv; % Augmented inverse kinematic matrix
    end
    
    methods
        function obj = Robot(range, maxx, maxy, maxav) 
            obj.hokuyo = Hokuyo(range);
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
                        
            % Display the red laser beams of active sensors
            api.simxSetIntegerSignal(vrep, 'displaylasers', 1, api.simx_opmode_oneshot); 
        end
        
        function headings = rotate2Pi(obj, dt)
            % Get robot pose and pi rotation pose
            curOri = obj.se2.T;
            curAngle = tr2rpy(curOri);
            curAngle = curAngle(3);

            headings = [];

            ori(:,:,1) = curOri * rotz(pi/2);
            ori(:,:,2) = curOri * rotz(pi);
            ori(:,:,3) = curOri * rotz(3*pi/2);
            ori(:,:,4) = curOri * rotz(2*pi);

            for n=1:size(ori, 3)
                nextOri = ori(:,:,n);
                nextAngle = tr2rpy(nextOri);
                nextAngle = nextAngle(3);

                nextOri(1:2,3) = 0;
                curOri(1:2,3) = 0;

                % Compute the time needed according to robot specs
                rotTime = pi - abs(abs(curAngle - nextAngle) - pi); 
                rotTime = rotTime/obj.maxav;
                rotTime = ceil(rotTime/dt) - 1;

                % Interpolation using unit quaternion
                robotOriQ = UnitQuaternion(curOri);
                nextPoseQ = UnitQuaternion(nextOri);
                res = robotOriQ.interp(nextPoseQ, [0:rotTime]'/rotTime, 'shortest');

                % Return the angle interpolation
                res = tr2rpy(res.R);
                res = res(:,3); 

                curOri = nextOri;
                curAngle = nextAngle;
                headings = [headings; res];
            end
        end
        
        function [res, timeSeg] = getHeadings(obj, path, dt)
            % Get SE3 robot pose with z axis rotation
            robotPose = obj.pose;
            timeSeg = [];

            % Initialise rotation variables
            curPose = SE3(robotPose.R * rotz(-pi/2), robotPose.t);
            res = tr2rpy(curPose);
            res = res(3);
            curP = transl(curPose);
            curP = curP(1:2);
            curOri = res;
            robotMaxV = sqrt(obj.maxv(1)^2 + obj.maxv(2)^2);

            for n = 1:numrows(path)
                % Get next cartesian point and compute heading angle
                nextP = path(n,:);
                nextOri = atan2(nextP(2) - curP(2), nextP(1) - curP(1));

                % Get time for translation and rotation
                nextPose = SE3(rotz(nextOri), [nextP, 0]');
                relativePose = curPose.inv.T * nextPose.T;

                translTime = transl(relativePose);
                translTime = sqrt(translTime(1)^2 + translTime(2)^2);
                translTime = abs(translTime)/robotMaxV;

                rotTime = pi - abs(abs(curOri - nextOri) - pi); 
                rotTime = rotTime/obj.maxav;
                rotTime = ceil(rotTime/dt) - 1;

                % The slowest composant limit the other
                timeTrav = max(translTime, rotTime);
                timeSeg = [timeSeg; timeTrav];

                % Orientation interpolation if needed
                relAngle = tr2rpy(relativePose);
                relAngle = relAngle(3);
                if round(relAngle, 3) ~= 0
                    % Interpolation using Quaternion function
                    rotSteps = ceil(rotTime/dt) - 1;

                    % Get the unit quaterion with only the rotation
                    curPoseQ = curPose.T;
                    curPoseQ(1:2,3) = 0;
                    curPoseQ = UnitQuaternion(curPoseQ);

                    nextPoseQ = nextPose.T;
                    nextPoseQ(1:2,3) = 0;
                    nextPoseQ = UnitQuaternion(nextPoseQ);

                    resInterpQ = curPoseQ.interp(nextPoseQ, [0:rotSteps]'/rotSteps, 'shortest');

                    % Taking z rotation
                    anglesToAdd = tr2rpy(resInterpQ.R);
                    anglesToAdd = anglesToAdd(:,3);

                    % Rotate as fast as possible
                    if rotTime < translTime
                        additionalSteps = ceil(translTime/dt) - rotSteps + 1;
                        anglesToAdd = [anglesToAdd; repelem(anglesToAdd(end), additionalSteps)'];
                    end

                    % Add interpolation to the result
                    res = [res; anglesToAdd];
                else
                    lastOri = tr2rpy(curPose);
                    lastOri = repelem(lastOri(3), ceil(translTime/dt))';
                    res = [res; lastOri];
                end

                curPose = nextPose;
                curOri = nextOri;
                curP = nextP;
            end
        end
    end
end

