classdef RobotController < handle
    
    properties
        api;
        vrep;
        robot;
        xGrid;
        yGrid;
        rangeLimit;
    end
    
    methods
        function obj = RobotController()
        end
        
        function init(obj, api, vrep, robot, scale)
            obj.robot = robot;
            obj.robot.init(api, vrep);

            obj.api = api;
            obj.vrep = vrep;
                        
            % Initialize the grids
            [x, y] = meshgrid(-obj.robot.hokuyo.range:1/scale:obj.robot.hokuyo.range, -obj.robot.hokuyo.range:1/scale:obj.robot.hokuyo.range);
            obj.xGrid = reshape(x, [], 1);
            obj.yGrid = reshape(y, [], 1);
            
            obj.rangeLimit = obj.robot.hokuyo.range - 0.0001;
            
            obj.updateData();
            obj.setWheelsSpeed(0, 0, 0, 0);
        end
        
        function updateData(obj)
            % Robot center, relative to the absolute frame
            [res, arcPos] = obj.api.simxGetObjectPosition(obj.vrep, obj.robot.handle, -1, obj.api. simx_opmode_blocking); vrchk(obj.api, res, true);
            [res, arcOri] = obj.api.simxGetObjectOrientation(obj.vrep, obj.robot.handle, -1, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);

            % Hokuyo1, relative to the robot center frame
            [res, rh1Pos] = obj.api.simxGetObjectPosition(obj.vrep, obj.robot.hokuyo.firstHandle, obj.robot.handle, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);
            [res, rh1Ori] = obj.api.simxGetObjectOrientation(obj.vrep, obj.robot.hokuyo.firstHandle, obj.robot.handle, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);

            % Hokuyo2, relative to the robot center frame
            [res, rh2Pos] = obj.api.simxGetObjectPosition(obj.vrep, obj.robot.hokuyo.secondHandle, obj.robot.handle, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);
            [res, rh2Ori] = obj.api.simxGetObjectOrientation(obj.vrep, obj.robot.hokuyo.secondHandle, obj.robot.handle, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);

            [res1, ~, auxData1, auxPacketInfo1] = obj.api.simxReadVisionSensor(obj.vrep, obj.robot.hokuyo.firstHandle, obj.api.simx_opmode_blocking); vrchk(obj.api, res1, true);
            [res2, ~, auxData2, auxPacketInfo2] = obj.api.simxReadVisionSensor(obj.vrep, obj.robot.hokuyo.secondHandle, obj.api.simx_opmode_blocking); vrchk(obj.api, res2, true);
            
            % Poses, relative to the absolute frame
            obj.robot.pose = transl(arcPos) * trotx(arcOri(1)) * troty(arcOri(2)) * trotz(arcOri(3));
            obj.robot.hokuyo.firstPose = obj.robot.pose * transl(rh1Pos) * trotx(rh1Ori(1)) * troty(rh1Ori(2)) * trotz(rh1Ori(3));
            obj.robot.hokuyo.secondPose = obj.robot.pose * transl(rh2Pos) * trotx(rh2Ori(1)) * troty(rh2Ori(2)) * trotz(rh2Ori(3));
       
            width = auxData1(auxPacketInfo1(1) + 1);
            height = auxData1(auxPacketInfo1(1) + 2);
            h1points = reshape(auxData1((auxPacketInfo1(1) + 2 + 1):end), 4, width*height);
            h1hits = h1points(4,:) < obj.rangeLimit;
            h1points = homtrans(obj.robot.hokuyo.firstPose, h1points(1:3,:));
            
            width = auxData2(auxPacketInfo2(1) + 1);
            height = auxData2(auxPacketInfo2(1) + 2);
            h2points = reshape(auxData2((auxPacketInfo2(1) + 2 + 1):end), 4, width*height);
            h2hits = h2points(4,:) < obj.rangeLimit;
            h2points = homtrans(obj.robot.hokuyo.secondPose, h2points(1:3,:));

            % Hit points
            obj.robot.hokuyo.hits = [h1points(1:2,h1hits), h2points(1:2,h2hits)]';

            % Surface points
            mask =  [h1points, h2points]; 
            [h1x, h1y, ~] = transl(obj.robot.hokuyo.firstPose);
            [h2x, h2y, ~] = transl(obj.robot.hokuyo.secondPose);
            [rx, ry, ~] = transl(obj.robot.pose);
            xPoly = obj.xGrid + rx;
            yPoly = obj.yGrid + ry;

            in = inpolygon(xPoly, yPoly, [h1x mask(1, :) h2x], [h1y mask(2, :) h2y]);
            obj.robot.hokuyo.surface = [xPoly(in) yPoly(in)];
        end
        
        function setWheelsSpeed(obj, flS, rlS, frS, rrS)
            res = obj.api.simxPauseCommunication(obj.vrep, true); vrchk(obj.api, res);
            obj.api.simxSetJointTargetVelocity(obj.vrep, obj.robot.wheels.flHandle, flS, obj.api.simx_opmode_oneshot);
            obj.api.simxSetJointTargetVelocity(obj.vrep, obj.robot.wheels.rlHandle, rlS,obj.api.simx_opmode_oneshot);
            obj.api.simxSetJointTargetVelocity(obj.vrep, obj.robot.wheels.frHandle, frS, obj.api.simx_opmode_oneshot);
            obj.api.simxSetJointTargetVelocity(obj.vrep, obj.robot.wheels.rrHandle, rrS, obj.api.simx_opmode_oneshot);
            res = obj.api.simxPauseCommunication(obj.vrep, false); vrchk(obj.api, res);
        end
    end
    
end

