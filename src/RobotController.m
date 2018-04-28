classdef RobotController < handle
    
    properties
        api;
        vrep;
        robot;
        rangeLimit;
    end
    
    methods
        function obj = RobotController()
        end
        
        function init(obj, api, vrep, robot)
            obj.robot = robot;
            obj.robot.init(api, vrep);

            obj.api = api;
            obj.vrep = vrep;
                        
            obj.rangeLimit = obj.robot.hokuyo.range - 0.0001;
            
            obj.updateData();
            obj.setWheelsSpeed(0, 0, 0, 0);
        end
        
        function updateData(obj)
            % Robot center, relative to the absolute frame
            [res, arcPos] = obj.api.simxGetObjectPosition(obj.vrep, obj.robot.handle, -1, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);
            [res, arcOri] = obj.api.simxGetObjectOrientation(obj.vrep, obj.robot.handle, -1, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);

            % Hokuyo1, relative to the robot center frame
            [res, rh1Pos] = obj.api.simxGetObjectPosition(obj.vrep, obj.robot.hokuyo.firstHandle, obj.robot.handle, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);
            [res, rh1Ori] = obj.api.simxGetObjectOrientation(obj.vrep, obj.robot.hokuyo.firstHandle, obj.robot.handle, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);

            % Hokuyo2, relative to the robot center frame
            [res, rh2Pos] = obj.api.simxGetObjectPosition(obj.vrep, obj.robot.hokuyo.secondHandle, obj.robot.handle, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);
            [res, rh2Ori] = obj.api.simxGetObjectOrientation(obj.vrep, obj.robot.hokuyo.secondHandle, obj.robot.handle, obj.api.simx_opmode_blocking); vrchk(obj.api, res, true);

            [res1, ~, auxData1, auxPacketInfo1] = obj.api.simxReadVisionSensor(obj.vrep, obj.robot.hokuyo.firstHandle, obj.api.simx_opmode_blocking); vrchk(obj.api, res1, true);
            [res2, ~, auxData2, auxPacketInfo2] = obj.api.simxReadVisionSensor(obj.vrep, obj.robot.hokuyo.secondHandle, obj.api.simx_opmode_blocking); vrchk(obj.api, res2, true);
            
            % Robot SE(2) relative to the absolute frame. We apply a
            % rotation of -pi/2 about the z axis in order to have the body
            % of the robot oriented toward the direction we wish to be
            % heading.
            obj.robot.se2 = SE2(arcPos(1), arcPos(2), arcOri(3) - pi/2);
            obj.robot.se2Inv = obj.robot.se2.inv();
            
            % Robot pose and hokuyo poses relatives to world frame.
            obj.robot.pose = SE3(rotx(arcOri(1)) * roty(arcOri(2)) * rotz(arcOri(3)), arcPos');
            obj.robot.hokuyo.firstPose = obj.robot.pose * SE3(rotx(rh1Ori(1)) * roty(rh1Ori(2)) * rotz(rh1Ori(3)), rh1Pos');
            obj.robot.hokuyo.secondPose = obj.robot.pose * SE3(rotx(rh2Ori(1)) * roty(rh2Ori(2)) * rotz(rh2Ori(3)), rh2Pos');
                   
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
            obj.robot.hokuyo.hits = [h1points(1:2,h1hits) h2points(1:2,h2hits)]';  
        end
        
        function setWheelsSpeed(obj, flS, rlS, frS, rrS)
            res = obj.api.simxPauseCommunication(obj.vrep, true); vrchk(obj.api, res);
            obj.api.simxSetJointTargetVelocity(obj.vrep, obj.robot.wheels.flHandle, flS, obj.api.simx_opmode_oneshot);
            obj.api.simxSetJointTargetVelocity(obj.vrep, obj.robot.wheels.rlHandle, rlS, obj.api.simx_opmode_oneshot);
            obj.api.simxSetJointTargetVelocity(obj.vrep, obj.robot.wheels.frHandle, frS, obj.api.simx_opmode_oneshot);
            obj.api.simxSetJointTargetVelocity(obj.vrep, obj.robot.wheels.rrHandle, rrS, obj.api.simx_opmode_oneshot);
            res = obj.api.simxPauseCommunication(obj.vrep, false); vrchk(obj.api, res);
        end
  
    end
    
end

