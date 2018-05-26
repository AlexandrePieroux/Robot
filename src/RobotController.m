classdef RobotController < handle
    
    properties
        api;
        vrep;
        robot;
        rangeLimit;
        
        xGrid;
        yGrid;
        
        a;
    end
    
    methods
        function obj = RobotController()
        end
        
        function init(obj, api, vrep, robot, scale)
            obj.robot = robot;
            obj.robot.init(api, vrep);
            obj.a = (2*pi)/60 * obj.robot.wheels.wheelRadius;

            obj.api = api;
            obj.vrep = vrep;
            
            % Initialize the grids
            [x, y] = meshgrid(-obj.robot.hokuyo.range:scale:obj.robot.hokuyo.range, -obj.robot.hokuyo.range:scale:obj.robot.hokuyo.range);
            obj.xGrid = reshape(x, [], 1);
            obj.yGrid = reshape(y, [], 1);
            
            obj.rangeLimit = obj.robot.hokuyo.range - 0.0001;
            
            obj.updateData();
            obj.setWheelsSpeed(0, 0, 0, 0);
        end
        
        function updateData(obj)
            % Robot center, relative to the absolute frame
            [res, arcPos] = obj.api.simxGetObjectPosition(obj.vrep, obj.robot.handle, -1, obj.api.simx_opmode_oneshot_wait); vrchk(obj.api, res, true);
            [res, arcOri] = obj.api.simxGetObjectOrientation(obj.vrep, obj.robot.handle, -1, obj.api.simx_opmode_oneshot_wait); vrchk(obj.api, res, true);
            
            % Hokuyo1, relative to the robot center frame
            [res, rh1Pos] = obj.api.simxGetObjectPosition(obj.vrep, obj.robot.hokuyo.firstHandle, obj.robot.handle, obj.api.simx_opmode_oneshot_wait); vrchk(obj.api, res, true);
            [res, rh1Ori] = obj.api.simxGetObjectOrientation(obj.vrep, obj.robot.hokuyo.firstHandle, obj.robot.handle, obj.api.simx_opmode_oneshot_wait); vrchk(obj.api, res, true);
            
            % Hokuyo2, relative to the robot center frame
            [res, rh2Pos] = obj.api.simxGetObjectPosition(obj.vrep, obj.robot.hokuyo.secondHandle, obj.robot.handle, obj.api.simx_opmode_oneshot_wait); vrchk(obj.api, res, true);
            [res, rh2Ori] = obj.api.simxGetObjectOrientation(obj.vrep, obj.robot.hokuyo.secondHandle, obj.robot.handle, obj.api.simx_opmode_oneshot_wait); vrchk(obj.api, res, true);

            [res1, ~, auxData1, auxPacketInfo1] = obj.api.simxReadVisionSensor(obj.vrep, obj.robot.hokuyo.firstHandle, obj.api.simx_opmode_oneshot_wait); vrchk(obj.api, res1, true);
            [res2, ~, auxData2, auxPacketInfo2] = obj.api.simxReadVisionSensor(obj.vrep, obj.robot.hokuyo.secondHandle, obj.api.simx_opmode_oneshot_wait); vrchk(obj.api, res2, true);
            
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
    
            % Void points
            [rx, ry, ~] = transl(obj.robot.pose);
            [h1x, h1y, ~] = transl(obj.robot.hokuyo.firstPose);
            [h2x, h2y, ~] = transl(obj.robot.hokuyo.secondPose);
            mask =  [h1points, h2points]; 
            xPoly = obj.xGrid + rx;
            yPoly = obj.yGrid + ry;

            in = inpolygon(xPoly, yPoly, [h1x mask(1, :) h2x], [h1y mask(2, :) h2y]);
            obj.robot.hokuyo.voidPoints = [xPoly(in), yPoly(in)];
        end
        
        function image = takePicture(obj)
            obj.api.simxSetIntegerSignal(obj.vrep, 'handle_rgb_sensor', 1, obj.api.simx_opmode_oneshot_wait);
            [res, resolution, image] = obj.api.simxGetVisionSensorImage2(obj.vrep, obj.robot.rgbsensor.handle, 0, obj.api.simx_opmode_oneshot_wait); vrchk(obj.api, res);
        end
        
        function drivePose(obj, map, poses, dt)
            % Plane the trajectory
            rotpath = obj.interp2z(poses, dt);
            
            % Plane the rotation
            tseg = numrows(rotpath) * dt;
            lispaceRot = linspace(0, tseg, numrows(rotpath));
            
            timeRef = tic;
            timeOld = 0;
            while(true) % While we have points to go
                obj.updateData();
                map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);

                % Time computation
                timeNew = toc(timeRef);
                timeDelta = (timeNew - timeOld);
                timeOld = timeNew;

                timeInterp = timeNew + (timeDelta * 1.5);
                curRot = interp1(lispaceRot, rotpath, timeInterp);
                if isnan(curRot)
                    obj.setWheelsSpeed(0, 0, 0, 0);
                    break;
                end

                % Compute angle to the point to the point and then derive the
                % angular velocity.       
                relativeAngle = wrapToPi(curRot - obj.robot.se2.angle());
                angularVelocity = min(abs(relativeAngle/timeDelta), obj.robot.maxav * 1.2);
                angularVelocity = angularVelocity * sign(relativeAngle);

                velocities = [  
                                0;
                                0;
                                angularVelocity;
                                0;
                             ];

                % Use the augmented jacobian matrice of the kinematic to
                % get wheel speed.
                wheelsSpeed = obj.robot.Ja * velocities;
                obj.setWheelsSpeed(wheelsSpeed(4), wheelsSpeed(1), wheelsSpeed(3), wheelsSpeed(2));
            end
        end
        
        function drivePath(obj, map, path, dt, at)            
            % Get robot cartesian position regardless orientation
            robotInitialPose = obj.robot.se2.T;
            robotInitialPose = [robotInitialPose(1,3) robotInitialPose(2,3)];
            
            % Plane the trajectory
            [rotpath, dseg] = obj.interp2zWithT(path, dt);
            
            % Plane the translational trajectory using quintic polynomial and heuristics
            translpath = mstraj(path, [], dseg, robotInitialPose, dt, at);
            tseg = numrows(translpath) * dt;
            lispaceTransl = linspace(0, tseg, numrows(translpath));
            tseg = numrows(rotpath) * dt;
            lispaceRot = linspace(0, tseg, numrows(rotpath));
            
            timeRef = tic;
            timeOld = 0;
            while(true) % While we have points to go
                obj.updateData();
                map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);
                
                % Time computation
                timeNew = toc(timeRef);
                timeDelta = (timeNew - timeOld);
                timeOld = timeNew;

                timeInterp = timeNew + (timeDelta * 1.5);
                curGoal = interp1(lispaceTransl, translpath, timeInterp);
                curRot = interp1(lispaceRot, rotpath, timeInterp);
                if isnan(curGoal)
                    obj.setWheelsSpeed(0, 0, 0, 0);
                    break;
                end
                    
                % Compute the point relative to the robot pose
                goalToRobot = homtrans(obj.robot.se2Inv.T, curGoal');

                % Compute the distance to the point and then derive the
                % velocity.
                distance = sqrt(goalToRobot(1)^2 + goalToRobot(2)^2);
                translAngle = atan2(goalToRobot(2), goalToRobot(1));
                velocity = min(abs(distance/timeDelta), sqrt(obj.robot.maxv(1)^2 + obj.robot.maxv(2)^2));
                velocity = velocity * sign(distance);

                % Compute angle to the point to the point and then derive the
                % angular velocity.             
                relativeAngle = wrapToPi(curRot - obj.robot.se2.angle());
                angularVelocity = min(abs(relativeAngle/timeDelta), obj.robot.maxav * 1.2);
                angularVelocity = angularVelocity * sign(relativeAngle);
                
                velocities = [  
                                velocity * cos(translAngle);
                                velocity * sin(translAngle);
                                angularVelocity;
                                0;
                             ];

                % Use the augmented jacobian matrice of the kinematic to
                % get wheel speed.
                wheelsSpeed = obj.robot.Ja * velocities;
                obj.setWheelsSpeed(wheelsSpeed(4), wheelsSpeed(1), wheelsSpeed(3), wheelsSpeed(2));                
            end
        end
    end
    
    
    methods (Access='protected')
        
        function angles = interp2z(obj, se2Poses, dt)
            % Get robot pose and pi rotation pose
            curOri = obj.robot.se2.T;
            curAngle = tr2rpy(curOri);
            curAngle = curAngle(3);
            angles = [];
            
            for n=1:size(se2Poses, 3)
                nextOri = se2Poses(:,:,n);
                nextAngle = tr2rpy(nextOri);
                nextAngle = nextAngle(3);

                nextOri(1:2,3) = 0;
                curOri(1:2,3) = 0;

                % Compute the time needed according to robot specs
                rotTime = pi - abs(abs(curAngle - nextAngle) - pi); 
                rotTime = rotTime/obj.robot.maxav;
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
                angles = [angles; res];
            end
        end
        
        function [angles, timeSeg] = interp2zWithT(obj, path, dt)
            % Get SE3 robot pose with z axis rotation
            robotPose = obj.robot.pose;
            timeSeg = zeros(numrows(path), 1);

            % Initialise rotation variables
            curPose = SE3(robotPose.R * rotz(-pi/2), robotPose.t);
            angles = tr2rpy(curPose);
            angles = angles(3);
            curP = transl(curPose);
            curP = curP(1:2);
            curOri = angles;
            robotMaxV = sqrt(obj.robot.maxv(1)^2 + obj.robot.maxv(2)^2);

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
                rotTime = rotTime/obj.robot.maxav;

                % The slowest composant limit the other
                timeTrav = max(translTime, rotTime);
                timeSeg(n) = timeTrav;

                % Orientation interpolation if needed
                relAngle = tr2rpy(relativePose);
                relAngle = relAngle(3);
                if round(relAngle, 1) ~= 0
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
                    angles = [angles; anglesToAdd];
                else
                    lastOri = tr2rpy(curPose);
                    lastOri = repelem(lastOri(3), ceil(translTime/dt))';
                    angles = [angles; lastOri];
                end

                curPose = nextPose;
                curOri = nextOri;
                curP = nextP;
            end
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

