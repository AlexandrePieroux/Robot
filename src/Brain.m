classdef Brain < handle
    
    properties
        % Api object
        vrep;
        
        % Component objects
        robot;
        controller;
        map;
        
        % Map threshold
        classTreshold;
        
        %ExpandData
        expander;
        dimExpander;
        inflateRay;
        
        % Path interpolation data
        dseg = [];         % Duration per each segment
        dt = 0.2;          % Sample interval
        at = 0.8;          % Acceleration time
        
        % Data
        path;
    end
    
    methods
        function obj = Brain(vrep)
            obj.vrep = vrep;
        end
        
        function init(obj, api, vrep, robot, scale, classTreshold)
            obj.robot = robot;
            obj.classTreshold = classTreshold;
            obj.controller = RobotController();
                        
            %Initialize the controller
            obj.controller.init(api, vrep, robot, scale);
            obj.inflateRay = sqrt(obj.robot.length^2 + obj.robot.width^2) * 1.15;
            
            %Initialize the map
            obj.map = Map(scale);
            obj.map.insertRobot(robot);
            obj.inflateRay = obj.inflateRay * obj.map.resolution;
        end
        
        function work(obj)
            
            % Do a barrel roll
            obj.barrelRoll();
            
            % Start exploration
            obj.explore();
            
            %%%%%%%%%% DEBUG %%%%%%%%%%%%%
            %robotMap = obj.map;
            %save('map.mat', 'robotMap');
            
            %%%%%%%%%% DEBUG %%%%%%%%%%%%%
            %load('map.mat');
            %obj.map = robotMap;
            
            %obj.dispatch();
        end
        
        function explore(obj)
            obj.discoverMap();
            %obj.discoverBins();
        end
        
        function discoverMap(obj)
            % Update data
            obj.controller.updateData();
            obj.map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);
            
            while(obj.setNextExplorationTrajectory())
                obj.drive(true);
            end
        end
        
        function dispatch(obj)
            %TODO
        end
        
        function barrelRoll(obj)  
            % Plane the trajectory
            rotpath = obj.robot.rotate2Pi(obj.dt);
            
            % Plane the rotation
            tseg = numrows(rotpath) * obj.dt;
            lispaceRot = linspace(0, tseg, numrows(rotpath));
            
            timeRef = tic;
            timeOld = 0;
            while(true) % While we have points to go
                obj.controller.updateData();
                obj.map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);

                % Time computation
                timeNew = toc(timeRef);
                timeDelta = (timeNew - timeOld);
                timeOld = timeNew;

                timeInterp = timeNew + (timeDelta * 1.5);
                curRot = interp1(lispaceRot, rotpath, timeInterp);
                if isnan(curRot)
                    obj.controller.setWheelsSpeed(0, 0, 0, 0);
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
                obj.controller.setWheelsSpeed(wheelsSpeed(4), wheelsSpeed(1), wheelsSpeed(3), wheelsSpeed(2));
            end
        end
        
        function drive(obj, reduce)
            % Transform path into waypoints if asked
            if reduce
                [wayPoints(:,1), wayPoints(:,2)] = reducem(obj.path(:,1), obj.path(:,2), 0.1);
                wayPoints = wayPoints(2:end,:);
            else
                wayPoints = obj.path;
            end
            
            % Get robot cartesian position regardless orientation
            robotInitialPose = obj.robot.se2.T;
            robotInitialPose = [robotInitialPose(1,3) robotInitialPose(2,3)];
            
            % Plane the trajectory
            [rotpath, obj.dseg] = obj.robot.getHeadings(wayPoints, obj.dt);
            
            % Plane the translational trajectory using quintic polynomial and heuristics
            translpath = mstraj(wayPoints, [], obj.dseg, robotInitialPose, obj.dt, obj.at);
            tseg = numrows(translpath) * obj.dt;
            lispaceTransl = linspace(0, tseg, numrows(translpath));
            tseg = numrows(rotpath) * obj.dt;
            lispaceRot = linspace(0, tseg, numrows(rotpath));
            
            timeRef = tic;
            timeOld = 0;
            while(true) % While we have points to go
                obj.controller.updateData();
                obj.map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);
                
                % Time computation
                timeNew = toc(timeRef);
                timeDelta = (timeNew - timeOld);
                timeOld = timeNew;

                timeInterp = timeNew + (timeDelta * 1.5);
                curGoal = interp1(lispaceTransl, translpath, timeInterp);
                curRot = interp1(lispaceRot, rotpath, timeInterp);
                if isnan(curGoal)
                    obj.controller.setWheelsSpeed(0, 0, 0, 0);
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
                obj.controller.setWheelsSpeed(wheelsSpeed(4), wheelsSpeed(1), wheelsSpeed(3), wheelsSpeed(2));                
            end
        end
        
%{
        function discoverBins(obj)
            % Initialise the list of pictures to load
            pictures = {'pictures/dog.png', ...
                      'pictures/plant.png', ...
                      'pictures/pumpkin.png', ...
                      'pictures/trashcan.png', ...
                      'pictures/trike'};
                  
            % Corner threshold test
            cornerThreshold = 50;
            
            %Load the matcher
            matcher = Matcher(pictures, cornerThreshold);
            corners = matcher.cornerDetection(obj.map.content);

            ds = Dstar(uint16(obj.map.content >= obj.classTreshold), {'inflate', obj.inflateRay});
            while(not(length(sizeTrajectory) == 2 & size(corners == [0 0])))
                obj.controller.objective = [0, corners(1,:)];

            binaryMap = obj.map.content >= obj.classTreshold;

            ds = Dstar(uint16(binaryMap));
            while(not(size(corners == [0 0])))
                point = round(corners(1,:));
                validPoint = getValidPoint(binaryMap, zeros(size(binaryMap)), point);
                ds.plan(validPoint);
                pathPos = generateWheelsTrajectory(ds, validPoint, false, );
              
                obj.wheelsTrajectory = pathPos;
                obj.drive();

                % Take picture
                % TODO
                imageHits = matcher.imageMatch(file);
                
                % Check if the image is good or not
                % TODO
                
                corners = corners(2:end, :);
            end 
        end
        

        function validPoint = getValidPoint(obj, binaryMap, memory, stack)
            point = stack(1, :);
            stack = stack(2:end,:);
            
            if memory(point(1), point(2)) == 0
                memory(point(1), point(2)) = 1;
                if binaryMap(stack(1,1), point(1,2)) == 0
                    validPoint = point;
                else
                    stack = [stack; ...
                        [point(1)-1, point(2)];...
                        [point(1)+1, point(2)];...
                        [point(1), point(2)-1];...
                        [point(1), point(2)+1]];
                    validPoint = getValidPoint(binaryMap, memory, stack);
                end
            else
                validPoint = getValidPoint(binaryMap, memory, stack);
            end
        end
%}
        
        function ret = setNextExplorationTrajectory(obj)
            obj.controller.updateData();
            obj.map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);
                
            [x, y, ~] = transl(obj.robot.pose);
            
            [rposR, rposC] = obj.map.world2Map(x, y);
            mapToProcess = obj.map.inflate(obj.classTreshold, double(round(obj.inflateRay)), [rposR, rposC]);
                
            ds = Dstar(mapToProcess);
            ds.plan([rposC, rposR]);
            
            distanceMap = ds.h;
            distanceMap(obj.map.content <= -obj.classTreshold | obj.map.content >= obj.classTreshold) = Inf;
            
            [v, i] = min(distanceMap(:));
            [row, col] = ind2sub(size(distanceMap), i);
            
            if v == Inf 
                obj.path = [];
                ret = false;
            else
                obj.path = ds.query([col, row]);
                
                % Inverting the path since we plan from the robot pose
                [obj.path(:,1), obj.path(:,2)] = obj.map.map2World(flipud(obj.path(:,2)), flipud(obj.path(:,1)));
                
                % Don't let it bump into obstacle it does not know
                newEnd = round(size(obj.path, 1) * 0.7);
                obj.path = obj.path(1:newEnd,:);
                ret = true;
            end
        end
    end
end

