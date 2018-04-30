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
        dseg = [];        % Duration per each segment
        dt = 0.01;        % Sample interval
        at = 0.05;        % Acceleration time
        
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
            obj.inflateRay = obj.robot.length;
            
            %Initialize the map
            obj.map = Map(scale);
            obj.map.insertRobot(robot);
            obj.inflateRay = obj.inflateRay * obj.map.resolution;
            
            %Do a barrel roll
            obj.barrelRoll();
        end
        
        function work(obj)      
            obj.explore();
            %obj.dispatch();
        end
        
        function explore(obj)
            obj.discoverMap();
            %obj.discoverBins();
        end
        
        function discoverMap(obj)
            while(obj.setNextExplorationTrajectory())
                obj.drive();
            end
        end
        
        function dispatch(obj)
            %TODO
        end
        
        function updateData(obj) 
            obj.controller.updateData();
            %dimHits = size(obj.robot.hokuyo.hits);
            %obj.robot.hokuyo.hits = kron(obj.robot.hokuyo.hits, ones(obj.dimExpander(1), 1)) + repmat(obj.expander, dimHits(1), 1);
        end
        
        function barrelRoll(obj)   
            velocities = [  0;
                            0;
                            0.3;
                            0;   ];

            wheelsSpeed = obj.robot.Ja * velocities;
            obj.controller.setWheelsSpeed(wheelsSpeed(4), wheelsSpeed(1), wheelsSpeed(3), wheelsSpeed(2));
            
            timeToAchieve = 35.5;
            timeStart = tic;
            while(true)
                obj.controller.updateData();
                obj.map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);
                if toc(timeStart) > timeToAchieve
                    obj.controller.setWheelsSpeed(0, 0, 0, 0);
                    break;
                end
            end
        end
        
        function drive(obj)
            % Update data
            obj.updateData();
            obj.map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);
            
            % Get robot cartesian position regardless orientation
            robotInitialPose = obj.robot.se2.T;
            robotInitialPose = [robotInitialPose(1,3) robotInitialPose(2,3)];
            
            % Plane the trajectory using quintic polynomial and heuristics
            robotpath = mstraj(obj.path, obj.robot.maxv, obj.dseg, robotInitialPose, obj.dt, obj.at);
            tseg = numrows(robotpath) * obj.dt;
            lispaceRes = linspace(0, tseg, numrows(robotpath));
            
            timeRef = tic;
            timeOld = 0;
            while(true) % While we have points to go
                obj.controller.updateData();
                obj.map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);
                
                % Time computation
                % We add 0.45 sec in order to never reach the goal
                timeNew = toc(timeRef);
                timeDelta = (timeNew - timeOld) + 0.45;
                timeOld = timeNew;
                
                curGoal = interp1(lispaceRes, robotpath, timeNew);
                if (isnan(curGoal))
                    obj.controller.setWheelsSpeed(0, 0, 0, 0);
                    break;
                end
                    
                % Compute the point relative to the robot pose
                goalToRobot = homtrans(obj.robot.se2Inv.T, curGoal');

                % Compute the distance to the point and then derive the
                % velocity.
                distance = sqrt(goalToRobot(1)^2 + goalToRobot(2)^2);
                velocity = distance/timeDelta;

                % Compute angle to the point to the point and then derive the
                % angular velocity.
                angle = atan2(goalToRobot(2), goalToRobot(1));
                angularVelocity = angle/timeDelta;
                angularVelocity = min(abs(angularVelocity), obj.robot.maxav) * sign(angularVelocity);

                velocities = [  velocity * cos(angle);
                                velocity * sin(angle);
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
            ds = Dstar(double(full(obj.map.content) >= obj.classTreshold), 'inflate', double(round(obj.inflateRay)), 'progress');
            [x, y, ~] = transl(obj.robot.pose);
            [rposR, rposC] = obj.map.world2Map(x, y);
            ds.plan([abs(rposC), abs(rposR)]);
            
            distanceMap = ds.h;
            distanceMap(obj.map.content <= -obj.classTreshold | obj.map.content >= obj.classTreshold) = Inf;
            
            [v, i] = min(distanceMap(:));
            [row, col] = ind2sub(size(distanceMap), i);
            
            if v == Inf 
                obj.path = [];
                ret = false;
            else
                obj.path = ds.query([col, row]);
                obj.path
                
                % Plot path on map representation
                obj.map.print();
                hold on;
                plot(obj.path(:,1), obj.path(:,2), 'g')
                
                % Inverting the path since we plan from the robot pose
                [obj.path(:,1), obj.path(:,2)] = obj.map.map2World(flipud(obj.path(:,2)), flipud(obj.path(:,1)));
                ret = true;
            end
        end
    end
end

