classdef Brain < handle
    
    properties
        vrep;
        robot;
        controller;
        map;
        wheelsTrajectory;
        classTreshold;
        
        %ExpandData
        expander;
        dimExpander;
        inflateRay;
    end
    
    methods
        function obj = Brain(vrep)
            obj.vrep = vrep;
        end
        
        function init(obj, api, vrep, robot, scale, radianPrecision, classTreshold)
            obj.robot = robot;
            obj.classTreshold = classTreshold;
            obj.controller = RobotController();
                        
            %Initialize the controller
            obj.controller.init(api, vrep, robot, scale);
            
            if(obj.robot.length > obj.robot.width)
                obj.inflateRay = obj.robot.length/2;
            else
                obj.inflateRay = obj.robot.width/2;
            end
            
            %{
            %Data expander
            if(obj.robot.length >= obj.robot.width)
              else
                ray =  obj.robot.width*3/2;
            end
            [x, y] = meshgrid(-ray:1/scale:ray, -ray:1/scale:ray);
            obj.expander = [reshape(x, [], 1), reshape(y, [], 1)];
            obj.dimExpander = size(obj.expander);
            %}
            
            %Initialize the map
            [x, y, ~] = transl(robot.pose);
            obj.map = Map([x,y], robot.width, robot.length, scale, radianPrecision, robot.hokuyo.range);
            
            %Do a barrel roll
            obj.wheelsTrajectory = obj.robot.pose * trotz(3*pi/4);
            obj.wheelsTrajectory(:,:,2) = obj.robot.pose * trotz(5*pi/4);
            obj.wheelsTrajectory(:,:,3) = obj.robot.pose;
                           
            obj.drive();
        end
        
        function work(obj)      
            obj.explore();
            obj.dispatch();
        end
        
        function updateData(obj) 
            obj.controller.updateData();
            %dimHits = size(obj.robot.hokuyo.hits);
            %obj.robot.hokuyo.hits = kron(obj.robot.hokuyo.hits, ones(obj.dimExpander(1), 1)) + repmat(obj.expander, dimHits(1), 1);
        end
        
        function drive(obj, path)
            % Update data
            obj.updateData();
            obj.map.update(obj.robot.hokuyo);
            
            % Interpolate the trajectory
            vseg = [0.3 0.3]; % Max speed on each axis
            dseg = [];        % Duration per each segment
            dt = 0.01;        % Sample interval
            at = 0.05;        % Acceleration time
            
            % Test path
            path =[ 
              5  -5.25;
            ];
            
            robotInitialPose = obj.controller.getSe2().T;
            robotInitialPose = [robotInitialPose(1,3) robotInitialPose(2,3)];
            
            robotpath = mstraj(path, vseg, dseg, robotInitialPose, dt, at);
            tseg = numrows(robotpath) * dt;
            lispaceRes = linspace(0, tseg, numrows(robotpath));
            
            tic;
            timeOld = 0;
            while(true) % While we have points to go
                
                % Time computation
                timeNew = toc;
                timeDelta = timeNew - timeOld + 0.15;
                timeOld = timeNew;
                
                curGoal = interp1(lispaceRes, robotpath, timeNew);
                if (isnan(curGoal))
                    obj.controller.setWheelsSpeed(0, 0, 0, 0);
                    break;
                end
                    
                % Compute the point relative to the robot pose
                goalToRobot = homtrans(obj.controller.getSe2Inv().T, curGoal');

                % Compute the distance and angle to the point
                distance = sqrt(goalToRobot(1)^2 + goalToRobot(2)^2);
                velocity = distance/timeDelta;

                %velocity = goalToRobot(2)/goalToRobot(1);
                angle = atan2(goalToRobot(2), goalToRobot(1));
                angularVelocity = angle/timeDelta;

                velocities = [  velocity * cos(angle);
                                velocity * sin(angle);
                                min(angularVelocity, 0.3);
                                0;
                             ];

                wheelsSpeed = obj.robot.Ja * velocities;
                obj.controller.setWheelsSpeed(wheelsSpeed(4), wheelsSpeed(1), wheelsSpeed(3), wheelsSpeed(2));
            end
        end
        
        function ret = updateWheelsTrajectory(obj)
            sizeTrajectory = size(obj.wheelsTrajectory);

            if (length(sizeTrajectory) == 2 & sizeTrajectory == [0 0])
                ret = false;
            else
                if (obj.map.onCell(obj.robot.pose, obj.wheelsTrajectory(:,:,1)) & obj.map.inOrientation(obj.robot.pose, obj.wheelsTrajectory(:,:,1)))
                    if length(sizeTrajectory) == 2
                        ret = false;
                    else
                        obj.wheelsTrajectory = obj.wheelsTrajectory(:,:,2:end);
                        ret = true;
                    end
                else
                    ret = true;
                end
            end
        end
        
        function explore(obj)
            obj.discoverMap();
            %obj.discoverBins();
        end
        
        function dispatch(obj)
            %TODO
        end
        
        function discoverMap(obj)
            while(obj.setNextExplorationTrajectory())
                obj.drive();
            end
        end
        
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
%{
            binaryMap = obj.map.content >= obj.classTreshold;

            ds = Dstar(uint16(binaryMap));
            while(not(size(corners == [0 0])))
                point = round(corners(1,:));
                validPoint = getValidPoint(binaryMap, zeros(size(binaryMap)), point);
                ds.plan(validPoint);
                pathPos = generateWheelsTrajectory(ds, validPoint, false, );
              
                obj.wheelsTrajectory = pathPos;
                obj.drive();
%}
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
        
        function ret = setNextExplorationTrajectory(obj)
            obj.map.print();
            ds = Dstar(uint16(obj.map.content >= obj.classTreshold), {'inflate', obj.inflateRay});
            [x, y, ~] = transl(obj.robot.pose);
            rpos = [ceil(y * obj.map.scale + obj.map.offset(2) - 0.5), ceil(x * obj.map.scale + obj.map.offset(1) - 0.5)];
            ds.plan(rpos);
            
            distanceMap = ds.distancemap_get();
            distanceMap(obj.map.content <= -obj.classTreshold | obj.map.content >= obj.classTreshold) = Inf;
            
            [~, i] = min(distanceMap(:));
            [row, col] = ind2sub(size(distanceMap), i);
            
            disp('going to');
            [row, col]
            %Ok jusque la
            if distanceMap(row, col) == Inf 
                obj.wheelsTrajectory = [];
                ret = false;
            else
                obj.generateWheelsTrajectory(ds, [col, row], true, fliplr(rpos));
                ret = true;
            end
        end
    end
end

