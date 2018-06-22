classdef Brain < handle
    
    properties
        % Api object
        vrep;
        
        % Component objects
        robot;
        controller;
        map;
        matcher;
        prm;
        
        % Map threshold
        classTreshold;
        
        %ExpandData
        expander;
        dimExpander;
        inflateRay;
        
        % Path interpolation parameter
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
            
            % Initialise the pictures classifier
            pictures = {'pictures/dog.png', ...
                        'pictures/plant.png', ...
                        'pictures/pumpkin.png', ...
                        'pictures/trashcan.png', ...
                        'pictures/trike.png'};      
            obj.matcher = Matcher(pictures);
        end
        
        function work(obj)
            %%%%%%%%%% DEBUG %%%%%%%%%%%%%
            %load('map.mat');
            %obj.map = robotMap;

            % Do a barrel roll
            obj.barrelRoll();
          
            % Discover the ground map
            obj.discoverMap();
            
            %%%%%%%%%% DEBUG %%%%%%%%%%%%%
            robotMap = obj.map;
            save('map.mat', 'robotMap');
            
            % Seek for bins
            obj.discoverBins();            
        end
    end
    
    
    methods (Access='protected')
        function discoverMap(obj)
            % Update data
            obj.controller.updateData();
            obj.map.update(obj.robot.hokuyo.hits, obj.robot.hokuyo.voidPoints);
            
            while(obj.setNextExplorationTrajectory())
                obj.map.print();
                obj.drive(true);
            end
            obj.map.print();
        end
        
        function barrelRoll(obj)  
            % Plan a barell roll
            curOri = obj.robot.se2.T;
            ori(:,:,1) = curOri * rotz(pi/2);
            ori(:,:,2) = curOri * rotz(pi);
            ori(:,:,3) = curOri * rotz(3*pi/2);
            ori(:,:,4) = curOri * rotz(2*pi);
            
            % Perform the poses
            obj.controller.drivePose(obj.map, ori, obj.dt);
        end
        
        function drive(obj, reduce)
            % Transform path into waypoints if asked
            if reduce
                [wayPoints(:,1), wayPoints(:,2)] = reducem(obj.path(:,1), obj.path(:,2), 0.1);
                wayPoints = wayPoints(2:end,:);
            else
                wayPoints = obj.path;
            end
            
            % Perform the path
            obj.controller.drivePath(obj.map, wayPoints, obj.dt, obj.at)
        end
        
        function [circles, radiuses] = findCircles(obj, mapThreshold, inflateRay, radiusRange)
            % Dilation of the map
            mp = full(obj.map.content >= mapThreshold);
            closedMap = imclose(mp, strel('disk', inflateRay));
            
            [circles, radiuses] = imfindcircles(closedMap,radiusRange, 'Sensitivity', 0.88,'EdgeThreshold',0.08);
            circles = round(circles);
            
            % Invert coordinate to match row col format
            circles = [circles(:,2), circles(:,1)];
        end
        
        function pathCost = getPathCost(obj, path)
            pathCost = 0;
            p1 = path(1,:);

            for o = 2:numrows(path)
                p2 = path(o,:);
                pathCost = pathCost + pdist2(p1, p2, 'euclidean');
                p1 = p2;
            end
        end
        
        function idx = nPointsClose(obj, point, threshold, numberOfPoints, circleRadius)
            function isClear = clearSight(x, y)
                % Determine if there can be a straight line between the two
                % given points
                mat = zeros(size(obj.map.content, 1), size(obj.map.content, 2), 'uint8');
                nPoints = max(abs(x(1) - y(1)), abs(x(2) - y(2)))+1;

                cIndex = round(linspace(x(2), y(2), nPoints));
                rIndex = round(linspace(x(1), y(1), nPoints));
                index = sub2ind(size(mat), rIndex, cIndex);
                mat(index) = 255;

                cirlceToCarve = strel('disk', double(round(circleRadius * 0.6)));
                
                % Closing of the map
                mapToProcess = full(obj.map.content >= obj.classTreshold);
                mapToProcess = imclose(mapToProcess, strel('disk', 5));
            
                [row, col] = size(cirlceToCarve.Neighborhood);
                subRow = round((row - 1)/2);
                subCol = round((col - 1)/2);

                coordMin = point - [subRow, subCol];
                coordMax = point + [row - subRow - 1, col - subCol - 1];

                % Carve the inflated robot position to avoid the robot to be
                % 'ate' by the obstacle inflate.
                circleZone = mapToProcess(coordMin(1):coordMax(1), coordMin(2):coordMax(2));
                circleZone(cirlceToCarve.Neighborhood) = 0;
                mapToProcess(coordMin(1):coordMax(1), coordMin(2):coordMax(2)) = circleZone;

                imshowpair(full(mapToProcess),mat)
                
                if any(mapToProcess(logical(mat)))
                    isClear = false;
                else
                    isClear = true;
                end
            end
            
            [~, v] = obj.prm.graph.distances(point);
            idx = [];
            
            % Get robot pose information
            [x, y, ~] = transl(obj.robot.pose);
            [rposR, rposC] = obj.map.world2Map(x, y);
            
            for o = 1:numcols(v)
                goal = obj.prm.graph.coord(v(o));
                
                % Keep the goal if there is a clear line sight to the goal
                % to take the picture
                if circleRadius == 0
                    isInSight = true;
                else
                    isInSight = clearSight(point, goal');
                end
                if ~obj.prm.isoccupied(goal') && pdist2(point, goal', 'euclidean') >= threshold && isInSight
                    % Plan the path to the circle coordinates
                    idx = [idx; goal'];
                    if numrows(idx) >= numberOfPoints
                        break;
                    end
                end
            end
        end
        
        function idx = closestCircle(obj, start, circles, threshold)
            distancePath = Inf;
            for m = 1:numrows(circles)
                circle = circles(m,:);
                goal = obj.nPointsClose([circle(2), circle(1)], threshold, 1, 0);

                % Plan the path to the circle coordinates
                newPathToCircle = obj.prm.query(start, goal');
                newDistancePath = obj.getPathCost(newPathToCircle);

                if newDistancePath < distancePath
                    distancePath = newDistancePath;
                    idx = m;
                end
            end
        end        
        
        function matchCircle(obj, cirlceCenter, circleRadius)
            % Get the n closest point of the given circle center
            closests = obj.nPointsClose(cirlceCenter, round(obj.inflateRay) * 3, 3, circleRadius);
            
            % Get robot pose information
            [x, y, ~] = transl(obj.robot.pose);
            [rposR, rposC] = obj.map.world2Map(x, y);
            
            for n = 1:numrows(closests)
                closePoint = closests(n,:);
        
                obj.path = obj.prm.query([rposC, rposR], [closePoint(2), closePoint(1)]);
                [obj.path(:,1), obj.path(:,2)] = obj.map.map2World(obj.path(:,2), obj.path(:,1));

                 % Drive to the closest detected circle
                obj.drive(true);
                
                % Update robot pose information after the drive
                [x, y, ~] = transl(obj.robot.pose);
                [rposR, rposC] = obj.map.world2Map(x, y);

                % Plan the poses to face the center of the circle
                circleAngle = atan2(cirlceCenter(1) - rposR, cirlceCenter(2) - rposC);
                angle = wrapToPi(circleAngle - obj.robot.se2.angle());                
                ori = obj.robot.se2.T * rotz(angle);
                obj.controller.drivePose(obj.map, ori, obj.dt);

                % Take a picture and match it against the pictures
                image = obj.controller.takePicture(); 
                imgMatches = obj.matcher.imageMatch(image);
                
                thresholdMatching = 170;

                if any(imgMatches > thresholdMatching)
                    disp('Strong match found');
                    [v, I] = max(imgMatches);
                    obj.matcher.imgList{I}
                    imgMatches
                    break;
                else
                    imgMatches
                end
                
                disp('The search continue...');
            end
        end
        
        function discoverBins(obj)
            % Find bins
            [circles, radiuses] = obj.findCircles(4, 5, [7 40]);
            
            % Number of points to place on the map
            numberPoints = 300;
            
            % Compute the random tree navigation structure
            [x, y, ~] = transl(obj.robot.pose);
            [rposR, rposC] = obj.map.world2Map(x, y);
            inflatedMap = obj.map.inflate(obj.classTreshold, double(round(obj.inflateRay)), [rposR, rposC]);
            obj.prm = PRM(inflatedMap, 'npoints', numberPoints);
            obj.prm.plan();
            
            threshold = round(obj.inflateRay) * 2;
            
            % For each circle shape detected on the map
            for n = 1:numrows(circles)
                % Get robot position
                [x, y, ~] = transl(obj.robot.pose);
                [rposR, rposC] = obj.map.world2Map(x, y);
                
                % Get the closest points circle 
                idxes = obj.closestCircle([rposC, rposR], circles, threshold);
                closestCircle = circles(idxes,:);
                circleRadius = double(round(radiuses(idxes)));
                
                obj.matchCircle(closestCircle, circleRadius);
                
                circles(idxes,:) = []; 
            end
        end
        
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

