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
            load('map.mat');
            obj.map = robotMap;

            % Do a barrel roll
            %obj.barrelRoll();
          
            % Discover the ground map
            %obj.discoverMap();
            
            %%%%%%%%%% DEBUG %%%%%%%%%%%%%
            %robotMap = obj.map;
            %save('map.mat', 'robotMap');
            
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
        
        function idx = nPointsClose(obj, point, threshold, numberOfPoints)
            [~, v] = obj.prm.graph.distances(point);
            idx = [];
            
            for o = 1:numcols(v)
                goal = obj.prm.graph.coord(v(o));
                if pdist2(point, goal', 'euclidean') >= threshold
                    idx = [idx; goal'];
                    if numrows(idx) >= numberOfPoints
                        break;
                    end
                end
            end
        end
        
        function idx = closestCircle(obj, start, circles, threshold)
            function pathCost = getPathCost(path)
                pathCost = 0;
                p1 = path(1,:);

                for o = 2:numrows(path)
                    p2 = path(o,:);
                    pathCost = pathCost + pdist2(p1, p2, 'euclidean');
                    p1 = p2;
                end
            end
            
            distancePath = Inf;
            for m = 1:numrows(circles)
                circle = circles(m,:);
                goal = obj.nPointsClose([circle(2), circle(1)], threshold, 1);

                % Plan the path to the circle coordinates
                newPathToCircle = obj.prm.query(start, goal');
                newDistancePath = getPathCost(newPathToCircle);

                if newDistancePath < distancePath
                    distancePath = newDistancePath;
                    idx = m;
                end
            end
        end        
        
        function matchCircle(obj, cirlceCenter)
            closests = obj.nPointsClose(cirlceCenter, round(obj.inflateRay) * 10, 5);
            for n = 1:numrows(closests)
                close = closests(n,:);
                
                % Get robot pose information
                [x, y, ~] = transl(obj.robot.pose);
                [rposR, rposC] = obj.map.world2Map(x, y);

                obj.path = obj.prm.query([rposC, rposR], close);
                [obj.path(:,1), obj.path(:,2)] = obj.map.map2World(obj.path(:,2), obj.path(:,1));

                 % Drive to the closest detected circle
                obj.drive(true);

                % Plan the poses to face the center of the circle
                ori = obj.robot.se2.T;
                robotAngle = tr2rpy(ori);
                robotAngle = robotAngle(3);

                circleAngle = atan2(cirlceCenter(1) - rposR, cirlceCenter(2) - rposC);
                angle = circleAngle - robotAngle;                
                ori = ori * rotz(angle);
                obj.controller.drivePose(obj.map, ori, obj.dt);

                % Take a picture and match it against the pictures
                image = obj.controller.takePicture(); 
                matches = obj.matcher.imageMatch(image);
                
                thresholdMatching = 170;

                if matches > thresholdMatching
                    disp('Strong match found');
                    break;
                end
                
                disp('The search continue...');
            end
        end
        
        function discoverBins(obj)
            % Find bins
            mp = full(obj.map.content >= 4);
            dilatedMap = imdilate(mp, strel('disk', 5));
            erodeMap = imerode(dilatedMap, strel('disk', 5));
            
            circles = imfindcircles(erodeMap,[7 40], 'Sensitivity', 0.88,'EdgeThreshold',0.08);
            circles = round(circles);
            
            % Invert coordinate to match row col format
            circles = [circles(:,2), circles(:,1)];
            
            % Number of points to place on the map
            numberPoints = 300;
            
            % Compute the random tree navigation structure
            [x, y, ~] = transl(obj.robot.pose);
            [rposR, rposC] = obj.map.world2Map(x, y);
            inflatedMap = obj.map.inflate(obj.classTreshold, double(round(obj.inflateRay)), [rposR, rposC]);
            obj.prm = PRM(inflatedMap, 'npoints', numberPoints);
            obj.prm.plan();
            
            threshold = round(obj.inflateRay) * 10;
            
            % For each circle shape detected on the map
            for n = 1:numrows(circles)
                % Get shortest path
                [x, y, ~] = transl(obj.robot.pose);
                [rposR, rposC] = obj.map.world2Map(x, y);
                idxes = obj.closestCircle([rposC, rposR], circles, threshold);
                closest = circles(idxes,:);

                obj.matchCircle(closest);
                
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

