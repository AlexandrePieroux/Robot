classdef Brain < handle
    
    properties
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
        function obj = Brain()
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
        
        function drive(obj)
            obj.updateData();
            obj.map.update(obj.robot.hokuyo);
            
            % THERE
            while(obj.updateWheelsTrajectory())
                obj.setWheelsSpeed();
                obj.updateData();
                obj.map.update(obj.robot.hokuyo);
            end

            obj.controller.setWheelsSpeed(0, 0, 0, 0);
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
        
        function setWheelsSpeed(obj)
            targetPose = obj.robot.pose \ obj.wheelsTrajectory(:,:,1);
            targetAngle = SO3Utils.getZAngle(targetPose);
            
            %Horizontal and vertical speeds
            if obj.map.onCell(obj.robot.pose, obj.wheelsTrajectory(:,:,1))
                vx = 0;
                vy = 0;
            else
                [vy, vx, ~] = transl(targetPose);
                v = sqrt(vx^2+vy^2);
                while v < obj.robot.wheels.maxlv,
                    vx=vx*2;
                    vy=vy*2;
                    v = sqrt(vx^2+vy^2);
                end
                vx = vx * obj.robot.wheels.maxlv / v;
                vy = vy * obj.robot.wheels.maxlv / v;
            end
            
            %Angular speed
            if(obj.map.inOrientation(obj.robot.pose, obj.wheelsTrajectory(:,:,1)))
                va = 0;
            else
                va = targetAngle / pi;
                va = (va/abs(va)) * obj.robot.wheels.minav + va * (obj.robot.wheels.maxav - obj.robot.wheels.minav);
            end
            
            obj.controller.setWheelsSpeed(-vx-vy+va, -vx+vy+va, -vx+vy-va, -vx-vy-va);
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
        
        function generateWheelsTrajectory(obj, ds, start, isRobot, rpos)
            if(isRobot)
                trajPath = rot90(ds.path(start), 2);
                trajPath = [trajPath; fliplr(start)];
                trajPath = trajPath(2:end,:); 
            else
                trajPath = fliplr(ds.path(start));
            end
                        
            pathSize = size(trajPath);
            if pathSize(1) > 1
                precDirection = obj.getDirection(rpos, trajPath(1,:));
                precPoint = trajPath(1,:);
                j = 1;
                i = 2;
                while(i < pathSize(1))
                    direction = obj.getDirection(precPoint, trajPath(i,:));
                    if(not(strcmp(direction, precDirection) == 0))
                        obj.wheelsTrajectory(:,:,j) = obj.getPose(precDirection, precPoint);
                        j = j + 1;
                    end
                    precDirection = direction;
                    precPoint = trajPath(i,:);
                    i = i + 1;
                end
                obj.wheelsTrajectory(:,:,j) = obj.getPose(precDirection, precPoint);
            else
                obj.wheelsTrajectory = [];
            end
        end
        
        function pose = getPose(obj, direction, target)
            pose = transl((target(1) - obj.map.offset(1)) / obj.map.scale, (target(2) - obj.map.offset(2)) / obj.map.scale, 0);
            switch(direction)
                case 'up'
                    pose = pose * trotz(pi/2 + pi/2);
                case 'upleft'
                    pose = pose * trotz(3*pi/4 + pi/2);
                case 'upright'
                    pose = pose * trotz(pi/4 + pi/2);
                case 'down'
                    pose = pose * trotz(-pi/2 + pi/2);
                case 'downleft'
                    pose = pose * trotz(-pi/4 + pi/2);
                case 'downright'
                    pose = pose * trotz(-3*pi/4 + pi/2);
                case 'right'
                    pose = pose * trotz(-pi + pi/2);
                case 'left'
                    pose = pose * trotz(pi + pi/2);
                    
            end
        end
        
        function direction = getDirection(~, source, target)
            if source(2) < target(2)
                %Up
                if source(1) < target(1)
                    %Right
                    direction = 'upright';
                else
                    if source(1) > target(1)
                        %Left
                        direction = 'upleft';
                    else
                        direction = 'up';
                    end
                end
            else
                if source(2) > target(2)
                    %Down
                    if source(1) < target(1)
                        %Right
                        direction = 'downright';
                    else
                        if source(1) > target(1)
                            %Left
                            direction = 'downleft';
                        else
                            direction = 'down';
                        end
                    end
                else 
                    if source(1) < target(1)
                        %Right
                        direction = 'right';
                    else
                        if source(1) > target(1)
                            %Left
                            direction = 'left';
                        else
                            direction = 'none';
                        end
                    end
                end
            end
        end
    end
    
end

