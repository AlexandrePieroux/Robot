classdef Map < handle
    
    properties
        offset;
        resolution;
        content;
    end
  
    methods
        function obj = Map(resolution)
            obj.offset = [0 0];
            obj.resolution = 1/resolution;
            obj.content = sparse(0);
        end
        
        function insertRobot(obj, robot)
            % Get the effective row and col in the map of the robot pose
            [x, y, ~] = transl(robot.pose);
            obj.update([x - robot.length / 2, y - robot.width / 2])
            [row, col] = obj.world2Map(x, y);
            
            % Compute the limits of the space the robot take in the map
            mapRowMin = abs(row) - round(robot.length * obj.resolution / 2);
            mapRowMax = abs(row) + round(robot.length * obj.resolution / 2);
            
            mapColMin = abs(col) - round(robot.width * obj.resolution / 2);
            mapColMax = abs(col) + round(robot.width * obj.resolution / 2);
            
            % Set the robot in the map
            obj.content(mapRowMin:mapRowMax, mapColMin:mapColMax) = -Inf;
        end
        
        function update(obj, hits)
            % Retrive the smallest coordinate to use it as offset
            % Only minimum here matter.
            minHitX = round(min(hits(:,1)) * obj.resolution);
            minHitY = round(min(hits(:,2)) * obj.resolution);
            
            % Compute the offset difference
            offsetDiffX = obj.offset(1) - min(minHitX, obj.offset(1));
            offsetDiffY = obj.offset(2) - min(minHitY, obj.offset(2));
            
            % Rebuild sparse matrix with offset
            [i, j, s] = find(obj.content);
            if ~isempty([i, j , s])
                i = i + offsetDiffX;
                j = j + offsetDiffY;
                obj.content = sparse(double(i), double(j), s);
            end
            
            % Keep the smallest as offset
            obj.offset = min([minHitX minHitY], obj.offset);

            % Rescale the points for them to be in the map
            % As the matrices only support positive integer indexing we
            % scale the result and offset it.
            [mapHits(:,1), mapHits(:,2)] = obj.world2Map(hits(:,1), hits(:,2));
            mapHits = unique(mapHits, 'rows');
            
            % Insert hokuyo data in the map
            for n = 1:size(mapHits, 1)
                if mapHits(n,1) > size(obj.content, 1) || mapHits(n,2) > size(obj.content, 2)
                    obj.content(mapHits(n,1), mapHits(n,2)) = 1;
                else
                    obj.content(mapHits(n,1), mapHits(n,2)) = obj.content(mapHits(n,1), mapHits(n,2)) + 1;
                end
            end
        end
        
        function [x, y] = map2World(obj, row, col)
            x = (row - abs(obj.offset(1)) - 1)/obj.resolution;
            y = (col - abs(obj.offset(2)) - 1)/obj.resolution;
        end
        
        function [row, col] = world2Map(obj, x, y)
            row = round(x * obj.resolution + abs(obj.offset(1)) + 1);
            col = round(y * obj.resolution + abs(obj.offset(2)) + 1);
        end
        
        function print(obj)
            figure(2);
            colormap('hot');
            imagesc(obj.content);
            caxis([-10,10]);
            colorbar;
            drawnow;
        end
    end
end

