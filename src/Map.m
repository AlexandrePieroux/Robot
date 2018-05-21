classdef Map < handle
    
    properties
        offset;
        resolution;
        content;
    end
  
    methods
        function obj = Map(resolution)
            obj.offset = [0, 0];
            obj.resolution = 1/resolution;
            obj.content = sparse(0);
        end
        
        function insertRobot(obj, robot)
            % Get the effective row and col in the map of the robot pose
            [x, y, ~] = transl(robot.pose);
            minX = x - robot.length/2;
            minY = y - robot.length/2;
            
            maxX = x + robot.length/2;
            maxY = y + robot.length/2;
            
            % Updating the offset if needed
            minXOff = round(minX * obj.resolution) - 1;
            minYOff = round(minY * obj.resolution) - 1;
            
            % Compute the difference
            offsetDiffCol = obj.offset(2) - min(minXOff, obj.offset(2));
            offsetDiffRow = obj.offset(1) - min(minYOff, obj.offset(1));
            
            % Rebuild sparse matrix with offset
            [r, c, v] = find(obj.content);
            if ~isempty([r, c])
                r = r + offsetDiffRow;
                c = c + offsetDiffCol;
                obj.content = sparse(double(r), double(c), v);
            end
            
            % Keep the smallest as offset
            obj.offset = min([minYOff, minXOff], obj.offset);
            
            % Compute the limits of the space the robot take in the map
            [rowMin, colMin] = obj.world2Map(minX, minY);
            [rowMax, colMax] = obj.world2Map(maxX, maxY);
                        
            % Set the robot in the map
            obj.content(rowMin:rowMax, colMin:colMax) = -Inf;
        end
        
        function update(obj, hits, voidPoints)
            if isempty(hits) || isempty(voidPoints)
                return
            end
            
            % Retrive the smallest coordinate to use it as offset
            % Only minimum here matter.
            minHitX = round(min(hits(:,1)) * obj.resolution) - 1;
            minHitY = round(min(hits(:,2)) * obj.resolution) - 1;
            
            minVoidX = round(min(voidPoints(:,1)) * obj.resolution) - 1;
            minVoidY = round(min(voidPoints(:,2)) * obj.resolution) - 1;
            
            minX = min(minHitX, minVoidX);
            minY = min(minHitY, minVoidY);
            
            % Compute the offset difference
            offsetDiffCol = obj.offset(2) - min(minX, obj.offset(2));
            offsetDiffRow = obj.offset(1) - min(minY, obj.offset(1));
            
            % Rebuild sparse matrix with offset
            [r, c, v] = find(obj.content);
            if ~isempty([r, c])
                r = r + offsetDiffRow;
                c = c + offsetDiffCol;
                obj.content = sparse(double(r), double(c), v);
            end
            
            % Keep the smallest as offset
            obj.offset = min([minY, minX], obj.offset);

            % Rescale the points for them to be in the map
            % As the matrices only support positive integer indexing we
            % scale the result and offset it.
            [mapHits(:,1), mapHits(:,2)] = obj.world2Map(hits(:,1), hits(:,2));
            mapHits = unique(mapHits, 'rows');
            
            [mapVoids(:,1), mapVoids(:,2)] = obj.world2Map(voidPoints(:,1), voidPoints(:,2));
            mapVoids = unique(mapVoids, 'rows');
            
            % Insert hokuyo data in the map
            for n = 1:size(mapHits, 1)
                if mapHits(n,1) > size(obj.content, 1) || mapHits(n,2) > size(obj.content, 2)
                    obj.content(mapHits(n,1), mapHits(n,2)) = 2;
                else
                    obj.content(mapHits(n,1), mapHits(n,2)) = obj.content(mapHits(n,1), mapHits(n,2)) + 2;
                end
            end
            
            % Insert voids data in the map
            for n = 1:size(mapVoids, 1)
                if mapVoids(n,1) > size(obj.content, 1) || mapVoids(n,2) > size(obj.content, 2)
                    obj.content(mapVoids(n,1), mapVoids(n,2)) = -1;
                else
                    obj.content(mapVoids(n,1), mapVoids(n,2)) = obj.content(mapVoids(n,1), mapVoids(n,2)) - 1;
                end
            end
        end
        
        function dilatedMap = inflate(obj, threshold, radius, robotCoord)
            
            % Inflate the map obstacles
            mapToProcess = double(obj.content >= threshold);
            dilatedMap = imdilate(full(mapToProcess), strel('disk', radius));
            
            % Dilate the robot position
            radius = round(radius * 0.75);
            robotInflateShape = strel('disk', radius);
            
            [row, col] = size(robotInflateShape.Neighborhood);
            subRow = round((row - 1)/2);
            subCol = round((col - 1)/2);
            
            coordMin = robotCoord - [subRow, subCol];
            coordMax = robotCoord + [row - subRow - 1, col - subCol - 1];
            
            % Carve the inflated robot position to avoid the robot to be
            % 'ate' by the obstacle inflate.
            robotZone = dilatedMap(coordMin(1):coordMax(1), coordMin(2):coordMax(2));
            robotZone(robotInflateShape.Neighborhood) = 0;
            dilatedMap(coordMin(1):coordMax(1), coordMin(2):coordMax(2)) = robotZone;
        end
        
        function [x, y] = map2World(obj, row, col)
            y = (row - abs(obj.offset(1)))/obj.resolution;
            x = (col - abs(obj.offset(2)))/obj.resolution;
        end
        
        function [row, col] = world2Map(obj, x, y)
            row = round(y * obj.resolution + abs(obj.offset(1)));
            col = round(x * obj.resolution + abs(obj.offset(2)));
        end
        
        function print(obj)
            figure(2);
            clf(2);
            colormap('hot');
            imagesc(obj.content);
            caxis([-10,10]);
            colorbar;
            drawnow;
        end
    end
end

