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
        
        function update(obj, data)
            % Retrive the smallest coordinate to use it as offset
            % Only minimum here matter.
            minHitX = round(min(data.hits(:,1)) * obj.resolution);
            minHitY = round(min(data.hits(:,2)) * obj.resolution);
            
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
            
            % Keep the smallest offset
            obj.offset = min([minHitX minHitY], obj.offset);

            % Rescale the points for them to be in the map
            % As the matrices only support positive integer indexing we
            % scale the result and offset it.
            hits(:,1) = round(data.hits(:,1) * obj.resolution + abs(obj.offset(1)));
            hits(:,2) = round(data.hits(:,2) * obj.resolution + abs(obj.offset(2)));
            hits = unique(hits, 'rows');
                        
            hits(:,1) = hits(:,1) + 1;
            hits(:,2) = hits(:,2) + 1;
            
            sizeRow = size(obj.content, 1);
            sizeColumn = size(obj.content, 2);
           
            % Insert hokuyo data in the map
            for n  = 1:length(hits)
                if hits(n,1) > sizeRow || hits(n,2) > sizeColumn
                    obj.content(hits(n,1), hits(n,2)) = 1;
                else
                    obj.content(hits(n,1), hits(n,2)) = obj.content(hits(n,1), hits(n,2)) + 1;
                end
            end
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

