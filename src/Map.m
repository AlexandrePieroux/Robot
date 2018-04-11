classdef Map < handle
    
    properties
        content;
        size;
        scale;
        radianPrecision;
        increasingFactor;
        offset;
    end
  
    methods
        function obj = Map(centerCoor, width, length, scale, radianPrecision, increasingFactor)
            x = increasingFactor * scale * 2 + 1;
            obj.size = [x, x];
            obj.scale = scale;
            obj.radianPrecision = radianPrecision;
            centerCoor = centerCoor * scale;
            centerPos = increasingFactor * scale + 1;
            obj.offset = [centerPos-centerCoor(1), centerPos-centerCoor(2)];
            obj.content = zeros(x);
            obj.content(centerPos-ceil(scale * width / 2):centerPos+ceil(scale * width / 2), centerPos-ceil(scale * length / 2):centerPos+ceil(scale * length / 2)) = -Inf;
            obj.increasingFactor = increasingFactor;
        end
        
        function update(obj, data)
            hits(:,1) = ceil(data.hits(:,1) * obj.scale + obj.offset(1) - 0.5);
            hits(:,2) = ceil(data.hits(:,2) * obj.scale + obj.offset(2) - 0.5);
            hits = unique(hits, 'rows');
            voids(:,1) = ceil(data.surface(:,1) * obj.scale + obj.offset(1) - 0.5);
            voids(:,2) = ceil(data.surface(:,2) * obj.scale + obj.offset(2) - 0.5);
            voids = unique(voids, 'rows');
            % ?OPTIMIZE?
            voids = setdiff(voids, hits, 'rows');

            % If needed, resize the map
            % Check if we have a large enough content matrix
            mins = min([hits; voids]);
            maxs = max([hits; voids]);

            % ?OPTIMIZE?
            update = false;
            if mins(1) < 1
                update = true;
                moreXBefore = obj.increasingFactor * obj.scale;
            else
                moreXBefore = 0;
            end

            if mins(2) < 1
                update = true;
                moreYBefore = obj.increasingFactor * obj.scale;
            else
                moreYBefore = 0;
            end

            if maxs(1) > obj.size(1)
                update = true;
                moreXAfter = obj.increasingFactor * obj.scale;
            else
                moreXAfter = 0;
            end

            if maxs(2) > obj.size(2)
                update = true;
                moreYAfter = obj.increasingFactor * obj.scale;
            else
                moreYAfter = 0;
            end

            if update 
                % Update the content matrix
                obj.content = padarray(obj.content, [moreXBefore, moreYBefore], 0, 'pre');
                obj.content = padarray(obj.content, [moreXAfter, moreYAfter], 0, 'post');

                % Update the offset
                obj.offset = obj.offset + [moreXBefore, moreYBefore];
                hits(:,1) = hits(:,1) + moreXBefore;
                hits(:,2) = hits(:,2) + moreYBefore;
                voids(:,1) = voids(:,1) + moreXBefore;
                voids(:,2) = voids(:,2) + moreYBefore;

                % Update the size
                obj.size = obj.size + [moreXBefore+moreXAfter, moreYBefore+moreYAfter];
            end

            % Insert hokuyo data in the new map
            for i  = 1:length(hits)
                obj.content(hits(i,1), hits(i,2)) = obj.content(hits(i,1), hits(i,2)) + 1;
            end

            for i = 1:length(voids)
                obj.content(voids(i,1), voids(i,2)) = obj.content(voids(i,1), voids(i,2)) - 1;
            end
        end
        
        function boolean = inOrientation(obj, this, that)
            thisAngle = SO3Utils.getZAngle(this);
            thatAngle = SO3Utils.getZAngle(that);
            
            boolean = abs(angdiff(thisAngle, thatAngle)) <= obj.radianPrecision;
        end
        
        function print(obj)
            figure(2);
            colormap('hot');
            imagesc(obj.content);
            caxis([-400,400]);
            colorbar;
            drawnow;
        end
    end
end

