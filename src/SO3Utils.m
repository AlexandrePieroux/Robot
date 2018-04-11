classdef SO3Utils
    
    methods(Static)
        function angle = getZAngle(trans)
            z = trans(1,1);
            if z > 1
                z = 1;
            else 
                if z < -1
                    z = -1;
                end
            end
            if trans(1, 2) > 0
                angle = -acos(z);
            else
                angle = acos(z);
            end
        end        
    end
    
end

