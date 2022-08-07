classdef DState < handle
    properties (SetAccess = private)
        x
        y
    end
    
    properties
        parent
        state
        tag
        
        h
        k
    end

    methods
        function obj = DState(x, y)
            arguments
                x {}
                y {}
            end
            obj.x = x;
            obj.y = y;
            obj.parent = DState.empty;
            obj.state = DMapState.EMPTY;
            obj.tag = DStateTag.NEW;
            
            obj.h = 0;
            obj.k = 0;
        end

        function res = cost(obj, state)
            if obj.state == DMapState.OBSTACLE || state.state == DMapState.OBSTACLE
                res = Inf;
            else
                res = sqrt((obj.x - state.x)^2 + (obj.y - state.y)^2);
            end
        end

        function e = eq(obj, s)
            e = obj.x == s.x && obj.y == s.y;
        end
    end
end