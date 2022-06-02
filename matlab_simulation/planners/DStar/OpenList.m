classdef OpenList < handle
    properties (Access = private)
        actualList;
    end

    methods (Access = private)
        function res = is_empty(obj)
            res = isempty(obj.actualList);
        end
    end
    
    methods (Access = public)
        function obj = OpenList(initialState)
            obj.actualList = initialState;
        end

        function insert(obj, state, h_new)
            if state.tag == DStateTag.NEW
                state.k = h_new;
            elseif state.tag == DStateTag.OPEN
                state.k = min(state.k, h_new);
            elseif state.tag == DStateTag.CLOSED
                state.k = min(state.h, h_new);
            end
            state.h = h_new;
            state.tag = DStateTag.OPEN;

            if isempty(obj.actualList)
                obj.actualList = state;
            else
                obj.actualList(end+1) = state;
            end
        end

        function remove(obj, state)
            if state.tag == DStateTag.OPEN
                state.tag = DStateTag.CLOSED;
            end
            pos = obj.find(state);
            obj.actualList(pos) = [];
        end
        
        function pos = find(obj, s)
            % find in the queue vertex s
            % if not exists return -1
            pos = -1;
            for i=1:size(obj.actualList, 2)
                if s.eq(obj.actualList(i))
                    pos=i;
                    return;
                end
            end
        end
        
        function h = has(obj, s)
            if obj.find(s) == -1
                h = false;
            else
                h = true;
            end
        end

        function k = get_kmin(obj)
            if isempty(obj.actualList)
                k = -1;
            else
                k = Inf;
                for e=obj.actualList
                    if e.k < k
                        k = e.k;
                    end
                end
            end
        end

        function [k, s] = min_state(obj)
            if isempty(obj.actualList)
                k = -1;
                s = State.empty(1, 0);
            else
                k = Inf;
                for e=obj.actualList
                    if e.k < k
                        k = e.k;
                        s = e;
                    end
                end
            end
        end
    end
end


