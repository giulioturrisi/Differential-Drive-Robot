classdef PriorityQueue < handle
    %
    %
    
    properties %(Access = private)
        % queue of states
        queue;
    end
    
    methods
        % PriorityQueue constructor
        function obj = PriorityQueue()
            obj.queue = State.empty(1, 0);
        end
        
        
        % find in the queue vertex s
        % if not exists return -1
        function pos = find(obj, s)
            pos = -1;
            for i=1:length(obj.queue)
                if all(s==obj.queue(i))
                    pos=i;
                    return;
                end
            end
        end
        
        % check if the queue is empty
        function res = isEmpty(obj)
            res = isempty(obj.queue);
        end
        
        % check if the queue has vertex s
        function [isInside, pos] = has(obj, s)
            isInside = true;
            pos = obj.find(s);
            if pos == -1
                isInside = false;
            end
        end
        
        
        % insert in the queue vertex s with value k
        % if already exists change priority of s from k (old) to k
        function insert(obj, s, k)
            s.k = k; % ==> s.Kold = k
            
            pos = obj.find(s);
            if pos == -1
                obj.queue(end+1) = s;
                
                % TODO
                if s.state == State.UNKNOWN || s.state == State.EMPTY
                    s.state = State.VISITED;
                end
            end
        end
        
        % remove from the queue vertex s
        function remove(obj, s)
            pos = obj.find(s);
            obj.queue(pos) = [];
        end
        
        % return a vertex with the smallest priority k
        % optional minV
        function [minS, minV] = top(obj)
            minV = [];
            minS = [];
            for elem=obj.queue
                if isempty(minV) || min2(elem.k, minV)
                    minV = elem.k;
                    minS = elem;
                end
            end
        end
        
        % return the smallest priority k of all vertices
        % if empty return [+inf, +inf]
        function minV = topKey(obj)
            minV = [inf, inf]; % [];
            for elem=obj.queue
                if isempty(minV) || min2(elem.k, minV)
                    minV = elem.k;
                end
            end
        end
        
        % delete from the queue the vertex with the smallest priority k
        % and return it, optional k
        function [s, k] = pop(obj)
            [s, k] = top(obj);
            obj.remove(s);
        end
        
        % pop the element in position pos
        function [s, k] = extract(obj, pos)
            s = obj.queue(pos);
            k = s.k;
            obj.remove(s);
        end
        
        
        % debug plot
        function plot(obj)
            disp("Priority Queue:")
            for elem=obj.queue
                disp(strjoin(["    [", elem.x, ", ", elem.y, "] --> [", elem.k, "]"]))
            end
        end
    end
end


