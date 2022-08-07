classdef DMap < handle
    % Class to keep and work with the map
    properties
        % num of map's rows
        row %double {mustBePositive, mustBeInteger}
        % num of map's cols
        col %double {mustBePositive, mustBeInteger}
        
        % row x col matrix of State
        map %(:, :) {}
    end
    
    methods (Access = private)
        function init_map(obj)
            obj.map = DState.empty(1, 0);
            for i=1:obj.row
                tmp = DState.empty(0, 1);
                for j=1:obj.col
                    tmp(j) = DState(i, j);
                end
                obj.map = [obj.map; tmp];
            end
        end
    end

    methods
        % map constructor
        function obj = DMap(row, col, obstacles)
            arguments
                % num of map's rows
                row %double {mustBePositive, mustBeInteger}
                % num of map's cols
                col %double {mustBePositive, mustBeInteger}
                
                % matrix 2 x N list of obstacles
                % | x1, x2, x3, ...
                % | y1, y2, y3, ...
                obstacles %(2, :) {mustBePositive, mustBeInteger} = []
            end
            
            obj.row = row;
            obj.col = col;
            
            obj.init_map()
            obj.setObstacles(obstacles);
        end

        % set a list of obstacles inside the map
        function setObstacles(obj, point_list)
            arguments
                obj
                
                % matrix 2 x N list of obstacles
                % | x1, x2, x3, ...
                % | y1, y2, y3, ...
                point_list %(2, :) {mustBePositive, mustBeInteger}
            end
            
            for point=point_list
                if obj.isInside(point(1), point(2))
                    obj.map(point(1), point(2)).state = DMapState.OBSTACLE;
                end
            end
        end
        
        % check if (x, y) is inside the map
        function res = isInside(obj, x, y)
            arguments
                obj
                
                % x and y position of the position to check
                x %double {mustBeNumeric}
                y %double {mustBeNumeric}
            end
            
            if x < 1 || x > obj.row
                res = false;
                return;
            end
            if y < 1 || y > obj.col
                res = false;
                return;
            end
            res = true;
        end
        
        % check if (x, y) is an obstacle
        function res = isObstacle(obj, x, y)
            arguments
                obj
                
                % x and y position of the position to check
                x %double {mustBeNumeric}
                y %double {mustBeNumeric}
            end
            
            res = obj.map(x, y).state == DMapState.OBSTACLE;
        end

        function s = neighbors(obj, X, moves)
            arguments
                obj {}
                X {}
                moves {} = [[1, 0], [1, 1], [0, 1], [-1, 0], [-1, -1], [0, -1], [-1, 1], [1, -1]];
            end
            s = [DState.empty];
            pos = [X.x; X.y];
            
            for m=moves
                succ_pos = pos + m;
                x = succ_pos(1);
                y = succ_pos(2);

                if obj.isInside(x, y) && ~obj.isObstacle(x, y)
                    s(end+1) = obj.map(x, y);
                end
            end
        end

        function addRandomObstaclesNear(~, state)
            p = randi([0, 3], 1, 1);
            if p == 0 % random obstacle is added with 25% probability
                s = state.parent;
                if s.state ~= DMapState.GOAL
                    s.state = DMapState.OBSTACLE;
                end
            end
        end

        % generate the map image
        function rgbImage = buildImage(obj, highlightedState)
            rgbImage = zeros(obj.row, obj.col, 3) + 255;

            for i = 1:obj.row
                for j = 1:obj.col
                    state = obj.map(i, j).state;
                    tag = obj.map(i, j).tag;

                    if state == DMapState.START || state == DMapState.GOAL...
                            || state == DMapState.PATH || state == DMapState.OBSTACLE
                        rgbImage(i, j, :) = state.getColor();
                    elseif tag == DStateTag.OPEN || tag == DStateTag.CLOSED
                        rgbImage(i, j, :) = tag.getColor();
                    else
                        rgbImage(i, j, :) = state.getColor();
                    end
                end
            end

            tmp = highlightedState;
            while size(tmp, 1) ~= 0 && tmp.state ~= DMapState.GOAL
                rgbImage(tmp.x, tmp.y, :) = DMapState.PATH.getColor();
                tmp = tmp.parent;
            end
        end


        % plot the map image
        function plot(obj, highlightedState)
            arguments
                obj {}
                highlightedState {} = DState.empty
            end
            J = obj.buildImage(highlightedState);
            %J = imrotate(rgbImage,90);
            %J = imresize( J , 100);
            imshow(J, 'InitialMagnification', 1000);
        end
    end
end