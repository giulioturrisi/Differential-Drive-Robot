classdef DMapState
    % enumeration map states
    enumeration
        OBSTACLE
        EMPTY
        
        START
        GOAL
        
        PATH
    end
    
    methods
        % return the value of the state obj
        function chr = str(obj)
            switch obj
                case DMapState.OBSTACLE
                    chr = "█";
                case DMapState.EMPTY
                    chr = "░";
                case DMapState.START
                    chr = "ⓢ";
                case DMapState.GOAL
                    chr = "♛";
                case DMapState.PATH
                    chr = "≡";
                    
                otherwise
                    error("Wrong value!");
            end
        end

        % return the color associated to the state obj
        function color = getColor(obj)
            switch obj
                case DMapState.OBSTACLE % "█"
                    color = [0, 0, 0];
                case DMapState.EMPTY % "░"
                    color= [255, 255, 255];
                case DMapState.START % "ⓢ"
                    color = [120, 0, 120];
                case DMapState.GOAL % "♛"
                    color = [255, 0, 0];
                case DMapState.PATH % "≡"
                    color = [255, 0, 0];
                otherwise
                    error("Wrong value!");
            end
        end
    end
end