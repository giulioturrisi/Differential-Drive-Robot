classdef DStateTag
    enumeration
        NEW
        OPEN
        CLOSED
    end

    methods
        function c = getColor(obj)
            switch obj
                case DStateTag.NEW
                    c = [0, 0, 0];
                case DStateTag.OPEN
                    c = [0, 255, 0];
                case DStateTag.CLOSED
                    c = [0, 0, 255];
            end
        end
    end
end