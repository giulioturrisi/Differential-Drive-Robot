% return the min value and pos inside a list
function [minV, minPos] = minVal(u, list)
    minV = inf;
    minPos = State.empty(1, 0);
    for s=list
        curr = u.c(s) + s.g;
        if curr < minV
            minV = curr;
            minPos = s;
        end
    end
end


