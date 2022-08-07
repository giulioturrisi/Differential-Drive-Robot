% check if val is inside list L
function isIn = isAlredyIn(L, val)
    isIn = false;
    for elem=L
        if all(elem==val)
            isIn = true;
            break
        end
    end
end


