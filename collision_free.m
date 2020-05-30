function free = collision_free(o, p, c)
%COLLISION_FREE takes in the list of points forming the obstacles and two
%other points (p and c) to verify if the line joining p and c has an
%intersection with the lines forming the obstacles.
%
% INPUTS
% o - Cell with matrices containing the points forming each obstacle
% p - Single point coordinates
% c - Single point coordinates
% 
% OUTPUTS
% free - Boolean: true ->There is no collision | false ->There is collision


for i = 1:length(o)
    obs = o{i};
    
    for j = 1:length(obs) - 1
        lp1 = obs(j,:);
        lp2 = obs(j+1,:);
        
        if ccw(p, lp1, lp2) ~= ccw(c, lp1, lp2) && ccw(p, c, lp1) ~= ccw(p, c, lp2)
            free = false;
            return;
        end
    end
end

free = true;



    function b = ccw(A, B, C)
        v1 = B - A;
        x1 = v1(1);
        y1 = v1(2);
        
        v2 = C - A;
        x2 = v2(1);
        y2 = v2(2);
        
        b = x1*y2 - x2*y1 > 0;
    end



end

