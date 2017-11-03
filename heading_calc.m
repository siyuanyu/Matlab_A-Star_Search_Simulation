function [heading] = heading_calc(dx,dy)

%1st quadrant
if dy>0 && dx>0 
    heading=atan(abs(dy/dx));
%2nd quadrant
elseif dy>0 && dx<0
    heading=pi-atan(abs(dy/dx));
%3rd quadrant
elseif dy<0 && dx<0
    heading=-pi+atan(abs(dy/dx));
%4th quadrant
elseif dy<0 && dx>0
    heading=-atan(abs(dy/dx));
elseif dy==0 && dx>0
    heading=0;
elseif dy==0 && dx<0
    heading=pi;
elseif dx==0 && dy>0
    heading=pi/2;
elseif dx==0 && dy<0
    heading=-pi/2;
end

end

