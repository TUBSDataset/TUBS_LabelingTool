function [] = transformBox(oMovableLabel, fDeltaYaw, vfDeltaXY)
% ---------------------------------------------------------------------------------------------
% Function transformBox(...) transforms a given box graphic. 
%
% INPUT:
%   oMovableLabel:  Object data of box to be transformed
%   oBox_h:         Handle to graphic object. 
%   fDeltaYaw:      Yaw angle by which to rotate current box.
%   vfDeltaXY:      2x1 translation vector (x,y)' in x,y-plane
%
% OUTPUT:
%   -
% ---------------------------------------------------------------------------------------------
oBox_h = oMovableLabel.m_Box_h;
child_handles = allchild(oBox_h);
center(1,1) = oMovableLabel.m_fBBMiddle_x;
center(2,1) = oMovableLabel.m_fBBMiddle_y;

base_h = findobj(oBox_h, 'Tag', 'velocity arrow base');
if(~isa(base_h, 'matlab.graphics.GraphicsPlaceholder'))
    center_velocity(1,1) = base_h.XData(1,1);
    center_velocity(2,1) = base_h.YData(1,1);
else
    center_velocity(1,1) = 0;
    center_velocity(2,1) = 0;
end

center_vel_rotated = 0;
for i = 1 : length(child_handles)
    if(strcmp(get(child_handles(i), 'Type'), 'hggroup'))
        subchild_handles = allchild(child_handles(i));
    else
        subchild_handles = child_handles(i);
    end
    for j = 1 : length(subchild_handles)
        if(strcmp(get(subchild_handles(j), 'Type'), 'text'))
            xData = subchild_handles(j).Position(1,1);
            yData = subchild_handles(j).Position(1,2);
        else
            xData = subchild_handles(j).XData;
            yData = subchild_handles(j).YData;
        end
        
        points = [];
        points(1,:) = xData(1,:);
        points(2,:) = yData(1,:);
        
        phi = fDeltaYaw/180*pi();
        R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
        points = repmat(center, [1, size(points,2)]) + R*(points - repmat(center, [1, size(points,2)])); % rotate via deltaYaw
        points = points + repmat(vfDeltaXY, [1, size(points,2)]); % translate
        
        if(~center_vel_rotated)
            center_velocity = center + R*(center_velocity - center); % rotate only once
            center_vel_rotated = 1;
        end
        if(strcmp(subchild_handles(j).Tag, 'velocity arrow base'))
            % Rotate velocity vector back, rotation center is vec start point
            R = [cos(-phi) -sin(-phi); sin(-phi) cos(-phi)];
            points = repmat(center_velocity, [1, size(points,2)]) + R*(points - repmat(center_velocity, [1, size(points,2)])); % rotate back via deltaYaw
        elseif(strcmp(subchild_handles(j).Tag, 'velocity arrow head'))
            R = [cos(-phi) -sin(-phi); sin(-phi) cos(-phi)];
            points = repmat(center_velocity, [1, size(points,2)]) + R*(points - repmat(center_velocity, [1, size(points,2)])); % rotate back via deltaYaw
        end
        
        xData = points(1,:);
        yData = points(2,:);
        
        if(strcmp(get(subchild_handles(j), 'Type'), 'text'))
            subchild_handles(j).Position(1,1) = xData;
            subchild_handles(j).Position(1,2) = yData;
        else
            set(subchild_handles(j), 'XData', xData);
            set(subchild_handles(j), 'YData', yData);
        end
    end
end

drawnow nocallbacks;

end





