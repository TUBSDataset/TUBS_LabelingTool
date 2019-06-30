function [] = updateLength(oPCMovableLabel, fDeltaL)
% ---------------------------------------------------------------------------------------------
% Function updateLength(...) updates a box plot's length of a given object.
%
% INPUT:
%   oPCMovableLabel:    Object of class cPCMovableLabel whose box is to be updated.
%   fDeltaL:            Value in m by which to increase an object's length.            

% OUTPUT:
%   -               
% ---------------------------------------------------------------------------------------------

fBBYaw = oPCMovableLabel.m_fBBYaw * pi()/180;
oBox_h = oPCMovableLabel.m_Box_h;
lvec(1,1) = fDeltaL/2*cos(fBBYaw);
lvec(2,1) = fDeltaL/2*sin(fBBYaw);

%% front
front_h = findobj(oBox_h, 'Tag', 'front');
front_h = [front_h; findobj(oBox_h, 'Tag', 'up front left')];
front_h = [front_h; findobj(oBox_h, 'Tag', 'up front right')];

xData = [front_h.XData];
xData = xData + repmat(lvec(1,1), [1, length(xData)]);
xData = reshape(xData, 2, [])'; xData = num2cell(xData, 2);
set(front_h, {'XData'}, xData);

yData = [front_h.YData];
yData = yData + repmat(lvec(2,1), [1, length(yData)]);
yData = reshape(yData, 2, [])'; yData = num2cell(yData, 2);
set(front_h, {'YData'}, yData);

%% back
back_h = findobj(oBox_h, 'Tag', 'back');
back_h = [back_h; findobj(oBox_h, 'Tag', 'up back left')];
back_h = [back_h; findobj(oBox_h, 'Tag', 'up back right')];

xData = [back_h.XData];
xData = xData - repmat(lvec(1,1), [1, length(xData)]);
xData = reshape(xData, 2, [])'; xData = num2cell(xData, 2);
set(back_h, {'XData'}, xData);

yData = [back_h.YData];
yData = yData - repmat(lvec(2,1), [1, length(yData)]);
yData = reshape(yData, 2, [])'; yData = num2cell(yData, 2);
set(back_h, {'YData'}, yData);

%% left
left_h = findobj(oBox_h, 'Tag', 'left');

xData = [left_h.XData];
xData = reshape(xData, 2, [])';
% first column is front, second is back
xData(:,1) = xData(:,1) + repmat(lvec(1,1), [size(xData,1),1]);
xData(:,2) = xData(:,2) - repmat(lvec(1,1), [size(xData,1),1]);
xData = num2cell(xData, 2);
set(left_h, {'XData'}, xData);

yData = [left_h.YData];
yData = reshape(yData, 2, [])';
yData(:,1) = yData(:,1) + repmat(lvec(2,1), [size(yData,1),1]);
yData(:,2) = yData(:,2) - repmat(lvec(2,1), [size(yData,1),1]);
yData = num2cell(yData, 2);
set(left_h, {'YData'}, yData);


%% right
right_h = findobj(oBox_h, 'Tag', 'right');

xData = [right_h.XData];
xData = reshape(xData, 2, [])';
% first column is front, second is back
xData(:,1) = xData(:,1) + repmat(lvec(1,1), [size(xData,1),1]);
xData(:,2) = xData(:,2) - repmat(lvec(1,1), [size(xData,1),1]);
xData = num2cell(xData, 2);
set(right_h, {'XData'}, xData);

yData = [right_h.YData];
yData = reshape(yData, 2, [])';
yData(:,1) = yData(:,1) + repmat(lvec(2,1), [size(yData,1),1]);
yData(:,2) = yData(:,2) - repmat(lvec(2,1), [size(yData,1),1]);
yData = num2cell(yData, 2);
set(right_h, {'YData'}, yData);

%% rectangle arrow
arrow_base_h = findobj(oBox_h, 'Tag', 'arrow base');

if(~isa(arrow_base_h, 'matlab.graphics.GraphicsPlaceholder'))
    xData = arrow_base_h.XData;
    % first column is middle point, second is arrow end
    xData(1,2) = xData(1,2) + lvec(1,1);
    set(arrow_base_h, 'XData', xData);
    
    yData = arrow_base_h.YData;
    yData(1,2) = yData(1,2) + lvec(2,1);
    set(arrow_base_h, 'YData', yData);
    
    arrow_head_h = findobj(oBox_h, 'Tag', 'arrow head');
    
    xData = [arrow_head_h.XData];
    % first column is middle point, second is arrow end
    xData = xData + repmat(lvec(1,1), [1,size(xData,2)]);
    set(arrow_head_h, 'XData', xData);
    
    yData = [arrow_head_h.YData];
    yData = yData + repmat(lvec(2,1), [1,size(xData,2)]);
    set(arrow_head_h, 'YData', yData);
end

%% velocity vector
velocity_h = findobj(oBox_h, 'Tag', 'velocity vector');
arrow_base_h = findobj(velocity_h, 'Tag', 'velocity arrow base');

if(~isa(arrow_base_h, 'matlab.graphics.GraphicsPlaceholder'))
    xData = [arrow_base_h.XData];
    xData = xData + repmat(lvec(1,1), [1, length(xData)]);
    set(arrow_base_h, 'XData', xData);
    
    yData = [arrow_base_h.YData];
    yData = yData + repmat(lvec(2,1), [1, length(yData)]);
    set(arrow_base_h, 'YData', yData);
    
    arrow_head_h = findobj(oBox_h, 'Tag', 'velocity arrow head');
    
    xData = [arrow_head_h.XData];
    xData = xData + repmat(lvec(1,1), [1, length(xData)]);
    set(arrow_head_h, 'XData', xData);
    
    yData = [arrow_head_h.YData];
    yData = yData + repmat(lvec(2,1), [1, length(yData)]);
    set(arrow_head_h, 'YData', yData);
end
drawnow;

end

