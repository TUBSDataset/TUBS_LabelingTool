function [] = updateWidth(oPCMovableLabel, fDeltaW)
% ---------------------------------------------------------------------------------------------
% Function updateWidth(...) updates a box plot's width of a given object.
%
% INPUT:
%   oPCMovableLabel:    Object of class cPCMovableLabel whose box is to be updated.
%   fDeltaL:            Value in m by which to increase an object's length.            

% OUTPUT:
%   -               
% ---------------------------------------------------------------------------------------------

fBBYaw = oPCMovableLabel.m_fBBYaw * pi()/180;
oBox_h = oPCMovableLabel.m_Box_h;

wvec(1,1) = fDeltaW/2*cos(fBBYaw + pi()/2);
wvec(2,1) = fDeltaW/2*sin(fBBYaw + pi()/2);

%% left
left_h = findobj(oBox_h, 'Tag', 'left');
left_h = [left_h; findobj(oBox_h, 'Tag', 'up front left')];
left_h = [left_h; findobj(oBox_h, 'Tag', 'up back left')];

xData = [left_h.XData];
xData = xData + repmat(wvec(1,1), [1, length(xData)]);
xData = reshape(xData, 2, [])'; xData = num2cell(xData, 2);
set(left_h, {'XData'}, xData);

yData = [left_h.YData];
yData = yData + repmat(wvec(2,1), [1, length(yData)]);
yData = reshape(yData, 2, [])'; yData = num2cell(yData, 2);
set(left_h, {'YData'}, yData);

%% right
right_h = findobj(oBox_h, 'Tag', 'right');
right_h = [right_h; findobj(oBox_h, 'Tag', 'up front right')];
right_h = [right_h; findobj(oBox_h, 'Tag', 'up back right')];

xData = [right_h.XData];
xData = xData - repmat(wvec(1,1), [1, length(xData)]);
xData = reshape(xData, 2, [])'; xData = num2cell(xData, 2);
set(right_h, {'XData'}, xData);

yData = [right_h.YData];
yData = yData - repmat(wvec(2,1), [1, length(yData)]);
yData = reshape(yData, 2, [])'; yData = num2cell(yData, 2);
set(right_h, {'YData'}, yData);

%% front
front_h = findobj(oBox_h, 'Tag', 'front');

xData = [front_h.XData];
xData = reshape(xData, 2, [])';

% first column is left, second is right
xData(:,1) = xData(:,1) + repmat(wvec(1,1), [size(xData,1),1]);
xData(:,2) = xData(:,2) - repmat(wvec(1,1), [size(xData,1),1]);
xData = num2cell(xData, 2);
set(front_h, {'XData'}, xData);

yData = [front_h.YData];
yData = reshape(yData, 2, [])';
yData(:,1) = yData(:,1) + repmat(wvec(2,1), [size(yData,1),1]);
yData(:,2) = yData(:,2) - repmat(wvec(2,1), [size(yData,1),1]);
yData = num2cell(yData, 2);
set(front_h, {'YData'}, yData);


%% back
back_h = findobj(oBox_h, 'Tag', 'back');

xData = [back_h.XData];
xData = reshape(xData, 2, [])';
% first column is left, second is right
xData(:,1) = xData(:,1) + repmat(wvec(1,1), [size(xData,1),1]);
xData(:,2) = xData(:,2) - repmat(wvec(1,1), [size(xData,1),1]);
xData = num2cell(xData, 2);
set(back_h, {'XData'}, xData);

yData = [back_h.YData];
yData = reshape(yData, 2, [])';
yData(:,1) = yData(:,1) + repmat(wvec(2,1), [size(yData,1),1]);
yData(:,2) = yData(:,2) - repmat(wvec(2,1), [size(yData,1),1]);
yData = num2cell(yData, 2);
set(back_h, {'YData'}, yData);

drawnow;

end

