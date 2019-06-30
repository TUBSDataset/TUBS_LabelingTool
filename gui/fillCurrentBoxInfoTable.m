function [clData] = fillCurrentBoxInfoTable(oPCMovableLabel)
% ---------------------------------------------------------------------------------------------
% Function fillCurrentBoxInfoTable(...) fills the current box information table.
%
% INPUT:
%   oPCMetadata:    Object of class cPCMovableLabel containing the object's information.
%
% OUTPUT:
%   clData:         Cell array of displayed data
%
% EXAMPLES:
%   fillCurrentBoxInfoTable() 
%   fillCurrentBoxInfoTable(oPCMovableLabel)
% ---------------------------------------------------------------------------------------------

if(nargin == 1)
    l = oPCMovableLabel.m_fBBLength;
    w = oPCMovableLabel.m_fBBWidth;
    h = oPCMovableLabel.m_fBBHeight;
    y = oPCMovableLabel.m_fBBYaw;
    vAbs = sqrt(oPCMovableLabel.m_fVxAbs^2 + oPCMovableLabel.m_fVyAbs^2);
    aAbs = sqrt(oPCMovableLabel.m_fAxAbs^2 + oPCMovableLabel.m_fAyAbs^2);
else
    l = 0;
    w = 0;
    h = 0;
    y = 0;
    vAbs = 0;
    aAbs = 0;
end

clData = {'<html><span style="font-weight: bold;"> l =', sprintf('<html><span style="align=center";> %.1f m', l), ...  
        '<html><span style="font-weight: bold;"> w =', sprintf('<html><span style="align=center";> %.1f m', w), ...
        '<html><span style="font-weight: bold;"> h =', sprintf('<html><span style="align=center";> %.1f m', h), ...
        '<html><span style="font-weight: bold;"> y =', sprintf('<html><span style="align=center";> %.0f %s', y, char(hex2dec('B0'))),...
        '<html><span style="font-weight: bold;"> v =', sprintf('<html><span style="align=center";> %.1f m/s', vAbs), ...
        '<html><span style="font-weight: bold;"> a =', sprintf('<html><span style="align=center";> %.1f m/s', aAbs)};
end





