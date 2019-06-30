function [nPosEnd] = setInfoText(varargin)
% ---------------------------------------------------------------------------------------------
% Function setInfoText(...) sets the GUI's info panel.
%
% Example: setInfoText(oInfo_h, 'Info', 1)
% Example: setInfoText(oInfo_h, 'Info', 1,    3,    nCtr)       monitor progress at line 3
% Example: setInfoText(oInfo_h, 'Info', 1, 'end',   nCtr)       monitor progress at last line
%
% INPUT:
%   oInfo_h:    Handle to info panel
%   sText:      Text to write
%   bAppend:    0: Overwrite panel, 1: append to existing text
%   nPosHeader: Optional. Overwrites specific line position. 'end' refers to last line.
%   nCtr:       Optional. Required if progress is required to be indicated
%   
% OUTPUT:
%   nPosEnd:    Number of current written lines in info panel
% --------------------------------------------------------------------------------------------- 

oInfo_h = varargin{1};
sText   = varargin{2};
bAppend = varargin{3}; % 0: Deletes text, 1: appends

bOverwriteHeader    = 0;
currString          = cellstr(get(oInfo_h, 'String'));

if nargin > 3
    bOverwriteHeader = 1;
    nPosHeader  = varargin{4};
    if strcmp(nPosHeader, 'end')
        nPosHeader = size(currString, 1);
    end
end
bProgress = 0;
if nargin > 4
    bProgress   = 1;
    nCtr        = varargin{5};
end

if ~bAppend
    currString = cellstr(sText);
elseif bProgress
    currString = cellstr(get(oInfo_h, 'String'));
    if mod(nCtr, 2) == 0
        currString{nPosHeader,1} = strcat(sText, ' ..');
    elseif mod(nCtr, 3) == 0
        currString{nPosHeader,1} = strcat(sText, ' ...');
    else
        currString{nPosHeader,1} = strcat(sText, ' .');
    end
elseif bOverwriteHeader
    currString{nPosHeader,1} = sText;
else
    currString = cellstr(get(oInfo_h, 'String'));
    currString{end+1,1} = sText;
end
nPosEnd = size(currString,1);
set(oInfo_h, 'String', currString); 
drawnow;

end

