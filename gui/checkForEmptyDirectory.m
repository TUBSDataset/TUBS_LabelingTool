function [bIsEmpty, files] = checkForEmptyDirectory(sDir)
% ---------------------------------------------------------------------------------------------
% Function checkForEmptyDirectory checks a directory and returns true if it's empty.
%
% INPUT:
%   sDir:       String specifying the directory to be checked 
% OUTPUT:
%   bIsEmpty:   True if directory is empty.
% --------------------------------------------------------------------------------------------- 

bIsEmpty = 0;
files = dir(sDir);
if isempty(files)
    bIsEmpty = 1;
    return;
end
if strcmp(files(end).name(1,1), '.')
    bIsEmpty = 1;
end
end

