function [] = setSavedPanel(oSavedText_h, oSavedPanel_h, sText, sColor)
% ---------------------------------------------------------------------------------------------
% Function setSavedPanel(...) sets the saved status panel in the GUI frame.
%
% INPUT:
%   oSavedText_h:   Handle to 'saved text'
%   oSavedPanel_h:  Handle to 'saved panel'
%   sText           Text to display
%   vnColor:        Color of panel
% OUTPUT:
%   -                  
% ---------------------------------------------------------------------------------------------

switch sColor
    case 'gr'
        vnColor = [0, 204/255, 102/255];
    case 'r'
        vnColor = [255/255, 77/255, 77/255];
    case 'y'
        vnColor = [255/255, 153/255, 102/255];
    otherwise
        vnColor = [1 0 0];   
end
       
oSavedPanel_h.BackgroundColor = vnColor; 
oSavedText_h.BackgroundColor  = vnColor;

oSavedText_h.String = sText;

end

