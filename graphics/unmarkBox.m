function [] = unmarkBox(oPCMovableLabel)
% ---------------------------------------------------------------------------------------------
% Function unmarkBox(...) resets specific graphics within an object's box plot.
%
% INPUT:
%   oPCMovableLabel:     Object of class cPCMovableLabel whose box is to unmark.
% OUTPUT:
%   -
% ---------------------------------------------------------------------------------------------

marker_h = findobj(oPCMovableLabel.m_Box_h, 'Tag', 'marker');
set(marker_h, 'Visible', 'off');
middle_h = findobj(oPCMovableLabel.m_Box_h, 'Tag', 'mid');
set(middle_h, 'Visible', 'off');
ref_h    = findobj(oPCMovableLabel.m_Box_h, 'Tag', 'reference');
set(ref_h, 'Visible', 'off');
data_h  = findobj(oPCMovableLabel.m_Box_h, 'Tag', 'object_data');
set(data_h, 'Visible', 'off');

text_h   = findobj(oPCMovableLabel.m_Box_h, 'Tag', 'text');
bExtensionFound = 0;
if ~isempty(text_h)
    for i = 1 : size(text_h.String, 2)
        if strcmp(text_h.String(1,i), '|')
            bExtensionFound = 1;
            break;
        end
    end
end

if bExtensionFound
    text_h.String = text_h.String(1,1:(i-2));
end

end

