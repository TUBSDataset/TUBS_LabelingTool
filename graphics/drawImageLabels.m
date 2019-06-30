function [oGUIObjects] = drawImageLabels(oGUIData, oGUIObjects)
% ---------------------------------------------------------------------------------------------
% Function drawImageLabels(...) draws all image labels contained in cGUIObjects to the axes contained
% in oGUIData.

% INPUT:
%   oGUIData:       Object of class cGUIData containing image axes handles.
%   oGUIObjects:    Object of class cGUIObjects containing image labels.
%
% OUTPUT:
%   oGUIObjects:    Updated cGUIObjects object.
% ---------------------------------------------------------------------------------------------

oConfig = oGUIData.m_oEditorConfig;
voPCMovableLabel = oGUIObjects.m_voPCMovableLabel;

for i = 1 : size(oGUIObjects.m_voImageData, 1)
    % Create missing graphic handle from stored object
    voImageLabels = oGUIObjects.m_voImageData(i,1).m_voImageLabels;
    
    for j = 1 : size(voImageLabels,1)
        sShapeType = voImageLabels(j,1).m_sShapeType;
        sClass     = voImageLabels(j,1).m_sClass;
        nCorres    = voImageLabels(j,1).m_nIdxPCObject;
        mfPos      = voImageLabels(j,1).m_mfPosition;
        
        if ~isempty(voImageLabels(j,1).m_oPoly_h)
            delete(voImageLabels(j,1).m_oPoly_h.Children)
        end
        
        if nCorres > 0
            oPCLabel = voPCMovableLabel(nCorres,1);
        end
        
        % Define color
        if oGUIData.m_bRandomColor
            col = [rand(1,1) rand(1,1) rand(1,1)];
        else
            col = [1 0 0];
            for c = 1 : size(oConfig.m_voMovableClasses, 1)
                if strcmp(voImageLabels(j,1).m_sClass, oConfig.m_voMovableClasses(c,1).Name)
                    col = oConfig.m_voMovableClasses(c,1).Color_RGB;
                end
            end
        end
        
        % Skip personal data labels 
        if ~oGUIData.m_bDisplayPD && (strcmp(sClass, 'Licence plate') || strcmp(sClass, 'Face'))
            continue
        end
        
        % Create label
        if nCorres > 0
            oImageLabel = createImageLabel(voImageLabels(j,1), oGUIData.m_voImageAxes_h(i,1), sShapeType, sClass, mfPos, ...
                oGUIObjects.m_voImageData(i,1), col, nCorres, oPCLabel);
        else
            oImageLabel = createImageLabel(voImageLabels(j,1), oGUIData.m_voImageAxes_h(i,1), sShapeType, sClass, mfPos, ...
                oGUIObjects.m_voImageData(i,1), col);
        end
        
        % Copy graphic
        voImageLabels(j,1).m_oPoly_h = oImageLabel.m_oPoly_h;
    end
    oGUIObjects.m_voImageData(i,1).m_voImageLabels = voImageLabels;
end

end

