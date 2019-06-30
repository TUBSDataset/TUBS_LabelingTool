classdef cImageData < matlab.mixin.Copyable
    % ---------------------------------------------------------------------------------------------
    % Class cImageData stores an image, its metadata information (timestamps etc. as given in 
    % ImageLabels_xyz) and the image's labels such as projected boxes or licence plates. Image
    % labels are objects of class cImageLabel.
    % ---------------------------------------------------------------------------------------------
    
    properties
        m_oImage            = [];
        m_fFormatVersion    = 0.0;
        m_sImageType        = '';   % Front, Right, Rear, Left
        m_nHeight           = 0;
        m_nWidth            = 0;
        m_nTimestamp        = 0;
        m_fDeltaT_ms        = 0;
        m_voImageLabels     = cImageLabel.empty; % vector (struct array) containing image labels (read from XML file)
        
        % Helper variables go here
        m_oImage_h               % handle to image in axes
        m_bImageAvailable   = 0; % true if image is available in sensor recording. Stored in PCMetadata.
        
        % Rays
        m_nRaysCtr = 0;
        m_clRays   = [];    % Cell array containing oRay_h and oPixel_h graphic handles for ray and scecified pixel
        m_mfMap_x  = [];    % Matrix containing distorted image coordinates (x)
        m_mfMap_y  = [];    % Matrix containing distorted image coordinates (y)
    end
    
    methods
        % Delete obsolete objects from vector (called when deleted by image context menu)
        function obj = validateImageLabelVector(obj)
            nNumObjects = size(obj.m_voImageLabels,1);
            
            % Mark invalid objects for deletion from vector
            index = true(1, nNumObjects);
            for i = 1 : nNumObjects
                if ~isvalid(obj.m_voImageLabels(i,1))
                    index(1,i) = false;
                end
            end
            obj.m_voImageLabels = obj.m_voImageLabels(index,:);
        end
    end
    
    methods (Access = protected)
        function oCopy = copyElement(obj)
            oCopy = copyElement@matlab.mixin.Copyable(obj);
            if ~isempty(obj.m_voImageLabels)
                oCopy.m_voImageLabels = copy(obj.m_voImageLabels);
            end
        end
    end
end

