classdef geometry < pointCloud
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        %==================================================================
        % Constructor
        %==================================================================
        % geo = pointCloud(xyzPoints);
        % geo = pointCloud(..., 'Color', C);
        % geo = pointCloud(..., 'Normal', nv);
        function this = geometry(varargin)
            narginchk(1, 5);
            
            [xyzPoints, C, nv] = validateAndParseInputs(varargin{:}); 
            
            this.Location = xyzPoints;
            this.Color = C;
            this.Normal = nv;
            
            this.IsOrganized = ~ismatrix(this.Location); % M-by-N-by-3
        end
    end
    
end

