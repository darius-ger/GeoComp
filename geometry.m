classdef geometry < matlab.mixin.Copyable & vision.internal.EnforceScalarHandle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (GetAccess = public, SetAccess = private)
        % Vertices is an M-by-3 or M-by-N-by-3 matrix. Each entry specifies
        % the x, y, z coordinates of a point.
        Vertices = single([]);
        
        % Faces is a M-by-3 matrix . Each entry specifies a Face
        % created by three corresponding points.
        Faces =single([]);
        
        % Normal is an M-by-3 or M-by-N-by-3 matrix. Each entry
        % specifies the x, y, z component of a normal vector.
        Normal = single([]);
    end
    
    properties (Access = public)
        % Color is an M-by-3 or M-by-N-by-3 uint8 matrix. Each entry
        % specifies the RGB color of a point.
        Color = uint8([]);                
    end
    
    properties%(Access = protected, Transient)
        % Points sorted in Kd tree
        Kdtree = [];
    end
    
    properties%(Access = protected, Hidden)
        IsOrganized;
        KdtreeIndexState
    end
       
    properties(Dependent)
        % Count specifies the number of points in the point cloud.
        Count;
        % XLimits is a 1-by-2 vector that specifies the range of point
        % Verticess along X axis.
        XLimits;
        % YLimits is a 1-by-2 vector that specifies the range of point
        % Verticess along Y axis.
        YLimits;
        % ZLimits is a 1-by-2 vector that specifies the range of point
        % Verticess along Z axis.
        ZLimits;
    end
    
    methods
        %==================================================================
        % Constructor
        %==================================================================
        % geo = geometry(xyzPoints);
        % geo = geometry(..., 'file', FILENAME);
        % geo = geometry(..., 'Normal', nv);
        function this = geometry(varargin)
            narginchk(1, 5);
            
            [FileName, C, nv] = validateAndParseInputs(varargin{:}); 
            
            if ~nv
                [this.Faces,this.Vertices] = stlread(FileName);
            else
                [this.Faces,this.Vertices,this.Normal] = stlread(FileName);
                %[this.Faces,this.Vertices] = stlread(FileName);
            end
            
            this.Color = C;
            
            
            this.IsOrganized = ~ismatrix(this.Vertices); % M-by-N-by-3
        end
            
    end
    
    methods
        %==================================================================
        % modifie geometry
        %==================================================================
        function rotateX(varargin)
        end
        
        function rotateY(varargin)
        end
        
        function rotateZ(varargin)
        end
        
        function translate(this, direction, value)
            %translate Translates the geometry along an axis
            %
            % translateX(direction, value)
            % direction: 'x', 'y' or 'z'
            % value: distance to translate in [mm]
            switch direction
                case 'x'
                    this.Vertices(:,1)= this.Vertices(:,1) + value;
                case 'y'
                    this.Vertices(:,2)= this.Vertices(:,2) + value;
                case 'z'
                    this.Vertices(:,3)= this.Vertices(:,3) + value;
            end
        end
        
        function show(this)
            %show Display the model
            %
            % The model is rendered with a PATCH graphics object. We also
            % add some dynamic lighting, and adjust the material properties
            % to change the specular highlighting.
            
            patch('Faces',this.Faces,'Vertices',this.Vertices,...
                'FaceColor',       [0.8 0.8 1.0], ...
                'EdgeColor',       'none',        ...
                'FaceLighting',    'gouraud',     ...
                'AmbientStrength', 0.15);
            
            % Add a camera light, and tone down the specular highlighting
            camlight('headlight');
            material('dull');
            
            % Fix the axes scaling, and set a nice view angle
            axis('image');
            view([-135 35]);
        end
    
        %==================================================================
        % K nearest neighbor search
        %==================================================================
        function [indices, dists] = findNearestNeighbors(this, point, K, varargin)
            %findNearestNeighbors Find the nearest neighbors of a point.
            %
            %  [indices, dists] = findNearestNeighbors(ptCloud, point, K)
            %  returns the K nearest neighbors of a query point. The point
            %  is an [X, Y, Z] vector. indices is a column vector that 
            %  contains K linear indices to the stored points in the point 
            %  cloud. dists is a column vector that contains K Euclidean 
            %  distances to the point. 
            %
            %  [...] = findNearestNeighbors(... , Name, Value)
            %  specifies additional name-value pairs described below:
            %  
            %  'Sort'          True or false. When set to true,
            %                  the returned indices are sorted in the
            %                  ascending order based on the distance
            %                  from a query point.
            %
            %                  Default: false
            %
            %  'MaxLeafChecks' An integer specifying the number of leaf
            %                  nodes that are searched in Kdtree. If the tree
            %                  is searched partially, the returned result may
            %                  not be exact. A larger number may increase the
            %                  search accuracy but reduce efficiency. When this
            %                  number is set to inf, the whole tree will be
            %                  searched to return exact search result. 
            %
            %                  Default: inf
            %
            %  Example : Find K-nearest neighbors
            %  ---------
            %  % Create a point cloud object with randomly generated points
            %  ptCloud = pointCloud(1000*rand(100,3,'single'));
            %
            %  % Define a query point and number of neighbors
            %  point = [50, 50, 50];
            %  K = 10;
            %
            %  % Get the indices and distances of 10 nearest points
            %  [indices, dists] = findNearestNeighbors(ptCloud, point, K);

            narginchk(3, 7);

            validateattributes(point, {'single', 'double'}, ...
                {'real', 'nonsparse', 'finite', 'size', [1, 3]}, mfilename, 'point');
            
            if isa(this.Vertices, 'single')
                point = single(point);
            else
                point = double(point);
            end

            validateattributes(K, {'single', 'double'}, ...
                {'nonsparse', 'scalar', 'positive', 'integer'}, mfilename, 'K');

            K = min(double(K), numel(this.Vertices)/3);

            [doSort, MaxLeafChecks] = validateAndParseSearchOption(varargin{:});

            % Use bruteforce if there are fewer than 500 points
            if numel(this.Vertices)/3 < 500
                if ~this.IsOrganized
                    allDists = visionSSDMetric(point', this.Vertices');
                else
                    allDists = visionSSDMetric(point', reshape(this.Vertices, [], 3)');
                end
                % This function will ensure returning actual number of neighbors
                % The result is already sorted
                [dists, indices] = vision.internal.partialSort(allDists, K);
                tf = isfinite(dists);
                indices = indices(tf);
                dists = dists(tf);
            else
                this.buildKdtree();

                searchOpts.checks = MaxLeafChecks;
                searchOpts.eps = 0;
                [indices, dists, valid] = this.Kdtree.knnSearch(point, K, searchOpts);
                
                % This step will ensure returning actual number of neighbors
                indices = indices(1:valid);
                dists = dists(1:valid);

                % Sort the result if specified
                if doSort
                    [dists, IND] = sort(dists);
                    indices = indices(IND);
                end
            end

            if nargout > 1
                dists = sqrt(dists);
            end
        end
   
        %==================================================================
        % Radius search
        %==================================================================
        function [indices, dists] = findNeighborsInRadius(this, point, radius, varargin)
            %findNeighborsInRadius Find the neighbors within a radius.
            %
            %  [indices, dists] = findNeighborsInRadius(ptCloud, point, radius) 
            %  returns the neighbors within a radius of a query point.
            %  The query point is an [X, Y, Z] vector. indices is a column
            %  vector that contains the linear indices to the stored points in
            %  the point cloud object ptCloud. dists is a column vector that
            %  contains the Euclidean distances to the query point.
            %
            %  [...] = findNeighborsInRadius(... , Name, Value) specifies
            %  specifies additional name-value pairs described below:
            %  
            %  'Sort'          True or false. When set to true,
            %                  the returned indices are sorted in the
            %                  ascending order based on the distance
            %                  from a query point.
            %
            %                  Default: false
            %
            %  'MaxLeafChecks' An integer specifying the number of leaf
            %                  nodes that are searched in Kdtree. If the tree
            %                  is searched partially, the returned result may
            %                  not be exact. A larger number may increase the
            %                  search accuracy but reduce efficiency. When this
            %                  number is set to inf, the whole tree will be
            %                  searched to return exact search result. 
            %
            %                  Default: inf
            %
            %   Example : Find neighbors within a given radius using Kdtree
            %   -----------------------------------------------------------
            %   % Create a point cloud object with randomly generated points
            %   ptCloud = pointCloud(100*rand(1000, 3, 'single'));
            %
            %   % Define a query point and search radius
            %   point = [50, 50, 50];
            %   radius =  5;
            %
            %   % Get all the points within a radius
            %   [indices, dists] = findNeighborsInRadius(ptCloud, point, radius);
        
            narginchk(3, 7);

            validateattributes(point, {'single', 'double'}, ...
                {'real', 'nonsparse', 'finite', 'size', [1, 3]}, mfilename, 'point');

            if isa(this.Vertices, 'single')
                point = single(point);
            else
                point = double(point);
            end

            validateattributes(radius, {'single', 'double'}, ...
                {'nonsparse', 'scalar', 'nonnegative', 'finite'}, mfilename, 'radius');
            
            radius = double(radius);
            
            [doSort, MaxLeafChecks] = validateAndParseSearchOption(varargin{:});
            
            % Use bruteforce if there are less than 500 points
            if numel(this.Vertices)/3 < 500
                if ~this.IsOrganized
                    allDists = visionSSDMetric(point', this.Vertices');
                else
                    allDists = visionSSDMetric(point', reshape(this.Vertices, [], 3)');
                end
                indices = uint32(find(allDists <= radius^2))';
                dists = allDists(indices)';
                tf = isfinite(dists);
                indices = indices(tf);
                dists = dists(tf);
            else
                this.buildKdtree();

                searchOpts.checks = MaxLeafChecks;
                searchOpts.eps = 0;
                [indices, dists] = this.Kdtree.radiusSearch(point, radius, searchOpts);
            end
            
            % Sort the result if specified
            if doSort
                [dists, IND] = sort(dists);
                indices = indices(IND);
            end
            
            if nargout > 1
                dists = sqrt(dists);
            end
        end       
        
        %==================================================================
        % Box search
        %==================================================================
        function indices = findPointsInROI(this, roi)
            %findPointsInROI Find points within a region of interest.
            %
            %  indices = findPointsInROI(ptCloud, roi) returns the points
            %  within a region of interest, roi. The roi is a cuboid
            %  specified as a 1-by-6 vector in the format of [xmin, xmax,
            %  ymin, ymax, zmin, zmax]. indices is a column vector that
            %  contains the linear indices to the stored points in the
            %  point cloud object, ptCloud.
            %
            %   Example : Find points within a given cuboid
            %   -------------------------------------------
            %   % Create a point cloud object with randomly generated points
            %   ptCloudA = pointCloud(100*rand(1000, 3, 'single'));
            %
            %   % Define a cuboid
            %   roi = [0, 50, 0, inf, 0, inf];
            %
            %   % Get all the points within the cuboid
            %   indices = findPointsInROI(ptCloudA, roi);
            %   ptCloudB = select(ptCloudA, indices);
            %
            %   pcshow(ptCloudA.Vertices, 'r');
            %   hold on;
            %   pcshow(ptCloudB.Vertices, 'g');
            %   hold off;
            
            narginchk(2, 2);

            validateattributes(roi, {'single', 'double'}, ...
                {'real', 'nonsparse', 'numel', 6}, mfilename, 'roi');
            
            if isvector(roi)
                roi = reshape(roi, [2, 3])';
            end
            
            if any(roi(:, 1) > roi(:, 2))
                error(message('vision:pointcloud:invalidROI'));
            end

            roi = double(roi);
            
            % Use bruteforce if there are less than 500 points
            if numel(this.Vertices)/3 < 500
                if ~this.IsOrganized
                    tf = this.Vertices(:,1)>=roi(1)&this.Vertices(:,1)<=roi(4) ...
                        &this.Vertices(:,2)>=roi(2)&this.Vertices(:,2)<=roi(5) ...
                        &this.Vertices(:,3)>=roi(3)&this.Vertices(:,3)<=roi(6);
                    indices = uint32(find(tf));
                else
                    tf = this.Vertices(:,:,1)>=roi(1)&this.Vertices(:,:,1)<=roi(4) ...
                        &this.Vertices(:,:,2)>=roi(2)&this.Vertices(:,:,2)<=roi(5) ...
                        &this.Vertices(:,:,3)>=roi(3)&this.Vertices(:,:,3)<=roi(6);
                    indices = uint32(find(tf));
                end
            else
                this.buildKdtree();

                indices = this.Kdtree.boxSearch(roi);
            end                        
        end
        
        %==================================================================
        % Obtain a subset of this point cloud object
        %==================================================================
        function ptCloudOut = select(this, varargin)
            %select Select points specified by index.
            %
            %  ptCloudOut = select(ptCloud, indices) returns a pointCloud
            %  object that contains the points selected using linear
            %  indices.
            %
            %  ptCloudOut = select(ptCloud, row, column) returns a pointCloud
            %  object that contains the points selected using row and
            %  column subscripts. This syntax applies only to organized
            %  point cloud (M-by-N-by-3).
            %
            %  Example : Downsample a point cloud with fixed step
            %  -------------------------------------------------
            %   ptCloud = pcread('teapot.ply');
            %
            %   % Downsample a point cloud with fixed step size 
            %   stepSize = 10;
            %   indices = 1:stepSize:ptCloud.Count;
            %
            %   ptCloudOut = select(ptCloud, indices);
            %
            %   pcshow(ptCloudOut);

            narginchk(2, 3);
            
            if nargin == 2
                indices = varargin{1};
                validateattributes(indices, {'numeric'}, ...
                    {'real','nonsparse', 'vector', 'integer'}, mfilename, 'indices');
            else
                % Subscript syntax is only for organized point cloud
                if ndims(this.Vertices) ~= 3
                    error(message('vision:pointcloud:organizedPtCloudOnly'));
                end
                
                row = varargin{1};
                column = varargin{2};
                validateattributes(row, {'numeric'}, ...
                    {'real','nonsparse', 'vector', 'integer'}, mfilename, 'row');
                validateattributes(column, {'numeric'}, ...
                    {'real','nonsparse', 'vector', 'integer'}, mfilename, 'column');
                indices = sub2ind([size(this.Vertices,1), size(this.Vertices,2)], row, column);
            end
            
            % Obtain the subset for every property
            [loc, c, nv] = this.subsetImpl(indices);
            
            ptCloudOut = pointCloud(loc, 'Color', c, 'Normal', nv);
        end
        
        %==================================================================
        % Remove invalid points from this point cloud object
        %==================================================================
        function [ptCloudOut, indices] = removeInvalidPoints(this)
            %removeInvalidPoints Remove invalid points.
            %
            %  [ptCloudOut, indices] = removeInvalidPoints(ptCloud) removes
            %  points whose coordinates contain Inf or NaN. The second
            %  output, indices, is a vector of linear indices indicating
            %  Verticess of valid points in the point cloud.
            %
            %  Note :
            %  ------
            %  An organized point cloud (M-by-N-by-3) will become
            %  unorganized (X-by-3) after calling this function.
            %               
            %  Example : Remove NaN valued points from a point cloud
            %  ---------------------------------------------------------
            %  ptCloud = pointCloud(nan(100,3))
            %
            %  ptCloud = removeInvalidPoints(ptCloud)

            % Find all valid points
            tf = isfinite(this.Vertices);
            if ~this.IsOrganized
                indices = (sum(tf, 2) == 3);
            else
                indices = (sum(reshape(tf, [], 3), 2) == 3);
            end

            [loc, c, nv] = this.subsetImpl(indices);
            ptCloudOut = pointCloud(loc, 'Color', c, 'Normal', nv);
            if nargout > 1,
                indices = find(indices);
            end
        end
    end

    methods
        %==================================================================
        % Writable Property
        %==================================================================
        function set.Color(this, value) 
            validateattributes(value,{'uint8'}, {'real','nonsparse'});
            if ~isempty(value) && ~isequal(size(value), size(this.Vertices)) %#ok<MCSUP>
                error(message('vision:pointcloud:unmatchedXYZColor'));
            end
            this.Color = value;
        end
        
        function set.Normal(this, value)
            validateattributes(value,{'single', 'double'}, {'real','nonsparse'});
            if ~isempty(value) && ~isequal(size(value), size(this.Faces)) %#ok<MCSUP>
                error(message('geometry:unmatchedXYZNormal'));
            end
            if isa(this.Faces,'double') %#ok<MCSUP>
                value = double(value);
            else
                value = single(value);
            end
            this.Normal = value;
        end
        
        %==================================================================
        % Dependent Property
        %==================================================================
        function xlim = get.XLimits(this)
            tf = ~isnan(this.Vertices);
            if ~this.IsOrganized
                tf = (sum(tf, 2)==3);
                xlim = [min(this.Vertices(tf, 1)), max(this.Vertices(tf, 1))];
            else
                tf = (sum(tf, 3)==3);
                X = this.Vertices(:, :, 1);
                xlim = [min(X(tf)), max(X(tf))];
            end                
        end  
        %==================================================================
        function ylim = get.YLimits(this)
            tf = ~isnan(this.Vertices);
            if ~this.IsOrganized
                tf = (sum(tf, 2)==3);
                ylim = [min(this.Vertices(tf, 2)), max(this.Vertices(tf, 2))];
            else
                tf = (sum(tf, 3)==3);
                Y = this.Vertices(:, :, 2);
                ylim = [min(Y(tf)), max(Y(tf))];
            end                
        end  
        %==================================================================
        function zlim = get.ZLimits(this)
            tf = ~isnan(this.Vertices);
            if ~this.IsOrganized
                tf = (sum(tf, 2)==3);
                zlim = [min(this.Vertices(tf, 3)), max(this.Vertices(tf, 3))];
            else
                tf = (sum(tf, 3)==3);
                Z = this.Vertices(:, :, 3);
                zlim = [min(Z(tf)), max(Z(tf))];
            end                
        end
        %==================================================================
        function count = get.Count(this)
            if ~this.IsOrganized
                count = size(this.Vertices, 1);
            else
                count = size(this.Vertices, 1)*size(this.Vertices, 2);
            end                
        end  
    end
    
    methods (Access = public, Hidden)
        %==================================================================
        % helper function to get subset for each property
        %==================================================================
        function [loc, c, nv] = subsetImpl(this, indices)
            if ~isempty(this.Vertices)
                if ~this.IsOrganized
                    loc = this.Vertices(indices, :);
                else
                    loc = reshape(this.Vertices, [], 3);
                    loc = loc(indices, :);
                end
            else
                loc = zeros(0, 3, 'single');
            end
            
            if nargout > 1
                if ~isempty(this.Color)
                    if ~this.IsOrganized
                        c = this.Color(indices, :);
                    else
                        c = reshape(this.Color, [], 3);
                        c = c(indices, :);
                    end
                else
                    c = uint8.empty;
                end
            end
            
            if nargout > 2
                if ~isempty(this.Normal)
                    if ~this.IsOrganized
                        nv = this.Normal(indices, :);
                    else
                        nv = reshape(this.Normal, [], 3);
                        nv = nv(indices, :);
                    end
                else
                     if isa(loc, 'single')
                        nv = single.empty;
                     else
                        nv = double.empty;
                     end
                end
            end
        end
        
        %==================================================================
        % helper function to support multiple queries in KNN search
        % indices, dists: K-by-numQueries
        % valid: 1-by-K
        % Note, the algorithm may return less than K results for each
        % query. Therefore, only 1:valid(n) in n-th column of indices and
        % dists are valid results. Invalid indices are all zeros.
        %==================================================================
        function [indices, dists, valid] = multiQueryKNNSearchImpl(this, points, K)
            % Validate the inputs
            validateattributes(points, {'single', 'double'}, ...
                {'real', 'nonsparse', 'size', [NaN, 3]}, mfilename, 'points');

            if isa(this.Vertices, 'single')
                points = single(points);
            else
                points = double(points);
            end

            validateattributes(K, {'single', 'double'}, ...
                {'nonsparse', 'scalar', 'positive', 'integer'}, mfilename, 'K');

            K = min(double(K), this.Count);
            
            this.buildKdtree();

            % Use exact search in Kdtree
            searchOpts.checks = 0;
            searchOpts.eps = 0;
            [indices, dists, valid] = this.Kdtree.knnSearch(points, K, searchOpts);
        end
        
        %==================================================================
        % helper function to compute normals
        % normals: the same size of the Vertices matrix
        %
        % Note, the algorithm uses PCA to fit local planes around a point,
        % and chooses the normal direction (inward/outward) arbitrarily.
        %==================================================================
        function normals = surfaceNormalImpl(this, K)            
            % Reset K if there are not enough points
            K = min(double(K), this.Count);
            
            if this.Count <= 2
                normals = NaN(size(this.Vertices), 'like', this.Vertices);
                return;
            end
            
            this.buildKdtree();

            if ~this.IsOrganized
                loc = this.Vertices;
            else
                loc = reshape(this.Vertices, [], 3);
            end
            
            % Use exact search in Kdtree
            searchOpts.checks = 0;
            searchOpts.eps = 0;

            % Find K nearest neighbors for each point
            [indices, ~, valid] = this.Kdtree.knnSearch(loc, K, searchOpts);

            % Find normal vectors for each point
            normals = visionPCANormal(loc, indices, valid);
            
            if this.IsOrganized
                normals = reshape(normals, size(this.Vertices));
            end
        end
    end

    methods (Access = protected)        
        %==================================================================
        % helper function to index data
        %==================================================================
        function buildKdtree(this)
            if isempty(this.Kdtree)                             
                % Build a Kdtree to index the data
                this.Kdtree = vision.internal.Kdtree();                
                createIndex = true;                                        
            elseif this.Kdtree.needsReindex(this.Vertices)                 
                createIndex = true;                          
            else
                createIndex = false;
            end
            
            if createIndex
                % store rand state prior to indexing
                this.KdtreeIndexState = rng;
                this.Kdtree.index(this.Vertices);
            end
        end
    end    
    
    methods(Static, Access=private)
        %==================================================================
        % load object 
        %==================================================================
        function this = loadobj(s)
            this = pointCloud(s.Vertices,...
                'Color', s.Color,...
                'Normal', s.Normal);
            
            if isfield(s, {'KdtreeIndexState'}) % added in R2015b
                % set the saved state prior to indexing to recreate the
                % same KD-Tree.
                                
                this.KdtreeIndexState = s.KdtreeIndexState;
                
                % only create index if it was created previously
                if ~isempty(this.KdtreeIndexState)                    
                    sprev = rng(this.KdtreeIndexState);
                    
                    buildKdtree(this);
                    
                    rng(sprev); % restore previous state
                end                         
            end       
        end  
    end
    
    methods(Access=private)
        %==================================================================
        % save object 
        %==================================================================
        function s = saveobj(this)
            % save properties into struct
            s.Vertices      = this.Vertices;
            s.Color         = this.Color;
            s.Normal        = this.Normal;
            
            % save the index state to enable recreate duplicate on load.
            s.KdtreeIndexState = this.KdtreeIndexState;
        end
    end
    
    methods(Access=protected)
        %==================================================================
        % copy object 
        %==================================================================
        % Override copyElement method:
        function cpObj = copyElement(obj)
            % Make a copy except the internal Kdtree
            cpObj = pointCloud(obj.Vertices, 'Color', obj.Color, 'Normal', obj.Normal);
        end
    end    
end

%==================================================================
% parameter validation
%==================================================================
function [FileName, C, nv] = validateAndParseInputs(varargin)
    % Validate and parse inputs
    narginchk(1, 5);

    parser = inputParser;
    parser.CaseSensitive = false;
    % Parse the arguments according to the format of the first argument

%     if ismatrix(varargin{1})
%         dims = [NaN, 3];
%     else
%         dims = [NaN, NaN, 3];
%     end

    parser.addRequired('FileName', @(x)validateattributes(x,{'char'}, {'nonempty'}));
    parser.addParameter('Color', uint8([]), @(x)validateattributes(x,{'uint8', 'single', 'double'}, {'real','nonsparse'}));
    parser.addParameter('Normal', single([]),  @(x)validateattributes(x,{'uint8', 'single', 'double'}, {'real','nonsparse'}));

    parser.parse(varargin{:});

    FileName = parser.Results.FileName;
    C = parser.Results.Color;
    if ~isa(C, 'uint8')
        C = im2uint8(C);
    end
    nv = parser.Results.Normal;           
%     if isa(FileName, 'single')
%         nv = single(nv);
%     else
%         nv = double(nv);
%     end
end                                
%==================================================================
% parameter validation for search
%==================================================================
function [doSort, maxLeafChecks] = validateAndParseSearchOption(varargin)
persistent p;
if isempty(p)
    % Validate and parse search options
    p = inputParser;
    p.CaseSensitive = false;
    p.addParameter('Sort', false, @(x)validateattributes(x, {'logical'}, {'scalar'}));
    p.addParameter('MaxLeafChecks', inf, @validateMaxLeafChecks);
    parser = p;
else
    parser = p;
end

parser.parse(varargin{:});

doSort = parser.Results.Sort;
maxLeafChecks = parser.Results.MaxLeafChecks;
if isinf(maxLeafChecks)
    % 0 indicates infinite search in internal function
    maxLeafChecks = 0;
end

end
%==================================================================
function validateMaxLeafChecks(value)
    % Validate MaxLeafChecks
    if any(isinf(value))
        validateattributes(value,{'double'}, {'real','nonsparse','scalar','positive'});
    else
        validateattributes(value,{'double'}, {'real','nonsparse','scalar','integer','positive'});
    end
end



        
    
    

