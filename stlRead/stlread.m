function varargout = stlread(file)
% STLREAD imports geometry from an STL file into MATLAB.
%    FV = STLREAD(FILENAME) imports triangular faces from the ASCII or binary
%    STL file idicated by FILENAME, and returns the patch struct FV, with fields
%    'faces' and 'vertices'.
%
%    [F,V] = STLREAD(FILENAME) returns the faces F and vertices V separately.
%
%    [F,V,N] = STLREAD(FILENAME) also returns the face normal vectors.
%
%    The faces and vertices are arranged in the format used by the PATCH plot
%    object.

% Copyright 2011 The MathWorks, Inc.

    if ~exist(file,'file')
        error(['File ''%s'' not found. If the file is not on MATLAB''s path' ...
               ', be sure to specify the full path to the file.'], file);
    end
    
    fid = fopen(file,'r');    
    if ~isempty(ferror(fid))
        error(lasterror); %#ok
    end
    M = fread(fid,inf,'uint8=>uint8');
    fclose(fid);
    
    %[f,v,n] = stlbinary(M);
    if( isbinary(M) ) % This may not be a reliable test
        [F,V,N] = stlbinary(M);
    else
        [F,V,N] = stlascii(file);
    end
    
    varargout = cell(1,nargout);
    switch nargout        
        case 2
            varargout{1} = F;
            varargout{2} = V;
        case 3
            varargout{1} = F;
            varargout{2} = V;
            varargout{3} = N;
        otherwise
            varargout{1} = struct('Faces',F,'Vertices',V);
    end

end


function [F,V,N] = stlbinary(M)

    F = [];
    V = [];
    N = [];
    
    if length(M) < 84
        error('MATLAB:stlread:incorrectFormat', ...
              'Incomplete header information in binary STL file.');
    end
    
    % Bytes 81-84 are an unsigned 32-bit integer specifying the number of faces
    % that follow.
    numFaces = typecast(M(81:84),'uint32');
    %numFaces = double(numFaces);
    if numFaces == 0
        warning('MATLAB:stlread:nodata','No data in STL file.');
        return
    end
    
    T = M(85:end);
    F = NaN(numFaces,3);
    V = NaN(3*numFaces,3);
    N = NaN(numFaces,3);
    
    numRead = 0;
    while numRead < numFaces
        % Each facet is 50 bytes
        %  - Three single precision values specifying the face normal vector
        %  - Three single precision values specifying the first vertex (XYZ)
        %  - Three single precision values specifying the second vertex (XYZ)
        %  - Three single precision values specifying the third vertex (XYZ)
        %  - Two unused bytes
        i1    = 50 * numRead + 1;
        i2    = i1 + 50 - 1;
        facet = T(i1:i2)';
        
        n  = typecast(facet(1:12),'single');
        v1 = typecast(facet(13:24),'single');
        v2 = typecast(facet(25:36),'single');
        v3 = typecast(facet(37:48),'single');
        
        n = double(n);
        v = double([v1; v2; v3]);
        
        % Figure out where to fit these new vertices, and the face, in the
        % larger F and V collections.        
        fInd  = numRead + 1;        
        vInd1 = 3 * (fInd - 1) + 1;
        vInd2 = vInd1 + 3 - 1;
        
        V(vInd1:vInd2,:) = v;
        F(fInd,:)        = vInd1:vInd2;
        N(fInd,:)        = n;
        
        numRead = numRead + 1;
    end
    
end


function [F,V,N] = stlascii(file)
%stlascii Reads STL ASCII files
%  FV = STLASCII(FILENAME) imports triangular faces from the ASCII 
%     STL file idicated by FILENAME, and returns the patch struct FV, with fields
%     'faces' and 'vertices'.
%
%  [F,V] = STLASCII(FILENAME) returns the faces F and vertices V separately.
%
%  [F,V,N] = STLASCII(FILENAME) also returns the face normal vectors.
%
%     The faces and vertices are arranged in the format used by the PATCH plot
%     object.
%
% filename = 'femur.stl';  % Example file.
%
% code is based on CAD2MATDEMO.M version 1.0 by Don Riley downloaded from 
% MatlabCentral
% Copyright (c) 2003, Don Riley
% All rights reserved.

%% Read .STL file
fid=fopen(file, 'r'); %Open the file, assumes STL ASCII format.
if fid == -1 
    error('File could not be opened, check name or path.')
end
%
% Render files take the form:
%   
%solid BLOCK
%  color 1.000 1.000 1.000
%  facet
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%      normal 0.000000e+00 0.000000e+00 -1.000000e+00
%    outer loop
%      vertex 5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 -5.000000e-01 -5.000000e-01
%      vertex -5.000000e-01 5.000000e-01 -5.000000e-01
%    endloop
% endfacet
%
% The first line is object name, then comes multiple facet and vertex lines.
% A color specifier is next, followed by those faces of that color, until
% next color line.
%
CAD_object_name = sscanf(fgetl(fid), '%*s %s');  %CAD object name, if needed.
%                                                %Some STLs have it, some don't.
vnum=0;       %Vertex number counter.
report_num=0; %Report the status as we go.
VColor = 0;
%
fprintf('Reading vertices\n');
while feof(fid) == 0                    % test for end of file, if not then do stuff
    tline = fgetl(fid);                 % reads a line of data from file.
    fword = sscanf(tline, '%s ');       % make the line a character string
    % Check for color
    if strncmpi(fword, 'c',1) == 1;    % Checking if a "C"olor line, as "C" is 1st char.
        VColor = sscanf(tline, '%*s %f %f %f'); % & if a C, get the RGB color data of the face.
    end                                % Keep this color, until the next color is used.
    if strncmpi(fword, 'v',1) == 1;    % Checking if a "V"ertex line, as "V" is 1st char.
        vnum = vnum + 1;                % If a V we count the # of V's
%         report_num = report_num + 1;    % Report a counter, so long files show status
%         if report_num > 4999;
%             fprintf('Reading vertex num: %d.\n',vnum);
%             report_num = 0;
%         end
        v(:,vnum) = sscanf(tline, '%*s %f %f %f'); % & if a V, get the XYZ data of it.
        %        c(:,vnum) = VColor;              % A color for each vertex, which will color the faces.
    end                                 % we "*s" skip the name "color" and get the data.
end
fclose(fid);
fprintf('%d vertices read.\n',vnum);

%% Build face list; The vertices are in order, so just number them.
%
fnum = vnum/3;      %Number of faces, vnum is number of vertices.  STL is triangles.
flist = 1:vnum;     %Face list of vertices, all in order.
F = reshape(flist, 3,fnum); %Make a "3 by fnum" matrix of face list data.
faces_read = F';  %Orients the array for direct use in patch.
vertices_read = v';  % "

%% unifies the duplicated vertices of a triangular meshed object
%
fprintf('Unify Vertices\n');
[F,V,VD] = unifyVertex(faces_read,vertices_read);
fprintf('%d duplicate vertices deleted.\n',size(VD));
fprintf('%d.vertices left.\n',size(V));

%% calculate normals
%
N = VD;
% TBD


%   Return the faces and vertexs.
%
%     F = [];
%     V = [];

end

% TODO: Change the testing criteria! Some binary STL files still begin with
% 'solid'.
function tf = isbinary(A)
% ISBINARY uses the first line of an STL file to identify its format.
    if isempty(A) || length(A) < 5
        error('MATLAB:stlread:incorrectFormat', ...
              'File does not appear to be an ASCII or binary STL file.');
    end    
    if strcmpi('solid',char(A(1:5)'))
        tf = false; % ASCII
    else
        tf = true;  % Binary
    end
end

function [f,v,vd] = unifyVertex(faces,vertices)
%unifyVertex unifies the duplicated vertices of a triangular meshed object
% INPUT:
%   faces - input facet triangulation 
%   vertices - input vertex coordinates
% OUTPUT:
%   f - unified facet triangulation
%   v - unified vertex coordinates
%   vd - indices of duplicated vertices
%
%   Author: Di Zhu 2016-05-02 (version 1.0)
% Copyright (c) 2016, Di Zhu
% All rights reserved.

%% check for duplicates
cri = sum(vertices,2); % sum as criterion for duplication
 
if (length(unique(cri)) == length(unique(vertices,'rows')))
    vd = dupV ( cri );
else
    cri = cri + vertices(:,1);
    vd = dupV ( cri );
end

%% rewrite vertex info
%
v = zeros(length(vd),3);
for i = 1 : length(vd)
    v(i,:) = vertices(vd(i,1),:);
end

%% rewrite facet info
%
nf = size(faces,1);
f = zeros(nf,3);
for i = 1 : nf
    for j = 1 : 3
        [row,~] = find(vd == faces(i,j));
        f(i,j) = row;
    end
end
end % end of function sub_unify


function [ vd ] = dupV ( cri )
nv = size(cri,1);
vd = []; % index of duplicated vertex
n = 0;
for i = 1 : nv
    if cri(i) == cri(1)
        n = n + 1;
        vd(1,n) = i; %duplicated vertices with V1
    else
    end
end
%% removing vertices above from original match list
%
match = ones(nv,1);
for i = 1 : length(vd)
    match(vd(i)) = 0;
end

%% complete detecting all vertices for duplicated elements
%
r = 1; % indicating row index
for i = 2 : nv % start scanning all points from v2
    c = 0; % indicating colomn index
    if match(i) ~= 0;  % has Vi been removed from original list already?
        r = r + 1;    % r++ when new vertex detected
        for m = i : nv % start scanning from Vi
            if cri(i) == cri(m);
                c = c + 1; % c++ when another vertex found equal to Vi
                vd(r,c) = m; % add index for duplicated element to "duplicated vertices"
                match(m) = 0; % remove Vi from original list
            end
        end
    else
    end
end
end % end of sub-function dupV

