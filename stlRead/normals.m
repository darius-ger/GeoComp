function N=normals(varargin)
%normals Calculate the vertex normals of a triangulated mesh, area weighted, left-hand-rule 
% N = patchnormals(FV)    calculates normals from a struct containing FV.faces 
%                         with a facelist N-by-3 and FV.vertices with a M-by-3 
%                         vertices list. Such a structure is created by Matlab
%                         Patch function
% N = patchnormals(F,V)   calculates normals from a N-by-3 matrix containing  
%                         a facelist and M-by-3 matrix vertices containing a  
%                         vertices list. 
% Options:
%
%
% 

%%  Validate and parse input
%
if isstruct(varargin{1})
    if ~all(isfield(varargin{1},{'vertices','faces'}))
        error('patchnormals:inputs','Input should be a faces/vertices structure')
    end
    vertices=FV.vertices;
    faces=FV.faces;
elseif nargin==2 
    faces=varargin{1};
    vertices=varargin{2};
elseif nargin>2
    errorerror('patchnormals:inputs','Too many input arguments')
end

% Check size of vertice array
sizev=size(vertices);
if((sizev(2)~=3)||(length(sizev)~=2))
    error('patchnormals:inputs','The vertice list is not a m x 3 array')
end

% Check size of vertice array
sizef=size(faces);
if((sizef(2)~=3)||(length(sizef)~=2))
    error('patchnormals:inputs','The vertice list is not a m x 3 array')
end

% Check if vertice indices exist
if(max(faces(:))>size(vertices,1))
    error('patchnormals:inputs','The face list contains an undefined vertex index')
end

% Check if vertice indices exist
if(min(faces(:))<1)
    error('patchnormals:inputs','The face list contains an vertex index smaller then 1')
end
 
%% calculate face normals
%
% face corners index 
A = faces(:,1); 
B = faces(:,2); 
C = faces(:,3);
% face normals 
n = cross(vertices(A,:)-vertices(B,:),vertices(C,:)-vertices(A,:)); %area weighted

%% create output 
N=n;
if false
    %vertice normals
    N = zeros(size(vertices)); %init vertix normals
    for i = 1:size(faces,1) %step through faces (a vertex can be reference any number of times)
        N(A(i),:) = N(A(i),:)+n(i,:); %sum face normals
        N(B(i),:) = N(B(i),:)+n(i,:);
        N(C(i),:) = N(C(i),:)+n(i,:);
    end
end
end


