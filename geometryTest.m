clear all;

femur = geometry('femur.stl');
%geo.show;

[aF,aV,aN]=stlread('DeformedGeometry.stl');
%ipl.show

[bF,bV,bN]=stlread('DeformedGeometry_binary.stl');
% ipl.show