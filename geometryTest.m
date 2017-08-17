clear all;

femur = geometry('femur.stl');
%geo.show;

ipl= geometry ('DeformedGeometry.stl');
ipl.show

%[bF,bV,bN]=stlread('DeformedGeometry_binary.stl');
% ipl.show