clear all;

femur = geometry('femur.stl');
%geo.show;

ipl= geometry ('DeformedGeometry.stl');
ipl.show

ipl_target=ipl;

%[bF,bV,bN]=stlread('DeformedGeometry_binary.stl');
% ipl.show


%show the error pvError on the initial geometry
%patch('Faces',ball.faces,'Vertices',ball.vertices,'FaceVertexCData',pVError,'FaceColor','interp','EdgeColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);