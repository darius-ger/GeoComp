% ipl= geometry ('DeformedGeometry.stl');
% ipl_noise = geometry ('DeformedGeometry.stl');
% ipl_noise.noise();
% ipl_noise.translate('z',1);
[pVError, nError] = ipl.distanceVertex2Geometry(ipl_noise);

%show
figure
hold on;
patch('Faces',ipl.Faces,'Vertices',ipl.Vertices,'EdgeColor','none','FaceVertexCData',pVError,'FaceColor','interp','FaceLighting','gouraud','AmbientStrength', 0.15);
patch('Faces',ipl_noise.Faces,'Vertices',ipl_noise.Vertices,'FaceColor','none','FaceLighting','gouraud','AmbientStrength', 0.15);
xlabel('x');
ylabel('y');
zlabel('z');

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
          
% Fix the axes scaling, and set a nice view angle
axis('image');
view([-135 35]);
c = colorbar('east');
c.Label.String = 'distance to target';
hold off;