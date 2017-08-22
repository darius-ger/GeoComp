% ipl= geometry ('DeformedGeometry.stl');
% ipl_noise = geometry ('DeformedGeometry.stl');
% ipl_noise.noise();
% ipl_noise.translate('z',1);
% [pVError, nError] = ipl.distanceVertex2Geometry(ipl_noise);

% show
figure;
hold on;
patch('Faces',ipl.Faces,...
      'Vertices',ipl.Vertices,...
      'EdgeColor','none',...
      'FaceVertexCData',pVError,...
      'FaceColor','interp',...
      'FaceLighting','gouraud',...
      'AmbientStrength', 0.15);
patch('Faces',ipl_noise.Faces,...
      'Vertices',ipl_noise.Vertices,...
      'EdgeAlpha',0.5,...
      'LineWidth',0.1,...
      'FaceColor','none',...
      'FaceLighting','gouraud',...
      'AmbientStrength', 0.15);
xlabel('x');
ylabel('y');
zlabel('z');

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
          
% Fix the axes scaling, 
axis('image');
% rotate axis lables
h = rotate3d;
h.ActionPreCallback = @align_axislabel;
h.ActionPostCallback= @align_axislabel;
% set a nice view angle
view([-135 35]);
% add and format a colorbar
c = colorbar('eastoutside');
c.AxisLocation='out';
c.Label.String = 'distance to target [mm]';
c.TickDirection='both';
hold off;