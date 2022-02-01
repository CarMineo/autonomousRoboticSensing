function handles = F_initFigure(TR,TRside,shapeBoundary)
%F_INITFIGURE initializes the figure to demonstrate the progress and the
%results of the autonomous robotic sensing framework described into the
%paper titled: "Autonomous Robotic Sensing for Simultaneous Geometric and
%Volumetric Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and
%A. Poole.
%   handles = F_initFigure(TR,TRside,shapeBoundary)
%
%   Inputs:
%       TR = Triangulation of part surface. This is only for simulation
%            purpose, to support the generation of simulated sensor signals.
%            The part is unknown in real applications.
%       TRside - Tringulation of side part surface.
%       shapeBoundary - geometry boundary.
%
%   Outputs:
%       handles - Figure handles.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_initFigure v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

handles = [];
handles.f = figure('units','normalized','outerposition',[0 0 1 1]);
handles.plotProgress = subaxis(2,2,1, 'SpacingHoriz',0.00,'SpacingVert',0.1,'PaddingRight',0.0,'PaddingLeft',0.00,'PaddingTop',0.0,'PaddingBottom',0.0, 'MarginRight',0.00,'MarginLeft',0.035,'MarginTop',0.03,'MarginBottom',0.06);
axis equal;hold on;grid on;

handles.shapeSide = trisurf(TRside.ConnectivityList,TRside.Points(:,1),TRside.Points(:,2),TRside.Points(:,3),'facecolor',[0.9 0.9 0.9],'edgecolor','none','facealpha',0.5);
handles.shape = trisurf(TR.ConnectivityList,TR.Points(:,1),TR.Points(:,2),TR.Points(:,3),'facecolor',[0.9 0.9 0.9],'edgecolor','none','facealpha',0.5);
set(handles.shape,'FaceLighting','gouraud');
handles.shapeBoundary = plot3(shapeBoundary(:,1),shapeBoundary(:,2),shapeBoundary(:,3),'k','linewidth',1);
handles.shapeBottomBoundary = plot3(shapeBoundary(:,1),shapeBoundary(:,2),zeros(size(shapeBoundary(:,3),1),1),'k','linewidth',1);
light;view(3);

handles.tri = trisurf([],nan,nan,nan,[],'EdgeColor','b','facecolor',[0 1 0]);%shading interp;
%set(handles.tri,'FaceAlpha',0.5);
set(handles.tri,'EdgeAlpha','interp','FaceAlpha','interp');
set(handles.tri,'AlphaDataMapping','scaled'); alim([0 1]);

handles.startPoint = plot3(nan,nan,nan,'.r','markersize',30);
handles.PoI = plot3(nan,nan,nan,'.b','markersize',30);

handles.inspPath = plot3(nan,nan,nan,'-k','linewidth',3);
handles.linkPath = plot3(nan,nan,nan,'-m','linewidth',3);

legend([handles.shape,handles.tri,handles.startPoint,handles.PoI,handles.inspPath,handles.linkPath],'Part','Inspected region','Start point','Current/end point','Inspection path','Link path','fontsize',12,'location','northwest');

set(handles.plotProgress,'SortMethod','childorder');
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
title('Scan progress representation');
pbaspect([1 1 1]);

handles.plotAmpMap = subaxis(2,2,2, 'SpacingHoriz',0.00,'SpacingVert',0.1,'PaddingRight',0.0,'PaddingLeft',0.00,'PaddingTop',0.0,'PaddingBottom',0.0, 'MarginRight',0.00,'MarginLeft',0.035,'MarginTop',0.03,'MarginBottom',0.06);
handles.triAmp = trisurf([],nan,nan,nan,[],'EdgeColor','none');%shading interp;
set(handles.triAmp,'FaceColor','interp','FaceAlpha','interp');
set(handles.triAmp,'AlphaDataMapping','scaled'); alim([0 1]);
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]'); axis equal; box on;
colorbar('location','East'); colormap(handles.plotAmpMap,'hot');
view(3); pbaspect([1 1 1]);
title('Sensor signal amplitude [Volts]');

handles.plotStandoffMap = subaxis(2,2,4, 'SpacingHoriz',0.00,'SpacingVert',0.1,'PaddingRight',0.0,'PaddingLeft',0.00,'PaddingTop',0.0,'PaddingBottom',0.0, 'MarginRight',0.00,'MarginLeft',0.035,'MarginTop',0.03,'MarginBottom',0.06);
handles.triStandoff = trisurf([],nan,nan,nan,[],'EdgeColor','none');%shading interp;
set(handles.triStandoff,'FaceColor','interp','FaceAlpha','interp');
set(handles.triStandoff,'AlphaDataMapping','scaled'); alim([0 1]);
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]'); axis equal; box on;
colorbar('location','East'); colormap(handles.plotStandoffMap,flipud(turbo)); 
view(3); pbaspect([1 1 1])
title('Sensor standoff [mm]');

pos2 = get(handles.plotAmpMap,'position');
pos1 = [0.05 0.02 pos2(1)-0.1 0.95];
set(handles.plotProgress,'position',pos1);

end

