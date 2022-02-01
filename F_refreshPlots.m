function handles = F_refreshPlots(handles,scanData,alpha,plotPath,nP)
%F_REFRESHPLOTS updates the figure that is used to demonstrate the 
%execution of the autonomous robotic sensing framework described into the
%paper titled: "Autonomous Robotic Sensing for Simultaneous Geometric and
%Volumetric Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and
%A. Poole.
%   handles = F_refreshPlots(handles,scanData,alpha,plotPath,nP)
%
%   Inputs:
%       handles - Figure handles
%       scanData - Structure array of the data structure, containing the 
%                  following fields:
%               .nPts - (1 x 1 uint32) Number of distinct visited points.
%               .pts - (nPts x 6 double) A matrix, where each row contains
%                      the Cartesian coordinates of one robot pose (X,Y,Z)
%                      and its Eulerian angular coordinates (A,B,C). The
%                      Eulerian angles are given in degrees. Each distinct
%                      pose is listed only once, to minimize memory consumption.
%               .isDataPt - (nPts x 1 bool) A Boolean value for each point.
%                           TRUE (1) = sensor data has been collected at
%                           the point (the part has been detected). 
%               .isOverlapPt - (nPts x 1 bool) A Boolean value for each
%                              point. TRUE (1) = the point overlaps with a
%                              previously inspected region.
%               .amp - (nPts x 1 double) A value for each point, to store
%                      the amplitude value measured by the sensor.
%               .standoff - (nPts x 1 double) A value for each point, to
%                           store the measured sensor standoff.
%               .nViaPts - (1 x 1 uint32) Number of points  constituting
%                          the robotic trajectory.
%               .iViaPts - (nViaPts x 1 uint32) Indices of the trajectory
%                          points (the indices refer to the points in pts).
%               .isAcquPt - (nViaPts x 1 bool) A Boolean value for each
%                           trajectory point. TRUE (1) = sensor data
%                           acquisition has been attempted at the relative
%                           trajectory point.
%               .nTri - (1 x 1 uint32) Number of triangles in the geometry
%                       reconstruction tessellated surface.
%               .iTri - (nTri x 3 uint32) Each row contains the indices of
%                       the vertices of one triangle. The indices refer to
%                       the points in pts (with no repetitions).
%               .nEdges - (1 x 1 uint32) Number of distinct triangle edges
%                         in the whole triangulated surface (with no
%                         repetitions).
%               .iTriEdges - (nEdges x 2 uint32) Each row contains the
%                            indices of the extremities of one edge. The
%                            indices refer to the points in pts. 
%               .isOutEdge - (nEdges x 1 bool) A Boolean value for each
%                            edge of the triangulation. TRUE (1) = the
%                            relative edge is on the perimeter of the
%                            triangulation.
%       alpha - A scalar number (double) indicating the orientation of the
%               initial sensor reference system, with respect to the global
%               x-axis direction. This is used in other functions to keep
%               the orientation of the sensor frame consistent throughout
%               the autonomous robotic inspection of a part. alpha is given
%               in degrees.
%       plotPath - a boolean variable indicating if the user wants the
%                  inspection trajectory to be highlighted through a solid
%                  black line.
%       nP - (1 x 1 uint32) number of trajectory points to lighlight 
%            through a shading line. If np==Inf, the whole trajectory is
%            highlighted. 
%
%   Outputs:
%       handles - Updated figure handles
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_refreshPlots v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

nP = nP+1;
scanData = F_propagateScanData(scanData,false,alpha);
set(handles.startPoint,'XData',scanData.pts(1,1),'YData',scanData.pts(1,2),'ZData',scanData.pts(1,3));

set(handles.PoI,'XData',scanData.pts(scanData.iViaPts(scanData.nViaPts),1),'YData',scanData.pts(scanData.iViaPts(scanData.nViaPts),2),'ZData',scanData.pts(scanData.iViaPts(scanData.nViaPts),3));

set(handles.tri,'Faces',scanData.iTri(1:scanData.nTri,:));
set(handles.tri,'Vertices',scanData.pts(1:scanData.nPts,1:3));
alphaVertices = ones(scanData.nPts,1).*0.5;
set(handles.tri,'FaceVertexAlphaData',alphaVertices);

if plotPath
    if isinf(nP)
        inspPath = scanData.pts(scanData.iViaPts(1:scanData.nViaPts),:);
        iLinks = find(~scanData.isAcquPt(1:scanData.nViaPts));
        colorMat = uint8([zeros(3,scanData.nViaPts); ones(1,scanData.nViaPts).*255]);
    else
        alphaLine = uint8(linspace(0,255,nP));
        if scanData.nViaPts<nP
            inspPath = scanData.pts(scanData.iViaPts(1:scanData.nViaPts),:);
            alphaLine = alphaLine(end-scanData.nViaPts+1:end);
            iLinks = find(~scanData.isAcquPt(1:scanData.nViaPts));
            colorMat = [uint8(zeros(3,scanData.nViaPts)); alphaLine];
        else
            inspPath = scanData.pts(scanData.iViaPts(scanData.nViaPts-nP+1:scanData.nViaPts),:);
            iLinks = find(~scanData.isAcquPt(scanData.nViaPts-nP+1:scanData.nViaPts));
            colorMat = [uint8(zeros(3,nP)); alphaLine];
        end
    end
    
    colorMat(1,iLinks) = 255;
    colorMat(3,iLinks) = 255;
    set(handles.inspPath,'XData',inspPath(:,1),'YData',inspPath(:,2),'ZData',inspPath(:,3));
    set(handles.inspPath.Edge, 'ColorBinding', 'interpolated', 'ColorData', colorMat,'ColorType', 'truecoloralpha');
else
    set(handles.inspPath,'XData',nan,'YData',nan,'ZData',nan);
end

set(handles.triAmp,'Faces',scanData.iTri(1:scanData.nTri,:));
set(handles.triAmp,'Vertices',scanData.pts(1:scanData.nPts,1:3));
set(handles.triAmp,'CData',scanData.amp(1:scanData.nPts));
alphaVertices = ones(scanData.nPts,1);
alphaVertices(~scanData.isDataPt(1:scanData.nPts)) = 0;
set(handles.triAmp,'FaceVertexAlphaData',alphaVertices);

set(handles.triStandoff,'Faces',scanData.iTri(1:scanData.nTri,:));
set(handles.triStandoff,'Vertices',scanData.pts(1:scanData.nPts,1:3));
set(handles.triStandoff,'FaceVertexAlphaData',alphaVertices);
set(handles.triStandoff,'CData',scanData.standoff(1:scanData.nPts));


xmin = min(scanData.pts(1:scanData.nPts,1));
xmax = max(scanData.pts(1:scanData.nPts,1));
xDelta = xmax - xmin;
if xDelta == 0
    xDelta = 1;
end
ymin = min(scanData.pts(1:scanData.nPts,2));
ymax = max(scanData.pts(1:scanData.nPts,2));
yDelta = ymax - ymin;
if yDelta == 0
    yDelta = 1;
end

if xDelta>=yDelta
    set(handles.plotAmpMap,'xlim',[xmin xmin+xDelta],'ylim',[ymin-((xDelta-yDelta)/2) ymin+xDelta-((xDelta-yDelta)/2)])
    set(handles.plotStandoffMap,'xlim',[xmin xmin+xDelta],'ylim',[ymin-((xDelta-yDelta)/2) ymin+xDelta-((xDelta-yDelta)/2)])
else
    set(handles.plotAmpMap,'xlim',[xmin-((yDelta-xDelta)/2) xmin+yDelta-((yDelta-xDelta)/2)],'ylim',[ymin ymin+yDelta])
    set(handles.plotStandoffMap,'xlim',[xmin-((yDelta-xDelta)/2) xmin+yDelta-((yDelta-xDelta)/2)],'ylim',[ymin ymin+yDelta])
end

colBarLim = get(handles.plotAmpMap,'CLim');
if abs(diff(colBarLim))<0.08
    set(handles.plotAmpMap,'CLim',[mean(colBarLim)-0.04 mean(colBarLim)+0.04]);
end

colBarLim = get(handles.plotStandoffMap,'CLim');
if abs(diff(colBarLim))<0.08
    set(handles.plotStandoffMap,'CLim',[mean(colBarLim)-0.04 mean(colBarLim)+0.04]);
end

drawnow;



