function scanData = F_propagateScanData(scanData,refineOutPts,alpha)
%F_PROPAGATESCANDATA propagates the sensor data to the poses where the part
%could not be detected. This is just to improve the graphical appearance of
%the amplitude map and standoff map and it does not influence the execution
%of the autonomous robotic sensing framework described into the paper
%titled: "Autonomous Robotic Sensing for Simultaneous Geometric and
%Volumetric Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and
%A. Poole.
%   scanData = F_propagateScanData(scanData,refineOutPts,alpha)
%
%   Inputs:
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
%       refineOutPts - a boolean variable indicating if the user wants to
%                      refine the coordinates of the poses where the part
%                      could not be detected, using the computations
%                      performed within this function (refineOutPts==TRUE).
%                      This slows down the update of the output figure.
%                      NOTE - The refinement of these poses does not
%                      influence the selection of the next inspection poses.
%       alpha - A scalar number (double) indicating the orientation of the
%               initial sensor reference system, with respect to the global
%               x-axis direction. This is used in other functions to keep
%               the orientation of the sensor frame consistent throughout
%               the autonomous robotic inspection of a part. alpha is given
%               in degrees.
%
%   Outputs:
%       scanData - Updated scanData
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_propagateScanData v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------


if ~isempty(find(scanData.isDataPt(1:scanData.nPts,1),1))
    
    isDataPropagated = false(scanData.nPts,1);
    logDataPropagation = isDataPropagated;
    
    ind = find((scanData.isDataPt(1:scanData.nPts,1) == false) & (isDataPropagated(1:scanData.nPts,1) == false));
    nToCorrect_old = inf;
    nToCorrect_new = length(ind);
    
    while ~isempty(ind) && (nToCorrect_new < nToCorrect_old)
        nToCorrect_old = nToCorrect_new;
        
        for i=1:length(ind)
            j1 = find((scanData.iTriEdges(1:scanData.nEdges,1) == ind(i)) & (scanData.isDataPt(scanData.iTriEdges(1:scanData.nEdges,2),1) | isDataPropagated(scanData.iTriEdges(1:scanData.nEdges,2),1)));
            j2 = find((scanData.iTriEdges(1:scanData.nEdges,2) == ind(i)) & (scanData.isDataPt(scanData.iTriEdges(1:scanData.nEdges,1),1) | isDataPropagated(scanData.iTriEdges(1:scanData.nEdges,1),1)));
            iPts = [scanData.iTriEdges(j1,2);scanData.iTriEdges(j2,1)];
            
            if length(iPts)>1
                if refineOutPts
                    meanPt = mean(scanData.pts(iPts,1:3),1);
                    rotM = eul2rotm(deg2rad(scanData.pts(iPts,4:6)),'ZYX');
                    rotM = averageRT(rotM);
                    nVect = rotM(:,3)';
                    
                    if ~isnan(alpha)
                        uRef = -[cos(atan2(-nVect(1),nVect(3))) 0 sin(atan2(-nVect(1),nVect(3)))];
                        vRef = cross(nVect,uRef);
                        vRef = vRef./norm(vRef);
                        Rref = [uRef(1,:)' vRef(1,:)' nVect(1,:)'];
                        uRef = uRef*Rref;
                        Rrot = [cos(alpha) -sin(alpha) 0;
                            sin(alpha)  cos(alpha) 0;
                            0           0          1];
                        uRef = uRef*Rrot;
                        uVect = uRef/Rref;
                        vVect = cross(nVect,uVect);
                        vVect = vVect./norm(vVect);
                    else
                        [uVect,vVect] = F_uvRodrigues(nVect);
                    end
                    
                    rotM = [uVect(1,:)' vVect(1,:)' nVect'];
                    scanData.pts(ind(i),4:6) = rad2deg(rotm2eul(rotM,'ZYX'));
                    scanData.pts(ind(i),1:3) = scanData.pts(ind(i),1:3) - (F_vectorDot3D(nVect,(scanData.pts(ind(i),1:3)-meanPt)).*nVect);
                end
                
                scanData.amp(ind(i),1) = mean(scanData.amp(iPts,1));
                scanData.standoff(ind(i),1) = mean(scanData.standoff(iPts,1));
                
                logDataPropagation(ind(i),1) = true;
            end
        end
        isDataPropagated = logDataPropagation;

        ind = find((scanData.isDataPt(1:scanData.nPts,1) == false) & (isDataPropagated(1:scanData.nPts,1) == false));
        nToCorrect_new = length(ind);
    end
end

