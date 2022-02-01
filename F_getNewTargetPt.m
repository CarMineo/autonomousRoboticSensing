function [P,R,scanData,ang] = F_getNewTargetPt(scanData,iPts,r,rDir,alpha)
%F_GETNEWTARGETPT computes the next best target sensor pose to visit in the
%execution of the autonomous robotic sensing framework described into the
%paper titled: "Autonomous Robotic Sensing for Simultaneous Geometric and
%Volumetric Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and
%A. Poole.
%   [P,R,scanData,ang] = F_getNewTargetPt(scanData,iPts,r,rDir,alpha)
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
%       iPts - (n x 1 uint32) Indices of poses to be taken into account for
%              the computation of the next best pose. A minimum of two
%              indices is required. The first index bust always be relative
%              to the current pose. The second index corresponds to pose A
%              (see explanation in paper). The third index (if present)
%              corresponds to pose B (see explanation in paper).
%       r - A scalar number indicating the target inspection resolution.
%       rDir - Boolean value indicating the preferred rotation direction
%              (TRUE = clockwise, FALSE = anticlockwise).
%       alpha - A scalar number (double) indicating the orientation of the
%               initial sensor reference system, with respect to the global
%               x-axis direction. This is used in other functions to keep
%               the orientation of the sensor frame consistent throughout
%               the autonomous robotic inspection of a part. alpha is given
%               in degrees.
%
%   Outputs:
%       P - (1 x 6 double) Third pose.
%       R - (3 x 3 double) Rotation matrix (ZYX convention) relative to the
%           Third pose.
%       scanData - updated data structure.
%       ang - (1 x 1 double) a scalar value representing the angle used to
%             select the correct pose to use as next best pose (see
%             explanation in paper).
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_getNewTargetPt v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

poses = scanData.pts(iPts,:);
isDataPresent = scanData.isDataPt(iPts);

n = length(iPts)-1;

currPt = poses(1,:);
oldPts = zeros(n,6);
oldPts(1,:) = poses(2,:);
if n>1
    oldPts(2,:) = poses(3,:);
end

normals = zeros(n,3);

zmin = -inf;
zmax = inf;

currRotM = eul2rotm(deg2rad(currPt(:,4:6)),'ZYX');
oldRotM = eul2rotm(deg2rad(oldPts(:,4:6)),'ZYX');

thetaInts = [];
parentHood = [];
thetaIntervals = cell(1,n);
outOfCyl = cell(1,n);

for j=1:n
    Cs = [oldPts(j,1:3);
          currPt(1,1:3)];
    Rs = [r;
        r];
    ns = [oldRotM(:,3,j)';
          currRotM(:,3)'];
      
    normals(j,:) = oldRotM(:,3,j)';
    
    %[us,vs] = F_uvRodrigues(ns);
    
    us = [oldRotM(:,1,j)';
        currRotM(:,1)'];
    
    vs = [oldRotM(:,2,j)';
        currRotM(:,2)'];
    
    % Consider first circle as section of cylinder with unitary radius. The
    % second circle parameters are:
    C = (Cs(2,:)-Cs(1,:))./Rs(1);
%     zmin = z0/Rs(1);
%     zmax =  z1/Rs(1);
    R = Rs(2)/Rs(1);
    u = us(2,:).*R;
    v = vs(2,:).*R;
    
    nVect = [us(1,:)' vs(1,:)' ns(1,:)'];
    
    C = C*nVect;
    u = u*nVect;
    v = v*nVect;

%     C = C/rotM;
%     u = u/rotM;
%     v = v/rotM;
    
    [thetaPts,thetaIntervals{j},outOfCyl{j}] = F_itersectionCirCyl(C,u,v,zmin,zmax);
    thetaInts = [thetaInts; thetaIntervals{j}(:,1); thetaIntervals{j}(end,2)];
    parentHood = [parentHood; ones(size([thetaIntervals{j}(:,1); thetaIntervals{j}(end,2)],1),1).*j];
%     figure(1+j)
%     tCyl = linspace(0,2*pi,100);
%     xCyl = cos(tCyl);
%     yCyl = sin(tCyl);
%     plot(xCyl,yCyl,'-k','linewidth',2); hold on; grid on; axis equal;
%     
%     for i=1:length(outOfCyl{j})
%         tCir = linspace(thetaIntervals{j}(i,1),thetaIntervals{j}(i,2),100);
%         xCir = C(1,1) + u(1,1)*cos(tCir) + v(1,1)*sin(tCir);
%         yCir = C(1,2) + u(1,2)*cos(tCir) + v(1,2)*sin(tCir);
%         if outOfCyl{j}(i)
%             plot(xCir,yCir,'-g','linewidth',2);
%         else
%             plot(xCir,yCir,'-r','linewidth',2);
%         end
%         xPts = C(1,1) + u(1,1)*cos(thetaPts) + v(1,1)*sin(thetaPts);
%         yPts = C(1,2) + u(1,2)*cos(thetaPts) + v(1,2)*sin(thetaPts);
%         plot(xPts,yPts,'.m','markersize',40);
%     end
end

if n>1
    thetaPts = [];
%     thetaInts(thetaInts<0) = thetaInts(thetaInts<0) + (2*pi);
%     thetaInts(thetaInts>(2*pi)) = thetaInts(thetaInts>(2*pi)) - (2*pi);
    thetaInts = mod(thetaInts,(2*pi));
    if isempty(find(thetaInts==0, 1))
        thetaInts = [thetaInts; 0];
        parentHood = [parentHood; nan];
    end
    if isempty(find(thetaInts==(2*pi), 1))
        thetaInts = [thetaInts; (2*pi)];
        parentHood = [parentHood; nan];
    end
    [thetaInts, iu] = uniquetol(thetaInts,eps('single'));
    parentHood = parentHood(iu);
    thetaInts = [thetaInts(1:end-1) thetaInts(2:end)];
    parentHood = [parentHood(1:end-1) parentHood(2:end)];
    thetaMeans = sum(thetaInts,2)./2;
    nIntervals = length(thetaMeans);
    outFlag = true(nIntervals,1);
    for i=1:nIntervals
        j = 0;
        while outFlag(i) && (j<n)
            j = j + 1;
            k = find((thetaMeans(i)>thetaIntervals{j}(:,1) & (thetaMeans(i)<thetaIntervals{j}(:,2))) |...
                     (thetaMeans(i)-(2*pi)>thetaIntervals{j}(:,1) & (thetaMeans(i)-(2*pi)<thetaIntervals{j}(:,2))) |...
                     (thetaMeans(i)+(2*pi)>thetaIntervals{j}(:,1) & (thetaMeans(i)+(2*pi)<thetaIntervals{j}(:,2))),1);
            outFlag(i) = outOfCyl{j}(k);
        end
    end
    
    if length(outFlag)>1
        intIsOut = outFlag(1);
        counter = 1;
        iStart = 1; iEnd = 1;
        thetaIntervals = [];
        parentIntervals = [];
        outOfCyl = false(0);
        while counter<length(outFlag)
            counter = counter + 1;
            if (outFlag(counter)==intIsOut)
                iEnd = counter;
                if (counter==length(outFlag))
                    thetaIntervals = [thetaIntervals; thetaInts(iStart,1) thetaInts(iEnd,2)];
                    parentIntervals = [parentIntervals; parentHood(iStart,1) parentHood(iEnd,2)];
                    outOfCyl = [outOfCyl; intIsOut];
                end
            else
                thetaIntervals = [thetaIntervals; thetaInts(iStart,1) thetaInts(iEnd,2)];
                parentIntervals = [parentIntervals; parentHood(iStart,1) parentHood(iEnd,2)];
                outOfCyl = [outOfCyl; intIsOut];
                iStart = counter;
                iEnd = counter;
                intIsOut = outFlag(counter);
                if (counter==length(outFlag))
                    thetaIntervals = [thetaIntervals; thetaInts(iStart,1) thetaInts(iEnd,2)];
                    parentIntervals = [parentIntervals; parentHood(iStart,1) parentHood(iEnd,2)];
                    outOfCyl = [outOfCyl; intIsOut];
                end
            end
        end
        
        if (length(outOfCyl)>1) && (outOfCyl(1)==outOfCyl(end)) && (thetaIntervals(1,1)<=0) && (abs(thetaIntervals(end,2)-(2*pi))<eps('single'))
            outOfCyl(end) = [];
            thetaIntervals = [(thetaIntervals(end,1)-thetaIntervals(end,2)) thetaIntervals(1,2);
                               thetaIntervals(2:end-1,:)];
            parentIntervals = [(parentIntervals(end,1)-parentIntervals(end,2)) parentIntervals(1,2);
                               parentIntervals(2:end-1,:)];
        end
    else
        thetaIntervals = thetaInts;
        parentIntervals = parentHood;
        outOfCyl = outFlag;
    end
    
    thetaPts = thetaIntervals(:);
    parentHood = parentIntervals(:);
    if isnan(parentHood(1))
        parentHood(1) = parentHood(end);
    elseif isnan(parentHood(end))
        parentHood(end) = parentHood(1);
    end
    thetaPts(thetaPts<0) = thetaPts(thetaPts<0) + (2*pi);
    thetaPts(abs(thetaPts-(2*pi))<eps('single')) = 0;
    [thetaPts, iu] = uniquetol(thetaPts,eps('single'));
    parentHood = parentHood(iu);
    if length(thetaPts)==1
        thetaPts = [];
        parentHood = [];
    end
else
    parentHood = ones(size(thetaPts,1),1);
end

% newPts = [currPt(1) + currRotM(1,1).*r.*cos(thetaPts) + currRotM(1,2).*r.*sin(thetaPts)...
%     currPt(2) + currRotM(2,1).*r.*cos(thetaPts) + currRotM(2,2).*r.*sin(thetaPts)...
%     currPt(3) + currRotM(3,1).*r.*cos(thetaPts) + currRotM(3,2).*r.*sin(thetaPts)...
%     repmat(currPt(4:6),size(thetaPts,1),1)];

newPts = [currPt(1) + currRotM(1,1).*r.*cos(thetaPts) + currRotM(1,2).*r.*sin(thetaPts)...
          currPt(2) + currRotM(2,1).*r.*cos(thetaPts) + currRotM(2,2).*r.*sin(thetaPts)...
          currPt(3) + currRotM(3,1).*r.*cos(thetaPts) + currRotM(3,2).*r.*sin(thetaPts)...
          zeros(size(thetaPts,1),3)];
      
% Decide plane on which points are projected


if n>1
    normal = oldRotM(:,3,1)';
    P0 = poses(2,1:3);
    P1 = poses(1,1:3);
    P2 = poses(3,1:3);
    V1 = P1 - P0;
    V2 = P2 - P0;
    V1 = V1 - (F_vectorDot3D(normal,V1).*normal);
    V2 = V2 - (F_vectorDot3D(normal,V2).*normal);
    uRef = V1./norm(V1);
    v = cross(uRef,normal);
    R = [uRef' v' normal'];
    
    testPoints = V2*R;
    ang = atan2(testPoints(:,2),testPoints(:,1));
    
    if rDir
        ang = -ang;
    end
end

newPts0 = newPts;
nPts = size(newPts,1);
angs = inf(nPts,1);

for i = 1:nPts
%     h = [];
    nPt = 0;
    meanRotM = zeros(3,3,0);
    meanPt = zeros(1,3);
    if isDataPresent(1)
%         h(end+1) = plot3(poses(1,1),poses(1,2),poses(1,3),'.b','markersize',60);
%         h(end+1) = quiver3(poses(1,1),poses(1,2),poses(1,3),currRotM(1,1),currRotM(2,1),currRotM(3,1),r/3,'r','linewidth',2);
%         h(end+1) = quiver3(poses(1,1),poses(1,2),poses(1,3),currRotM(1,2),currRotM(2,2),currRotM(3,2),r/3,'g','linewidth',2);
%         h(end+1) = quiver3(poses(1,1),poses(1,2),poses(1,3),currRotM(1,3),currRotM(2,3),currRotM(3,3),r,'b','linewidth',2);
        
        meanPt = meanPt + poses(1,1:3);
        meanRotM(:,:,nPt + 1) = currRotM;
        nPt = nPt + 1;
    end
    if isDataPresent(2) %&& (parentHood(i)==1) 
%         h(end+1) = plot3(poses(2,1),poses(2,2),poses(2,3),'.g','markersize',60);
%         h(end+1) = quiver3(poses(2,1),poses(2,2),poses(2,3),oldRotM(1,1,1),oldRotM(2,1,1),oldRotM(3,1,1),r/3,'r','linewidth',2);
%         h(end+1) = quiver3(poses(2,1),poses(2,2),poses(2,3),oldRotM(1,2,1),oldRotM(2,2,1),oldRotM(3,2,1),r/3,'g','linewidth',2);
%         h(end+1) = quiver3(poses(2,1),poses(2,2),poses(2,3),oldRotM(1,3,1),oldRotM(2,3,1),oldRotM(3,3,1),r,'b','linewidth',2);
        
        meanPt = meanPt + poses(2,1:3);
        meanRotM(:,:,nPt + 1) = oldRotM(:,:,1);
        nPt = nPt + 1;
    end
    if (n>1) && isDataPresent(3) && (ang>0 && ang<=((2/3)*pi + eps('single')))
%         h(end+1) = plot3(poses(3,1),poses(3,2),poses(3,3),'.r','markersize',60);
%         h(end+1) = quiver3(poses(3,1),poses(3,2),poses(3,3),oldRotM(1,1,2),oldRotM(2,1,2),oldRotM(3,1,2),r/3,'r','linewidth',2);
%         h(end+1) = quiver3(poses(3,1),poses(3,2),poses(3,3),oldRotM(1,2,2),oldRotM(2,2,2),oldRotM(3,2,2),r/3,'g','linewidth',2);
%         h(end+1) = quiver3(poses(3,1),poses(3,2),poses(3,3),oldRotM(1,3,2),oldRotM(2,3,2),oldRotM(3,3,2),r,'b','linewidth',2);
        
        meanPt = meanPt + poses(3,1:3);
        meanRotM(:,:,nPt + 1) = oldRotM(:,:,2);
        nPt = nPt + 1;
    end
    
    meanPt = meanPt./nPt;
    meanRotM = averageRT(meanRotM);
    nVect = (meanRotM(:,3)');
    
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
    
%     h(end+1) = plot3(meanPt(1,1),meanPt(1,2),meanPt(1,3),'*m','markersize',10,'linewidth',2);
%     h(end+1) = quiver3(meanPt(1,1),meanPt(1,2),meanPt(1,3),rotM(1,1),rotM(2,1),rotM(3,1),r/3,'r','linewidth',3);
%     h(end+1) = quiver3(meanPt(1,1),meanPt(1,2),meanPt(1,3),rotM(1,2),rotM(2,2),rotM(3,2),r/3,'g','linewidth',3);
%     h(end+1) = quiver3(meanPt(1,1),meanPt(1,2),meanPt(1,3),rotM(1,3),rotM(2,3),rotM(3,3),r,'b','linewidth',3);
        
    
    newPts(i,1:3) = newPts(i,1:3) - (F_vectorDot3D(nVect,(newPts(i,1:3)-meanPt)).*nVect);
    newPts(i,4:6) = rad2deg(rotm2eul(rotM,'ZYX'));
    
    newPts0(i,4:6) = newPts(i,4:6);
    
    cPt = currPt(1,1:3) - (F_vectorDot3D(nVect,(currPt(1,1:3)-meanPt)).*nVect);
    oldPt = oldPts(1,1:3) - (F_vectorDot3D(nVect,(oldPts(1,1:3)-meanPt)).*nVect);
    uRef = oldPt - cPt;
    uRef = uRef./norm(uRef);
    v = cross(uRef,nVect);
    R = [uRef' v' nVect'];
    
    testPt = newPts(i,1:3) - cPt;
    testPt = testPt*R;
    angs(i) = atan2(testPt(:,2),testPt(:,1));
    
%     delete(h);
end

% newPts(:,1:3) = newPts(:,1:3) - (F_vectorDot3D(normals(parentHood,1:3),(newPts(:,1:3)-oldPts(parentHood,1:3))).*normals(parentHood,1:3));
% newPts(:,4:6) = oldPts(parentHood,4:6);
% 
% 
% n = currRotM(:,3)';
% oldPts(1,1:3) = oldPts(1,1:3) - (F_vectorDot3D(n,(oldPts(1,1:3)-currPt(1,1:3))).*n);
% uRef = oldPts(1,1:3) - currPt(1,1:3);
% uRef = uRef./norm(uRef);
% v = cross(uRef,n);
% R = [uRef' v' n'];
% 
% testPoints = newPts(:,1:3) - (F_vectorDot3D(repmat(n,size(newPts(:,1:3),1),1),(newPts(:,1:3)-repmat(currPt(1,1:3),size(newPts(:,1:3),1),1))).*repmat(n,size(newPts(:,1:3),1),1));
% testPoints = testPoints - repmat(currPt(1,1:3),size(newPts(:,1:3),1),1);
% testPoints = testPoints*R;
% angs = atan2(testPoints(:,2),testPoints(:,1));

if rDir
    i = find(angs>0);
    i = i(find(angs(i)==min(angs(i)),1));
else
    i = find(angs<0);
    i = i(find(angs(i)==max(angs(i)),1));
end
P = newPts(i,:);


% v1 = targetPt(1,1:3) - poses(2,1:3);
% v2 = poses(3,1:3) - poses(2,1:3);
% ang = acos(F_vectorDot3D(v1,v2)./(vectorNorm3d(v1).*vectorNorm3d(v2)));

if n>1
    dist = sqrt(sum((poses(3,1:3)-P(1,1:3)).^2,2));
    if dist<=(sqrt(2)*r)
        ang = pi/4;
    else
        ang = inf;
    end
else
    ang = nan;
end


R = eul2rotm(deg2rad(P(1,4:6)),'ZYX');

scanData.nPts = scanData.nPts + 1;
scanData.pts(scanData.nPts,:) = P;
scanData.nViaPts = scanData.nViaPts + 1;
scanData.iViaPts(scanData.nViaPts) = scanData.nPts;
scanData.isAcquPt(scanData.nViaPts) = true;


end

