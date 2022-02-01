function [TR,FN,IC,shapeBoundary,P,TRside] = F_randomShapeGen(geomProp)
%F_RANDOMSHAPEGEN Generator of a random geometry to support the execution
%of the demostration program (main.m), related to the to the paper titled:
% "Autonomous Robotic Sensing for Simultaneous Geometric and Volumetric
% Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and A. Poole.
%   [TR,FN,IC,shapeBoundary,P,TRside] = F_randomShapeGen(geomProp)
%
%   Inputs:
%       geomProp - structured array containing the key parameters to 
%                  control the generation of a random geometry. This
%                  has the following elements:
%                  .surfType - A string indicating the type of surface
%                              'flat' - Flat surface
%                              'cylindrical' - Cylindrical surface
%                              'spherical' - Spherical surface
%                  .curvRad - Surface radius of curvature
%                  .hollow - Boolean variable indicating if the surface
%                            contains an hole.
%                  .inRad - Minimum distance of surface boundary from origin.
%                  .outRad - Maximum distance of surface boundary from origin.
%                  .baseThickness - Heigth of part base
%
%   Outputs:
%       TR - Triangulation of target surface
%       FN - Face normals
%       IC - Incenters of faces
%       shapeBoundary - geometry boundary
%       P - Random poses to serve as starting pose, to place sensor on part
%       TRside - Tringulation of side part surface
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_randomShapeGen v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.

%------------- BEGIN CODE --------------

surfType = geomProp.surfType;
curvRad = geomProp.curvRad;
holePresent = geomProp.hollow;
inRad = geomProp.inRad;
outRad = geomProp.outRad;
r = geomProp.baseThickness;

numPoints = 100;
theta = linspace(0,(2*pi),numPoints)';

if holePresent
    rectBoundary = [-1 -1 0;
                    -1  1 0;
                     1  1 0;
                     1 -1 0];
    rectBoundary = rectBoundary.*outRad;
                 
    P = rectBoundary./sqrt(2);
    xP = P(:,1);
    yP = P(:,2);
    
    
    xShape = inRad.*cos(theta);
    yShape = inRad.*sin(theta);
    
                 
    rectBoundary = [linspace(-1,-1,100)' linspace(-1,1,100)' linspace(0,0,100)';
                    linspace(-1,1,100)' linspace(1,1,100)' linspace(0,0,100)';
                    linspace(1,1,100)' linspace(1,-1,100)' linspace(0,0,100)';
                    linspace(1,-1,100)' linspace(-1,-1,100)' linspace(0,0,100)'];
    rectBoundary = rectBoundary.*outRad;
    
    xShape = [xShape; nan; rectBoundary(:,1)];
    yShape = [yShape; nan; rectBoundary(:,2)];
    
    fd=@(p) ddiff(drectangle(p,-1,1,-1,1),dcircle(p,0,0,inRad/outRad));
    fh=@(p) 0.05+0.3*dcircle(p,0,0,inRad/outRad);
    [p,t]=distmesh2d(fd,fh,0.05,[-1,-1;1,1],[-1,-1;-1,1;1,-1;1,1]);
    t = [t(:,3) t(:,2) t(:,1)];
    p = p.*outRad;
    p(:,3) = 0;
%     figure(20)
%     trimesh(t,p(:,1),p(:,2),p(:,3));
else
    degree = 5;
    
    coeffs = rand(degree,1);
    rho = zeros(size(theta));
    for i = 1:degree
        rho = rho + coeffs(i)*sin(i*theta);
    end
    
    rho = rho - min(rho);
    rho = rho./max(rho);
    rho = inRad + rho.*(outRad-inRad);
    
    [TF,prom] = islocalmax(rho);
    thetaSorted = theta(TF);
    rhoSorted = rho(TF);
    prom = prom(TF);
    [prom,iS] = sort(prom,'descend');
    thetaP = thetaSorted(iS);
    rhoP = rhoSorted(iS);
    
    if length(iS)<5
        n = 5 - length(iS);
        thetaP = [thetaP; theta(round(linspace(1+(100/(n+2)),100-(100/(n+2)),n))')];
        rhoP = [rhoP; rho(round(linspace(1+(100/(n+2)),100-(100/(n+2)),n))')];
    else
        thetaP = thetaP(1:5,:);
        rhoP = rhoP(1:5,:);
    end
    
    phase = -theta(find(rho == max(rho),1)) + ((5/4)*pi);
    [xShape,yShape] = pol2cart(theta+phase, rho);
    pv = [xShape,yShape]./outRad;
    [p,t]=distmesh2d(@dpoly,@huniform,0.05,[-1,-1; 1,1],pv,pv);
    t = [t(:,3) t(:,2) t(:,1)];
    
    p = p.*outRad;
    [xP,yP] = pol2cart(thetaP+phase, rhoP.*(0.5 + (0.5*rand(5,1))));
    xP = [0; xP];
    yP = [0; yP];
end



switch surfType
    case 'flat'
        zShape = zeros(size(xShape));
        p(:,3) = zeros(size(p(:,1)));
        n = repmat([0 0 -1],size(xP,1),1);
        zP = zeros(size(xP));
    case 'cylindrical'
        zShape = sqrt((curvRad^2) - (yShape.^2)) - curvRad;
        p(:,3) = sqrt((curvRad^2) - (p(:,2).^2)) - curvRad;
        zP = sqrt((curvRad^2) - (yP.^2));
        n = -[zeros(size(xP,1),1),yP,zP];
        zP = zP - curvRad;
    case 'spherical'
        zShape = sqrt((curvRad^2) - (xShape.^2) - (yShape.^2)) - curvRad;
        p(:,3) = sqrt((curvRad^2) - (p(:,1).^2) - (p(:,2).^2)) - curvRad;
        zP = sqrt((curvRad^2) - (xP.^2) - (yP.^2));
        n = -[xP,yP,zP];
        zP = zP - curvRad;
end

P = [xP,yP,zP];
[u,v] = F_uvRodrigues(n);

for i = 1:size(u,1)
    n(i,:) = n(i,:)./norm(n(i,:));
    R = [u(i,:)' v(i,:)' n(i,:)'];
    P(i,4:6) = rad2deg(rotm2eul(R,'ZYX'));
end

zMin = min(zShape) - r;
p(:,3) = p(:,3) - zMin;
zShape = zShape - zMin;
P(:,3) = P(:,3) - zMin; 

% nPtTri = size(p(:,3),1);
% t = [t; t+nPtTri];
% %t = t+nPtTri;
% p = [p; p(:,1) p(:,2) zeros(nPtTri,1)];

TR = triangulation(t,p(:,1),p(:,2),p(:,3));
TI = (1:size(t,1))';
FN = faceNormal(TR,TI);
IC = incenter(TR, TI);

shapeBoundary = [xShape, yShape, zShape];
if holePresent
    t = [F_triAlgorithm(100,2,1,0); F_triAlgorithm(400,2,1,0) + 200];
    TRside = triangulation(double(t),[xShape(1:100);xShape(1:100);xShape(102:end);xShape(102:end)],[yShape(1:100);yShape(1:100);yShape(102:end);yShape(102:end)],[zShape(1:100);zeros(100,1); zShape(102:end);zeros(size(zShape(102:end)))]);
else
    t = F_triAlgorithm(size(shapeBoundary,1),2,1,0);
    TRside = triangulation(double(t),[xShape;xShape],[yShape;yShape],[zShape;zeros(size(zShape))]);
end




end