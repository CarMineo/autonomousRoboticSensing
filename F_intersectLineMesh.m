function [intersectionPoints, distFromOrigin, intersectedFaces] = F_intersectLineMesh(origin,line,vertices,faces, varargin)
%F_INTERSECTLINEMESH Computes the intersection points between a single 3D
%line and a 3D mesh defined by vertices and faces. It also returns the
%position of each intersection point on the input line, and the index of
%the intersected faces. distFromOrigin > 0, the point is also on the ray
%corresponding to the line.
%   [intersectionPoints, distFromOrigin, intersectedFaces] = F_intersectLineMesh(origin,line,vertices,faces, varargin)
%
%   Inputs:
%       origin - [1x3] ray origins
%       line - [1x3] ray direction vectors
%       vertices - [mx3] vertices of mesh
%       faces - [px3] indices of points mesh faces
%
%   Outputs:
%       intersectionPoints - [nx3] intersection points
%       distFromOrigin - [nx1] distance from ray origin
%       intersectedFaces - [nx1] indices of intersected faces
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
% Acknowledgements:
%    David Legland - Author of INTERSECTLINEMESH3D.
%
%   F_intersectLineMesh v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

tol = 1e-12;
if ~isempty(varargin)
    tol = varargin{1};
end

% ensure the mesh has triangular faces
tri2Face = [];
if iscell(faces) || size(faces, 2) ~= 3
    [faces, tri2Face] = triangulateFaces(faces);
end

% find triangle edge vectors
t0  = vertices(faces(:,1), :);
u   = vertices(faces(:,2), :) - t0;
v   = vertices(faces(:,3), :) - t0;

% test if intersection point is inside triangle
% normalize direction vectors of triangle edges
uu  = dot(u, u, 2);
uv  = dot(u, v, 2);
vv  = dot(v, v, 2);

% normalization constant
D = uv.^2 - uu .* vv;

% triangle normal
n   = normalizeVector3d(vectorCross3d(u, v));
% vector between triangle origin and line origin
w0 = bsxfun(@minus, origin, t0);
a = -dot(n, w0, 2);

intersectionPoints = nan(0,3);
distFromOrigin = nan(0,1);
intersectedFaces = nan(0,1);

i=1;
%tic
% direction vector of line
dir = line(i,:);
%b = dot(n, repmat(dir, size(n, 1), 1), 2);
b = (n(:,1).*dir(1)) + (n(:,2).*dir(2)) + (n(:,3).*dir(3));

valid = abs(b) > tol & vectorNorm3d(n) > tol;

% compute intersection point of line with supporting plane
% If pos < 0: point before ray
% IF pos > |dir|: point after edge
pos = a ./ b;

% coordinates of intersection point
points = bsxfun(@plus, origin, bsxfun(@times, pos, dir));

% coordinates of vector v in triangle basis
w   = points - t0;
%     tic
%     wu  = dot(w, u, 2);
%     wv  = dot(w, v, 2);
%     toc
%     tic
wu  = (w(:,1).*u(:,1)) + (w(:,2).*u(:,2)) + (w(:,3).*u(:,3));
wv  = (w(:,1).*v(:,1)) + (w(:,2).*v(:,2)) + (w(:,3).*v(:,3));
%     toc

% test first coordinate
s = (uv .* wv - vv .* wu) ./ D;
% ind1 = s < 0.0 | s > 1.0;
ind1 = s < -tol | s > (1.0 + tol);
points(ind1, :) = NaN;
pos(ind1) = NaN;

% test second coordinate, and third triangle edge
t = (uv .* wu - uu .* wv) ./ D;
% ind2 = t < 0.0 | (s + t) > 1.0;
ind2 = t < -tol | (s + t) > (1.0 + tol);
points(ind2, :) = NaN;
pos(ind2) = NaN;

% keep only interesting points
inds = ~ind1 & ~ind2 & valid;

if ~isempty(find(inds,1))
    points = points(inds, :);
    pos = pos(inds);
    faceInds = find(inds);
    [pos,iSort] = sort(pos);
    points = points(iSort,:);
    faceInds = faceInds(iSort);
    [pos,iUnique] = uniquetol(pos,eps('single'));
    
    distFromOrigin = pos;
    intersectionPoints = points(iUnique,:);
    intersectedFaces = faceInds(iUnique);
    % convert to face indices of original mesh
    if ~isempty(tri2Face)
        for i=1:length(inds)
            intersectedFaces(i) = tri2Face(intersectedFaces(i));
        end
    end
end

