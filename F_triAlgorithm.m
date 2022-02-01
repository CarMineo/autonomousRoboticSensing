function [iTri,iTriEdges] = F_triAlgorithm(n_elem,n_frames,isPipe,isFlipping)
%F_TRIALGORITHM Efficient generator of triangular mesh for structured point
%clouds.
%   [iTri,iTriEdges] = F_triAlgorithm(n_elem,n_frames,isPipe,isFlipping)
%
%   Inputs:
%       n_elem = number of points per frame
%       n_frames = number of frames
%       isPipe = Boolean variable indicating if the mesh has to be closed
%       isFlipping - Boolean variable indicating if the frame points have
%                    to be flipped alternatively
%
%   Outputs:
%       iTri - Triangulation indices (nx3)
%       iTriEdges - Indices of the extremities of the distinct edges of the
%                   triangulation (mx2) 
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_triAlgorithm v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------


% CREATE TEMPLATE FOR MESHING A CIRCULAR STRIPE
templSubtri = uint32(zeros(n_elem*2,3));
sSubtri = n_elem*2;

% Triangles between two circles of points and create a circular stripe
if isFlipping
    for ielem = 1:n_elem-1
        templSubtri(((ielem-1)*2)+1:((ielem-1)*2)+2,:) = uint32([ielem (n_elem+1-ielem)+n_elem (n_elem-ielem)+n_elem; ielem (n_elem-ielem)+n_elem ielem+1]);
    end
else
    for ielem = 1:n_elem-1
        templSubtri(((ielem-1)*2)+1:((ielem-1)*2)+2,:) = uint32([ielem ielem+n_elem ielem+n_elem+1; ielem ielem+n_elem+1 ielem+1]);
    end
end

if isPipe == 1
    % Two more triangles to close the loop
    templSubtri(((n_elem-1)*2)+1:((n_elem-1)*2)+2,:) = uint32([n_elem n_elem+n_elem n_elem+1; n_elem n_elem+1 1]);
else
    templSubtri(((n_elem-1)*2)+1:((n_elem-1)*2)+2,:) = [];
    sSubtri = (n_elem-1)*2;
end

% EXTEND THE TEMPLATE TO THE FULL LENGTH OF THE PIPE
iTri = uint32(zeros((n_elem*2)*n_frames,3));
itri = 1;
for iframe = 0:n_frames-2
    istart = uint32((iframe*n_elem) + 1);
    subtri = templSubtri + (istart - 1);
    iTri(itri:(itri+sSubtri(1)-1),:) = subtri;
    itri = itri+sSubtri(1);
end
iTri(itri:end,:)=[];

iTriEdges = [iTri(:,1) iTri(:,2); iTri(:,2) iTri(:,3); iTri(:,3) iTri(:,1)];
iTriEdges = unique(sort(iTriEdges,2),'rows');
end