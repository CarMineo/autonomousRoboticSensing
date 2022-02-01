function [thetaPts,thetaIntervals,outOfCyl] = F_itersectionCirCyl(C,u,v,zmin,zmax)
%F_INTERSECTIONCIRCYL computes the intersection between a circumference 
%and a cylinder. This is a fundamentally important function in the
%execution of the autonomous robotic sensing framework described into the
%paper titled: "Autonomous Robotic Sensing for Simultaneous Geometric and
%Volumetric Inspection of Free-Form Parts", by C. Mineo, D. Cerniglia and
%A. Poole.
%   [thetaPts,thetaIntervals,outOfCyl] = F_itersectionCirCyl(C,u,v,zmin,zmax)
%
%   Inputs:
%       C - (1 x 3 double) Centre of circumference
%       u - (1 x 3 double) Vector U of circumference plane
%       v - (1 x 3 double) Vector V of circumference plane (perpendicular
%           to U). The length of U and V must correspond to the radius of
%           the cicumference.
%       zmin and zmax - Two scalar indicating the vertical span of the
%                       cylinder. The cylinder is assumed to have axis
%                       coincident to the z-axis and unitary radius.
%
%   Outputs:
%       thetaPts - angular coordinates of the intersection points, in the
%                  reference system defined by U and V.
%       thetaIntervals - Angular intervals delimited by the intersection
%                        points.
%       outOfCyl - a Boolean array of the same length of thetaIntervals,
%                  indicating if for each interval the corresponding
%                  circunference arc is outside the cylinder (TRUE) or
%                  inside the cylinder (FALSE).
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_itersectionCirCyl v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

numCir = size(C,1);
a = C(:,1);
b = C(:,2);
c = u(:,1);
d = u(:,2);
e = v(:,1);
f = v(:,2);

c5 = -1 + (a.^2) + (b.^2) + (2*a.*c) + (c.^2) + (2*b.*d) + (d.^2);
c4 = 4*((a.*e) + (c.*e) + (b.*f) + (d.*f));
c3 = -2 + (2*(a.^2)) + (2*(b.^2)) - (2*(c.^2)) - (2*(d.^2)) + (4*(e.^2)) + (4*(f.^2));
c2 = 4*((a.*e) - (c.*e) + (b.*f) - (d.*f));
c1 = -1 + (a.^2) + (b.^2) - (2*a.*c) + (c.^2) - (2*b.*d) + (d.^2);

thetaPts = cell(1,numCir);
thetaIntervals = cell(1,numCir);
outOfCyl = cell(1,numCir);


% figure(10);
% t = linspace(-100000,100000,100);
% y = (c1(1).*t.^4) + (c2(1).*t.^3) + (c3(1).*t.^2) + (c4(1).*t) + c5(1);
% plot(t,y); grid on; hold on;
% 
% load('difficultInputs.mat')
% u = double(single(u./norm(u)));
% v = double(single(v./norm(v)));
% a = C(:,1);
% b = C(:,2);
% c = u(:,1);
% d = u(:,2);
% e = v(:,1);
% f = v(:,2);
% 
% c5 = -1 + (a.^2) + (b.^2) + (2*a.*c) + (c.^2) + (2*b.*d) + (d.^2);
% c4 = 4*((a.*e) + (c.*e) + (b.*f) + (d.*f));
% c3 = -2 + (2*(a.^2)) + (2*(b.^2)) - (2*(c.^2)) - (2*(d.^2)) + (4*(e.^2)) + (4*(f.^2));
% c2 = 4*((a.*e) - (c.*e) + (b.*f) - (d.*f));
% c1 = -1 + (a.^2) + (b.^2) - (2*a.*c) + (c.^2) - (2*b.*d) + (d.^2);
% y = (c1(1).*t.^4) + (c2(1).*t.^3) + (c3(1).*t.^2) + (c4(1).*t) + c5(1);
% plot(t,y);

for j = 1:numCir
    t = roots([c1(j) c2(j) c3(j) c4(j) c5(j)]);
    
    ang = 0;
    if length(t)<4
        t0 = t;
        while (length(t0)<4) && (ang<((3/2)*pi))
            ang = ang + (pi/2);
            Rz = [cos(ang) -sin(ang);
                sin(ang)  cos(ang)];
            ur = (Rz*u(1,1:2)')';
            vr = (Rz*v(1,1:2)')';
            cr = ur(:,1);
            dr = ur(:,2);
            er = vr(:,1);
            fr = vr(:,2);
            
            c5 = -1 + (a.^2) + (b.^2) + (2*a.*cr) + (cr.^2) + (2*b.*dr) + (dr.^2);
            c4 = 4*((a.*er) + (cr.*er) + (b.*fr) + (dr.*fr));
            c3 = -2 + (2*(a.^2)) + (2*(b.^2)) - (2*(cr.^2)) - (2*(dr.^2)) + (4*(er.^2)) + (4*(fr.^2));
            c2 = 4*((a.*er) - (cr.*er) + (b.*fr) - (dr.*fr));
            c1 = -1 + (a.^2) + (b.^2) - (2*a.*cr) + (cr.^2) - (2*b.*dr) + (dr.^2);
            
            t0 = roots([c1(j) c2(j) c3(j) c4(j) c5(j)]);
        end
        
        if length(t0)>length(t)
            t = t0;
        end
    end
    
    valid = false(length(t),1);
    for i = 1:length(t)
        valid(i) = (isreal(t(i)) & ~isnan(t(i)) & (abs(t(i))<inf));
    end
    t = t(valid);
    cost = (1-(t.^2))./(1+(t.^2));
    sint = (2*t)./(1+(t.^2));
    
    theta = atan2(sint,cost);
    theta = theta + ang;
    %theta(theta<0) = theta(theta<0) + (2*pi);
    
    theta = mod(theta,(2*pi));
    if isempty(find(theta==0, 1))
        theta = [0; theta];
    end
    if isempty(find(theta==(2*pi), 1))
        theta = [theta; (2*pi)];
    end

%     
%     if isempty(theta)
%         theta = [0; (2*pi)];
%     else
%         theta = [0; theta; (2*pi)];
%     end

    
    theta = uniquetol(theta,eps('single'));
    thetaInts = [theta(1:end-1) theta(2:end)];
    
    if size(thetaInts,1) == 1
        meanPts = C(j,1:2);
    else
        thetaMeans = mean(thetaInts,2);
        meanPts = [(a(j)   + c(j)*cos(thetaMeans)   + e(j)*sin(thetaMeans))...
                   (b(j)   + d(j)*cos(thetaMeans)   + f(j)*sin(thetaMeans))];
    end
    
    outFlag = (sqrt(sum((meanPts.^2),2))>1);
    
    if ~isempty(find(~outFlag,1))
        if (zmin>-inf) || (zmax<inf)
            if (zmin>-inf)
                thetaAboveZmin = F_itersectionCirPlane(C,u,v,zmin);
                if (zmax==inf)
                    thetaBelowZmax = [0;(2*pi)];
                end
            end
            if (zmax<inf)
                thetaBelowZmax = F_itersectionCirPlane(C,u,v,zmax);
                if (zmin==-inf)
                    thetaAboveZmin = [0;(2*pi)];
                end
            end
            
            theta = [theta;thetaAboveZmin;thetaBelowZmax];
        end
        
        theta = uniquetol(theta,eps('single'));
        
        %     x = a(j) + c(j)*cos(theta) + e(j)*sin(theta);
        %     y = b(j) + d(j)*cos(theta) + f(j)*sin(theta);
        
        %     tCyl = linspace(0,2*pi,100);
        %     xCyl = cos(tCyl);
        %     yCyl = sin(tCyl);
        %     tCir = linspace(0,2*pi,100);
        %     xCir = a(j) + c(j)*cos(tCir) + e(j)*sin(tCir);
        %     yCir = b(j) + d(j)*cos(tCir) + f(j)*sin(tCir);
        %     figure(1)
        %     plot(xCyl,yCyl,'-k','linewidth',2); hold on; grid on; axis equal;
        %     plot(xCir,yCir,'-b','linewidth',2);
        %     plot(x,y,'.r','markersize',50);
        
        thetaInts = [theta(1:end-1) theta(2:end)];
        
        if size(thetaInts,1) == 1
            meanPts = C(j,:);
        else
            thetaMeans = mean(thetaInts,2);
            meanPts = [(a(j)   + c(j)*cos(thetaMeans)   + e(j)*sin(thetaMeans))...
                (b(j)   + d(j)*cos(thetaMeans)   + f(j)*sin(thetaMeans))...
                (C(j,3) + u(j,3)*cos(thetaMeans) + v(j,3)*sin(thetaMeans))];
        end
        
        %     plot(meanPts(:,1),meanPts(:,2),'.g','markersize',50);
        outFlag = (sqrt(sum((meanPts(:,1:2).^2),2))>1) | (meanPts(:,3)<zmin) | (meanPts(:,3)>zmax);
    end
    
    if length(outFlag)>1
        intIsOut = outFlag(1);
        counter = 1;
        iStart = 1; iEnd = 1;
        thetaIntervals{j} = [];
        outOfCyl{j} = false(0);
        while counter<length(outFlag)
            counter = counter + 1;
            if (outFlag(counter)==intIsOut)
                iEnd = counter;
                if (counter==length(outFlag))
                    thetaIntervals{j} = [thetaIntervals{j}; thetaInts(iStart,1) thetaInts(iEnd,2)];
                    outOfCyl{j} = [outOfCyl{j}; intIsOut];
                end
            else
                thetaIntervals{j} = [thetaIntervals{j}; thetaInts(iStart,1) thetaInts(iEnd,2)];
                outOfCyl{j} = [outOfCyl{j}; intIsOut];
                iStart = counter;
                iEnd = counter;
                intIsOut = outFlag(counter);
                if (counter==length(outFlag))
                    thetaIntervals{j} = [thetaIntervals{j}; thetaInts(iStart,1) thetaInts(iEnd,2)];
                    outOfCyl{j} = [outOfCyl{j}; intIsOut];
                end
            end
        end
        
        if (length(outOfCyl{j})>1) && (outOfCyl{j}(1)==outOfCyl{j}(end)) && (thetaIntervals{j}(1,1)<=0) && (abs(thetaIntervals{j}(end,2)-(2*pi))<eps('single'))
            outOfCyl{j}(end) = [];
            thetaIntervals{j} = [(thetaIntervals{j}(end,1)-thetaIntervals{j}(end,2)) thetaIntervals{j}(1,2);
                thetaIntervals{j}(2:end-1,:)];
        end
    else
        thetaIntervals{j} = thetaInts;
        outOfCyl{j} = outFlag;
    end

    thetaPts{j} = thetaIntervals{j}(:);
    thetaPts{j}(thetaPts{j}<0) = thetaPts{j}(thetaPts{j}<0) + (2*pi);
    thetaPts{j}(abs(thetaPts{j}-(2*pi))<eps('single')) = 0;
    thetaPts{j} = uniquetol(thetaPts{j},eps('single'));
    if length(thetaPts{j})==1
        thetaPts{j} = [];
    end
end

if numCir==1
    thetaPts = thetaPts{1};
    thetaIntervals = thetaIntervals{1};
    outOfCyl = outOfCyl{1};
end


end

