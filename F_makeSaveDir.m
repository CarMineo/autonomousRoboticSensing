function saveDir = F_makeSaveDir()
%F_MAKESAVEDIR creates a saving directory to save data. A folder with a
%unique name obtained by the local clock is created in .\Datasets\
%   saveDir = F_makeSaveDir()
%
%   Inputs:
%       void
%
%   Outputs:
%       saveDir - A string containing the full directory path
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% February 2022; Last revision: 01-02-2022
% Tested with: Matlab 2020b
%
%   F_makeSaveDir v1.0
%   Copyright (C) 2022 Carmelo Mineo. See COPYRIGHT.txt for details.


%------------- BEGIN CODE --------------

c = clock;
saveDir = ['.\\Datasets\\' num2str(c(1),'%.4u') '_' num2str(c(2),'%.2u') '_' num2str(c(3),'%.2u') ' - ' num2str(c(4),'%.2u') '.' num2str(c(5),'%.2u') '.' num2str(round(c(6)),'%.2u')];
mkdir(saveDir);

end

