function [] = addPath()

% go to correct directory
this_function_path = fileparts(mfilename('fullpath'));
cd(this_function_path);
cd ..

% add folders to path
addpath(genpath(pwd));

cd(this_function_path);

% add to path TiGL and TiXI
addPathTiGL('2.2.3');

end
