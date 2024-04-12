% add folders to path
is_tigl_installed = addPath();

is_tikz_export_desired = false;

%% Init aircraft
if is_tigl_installed
    [aircraft,~] = aircraftSe2aCreate( );
else
    load('data/aircraft_structure.mat');
    wingSetCustomActuatorPath(aircraft.wing_main);
end

%% Plot wing
figure

wingPlotGeometry(aircraft.wing_main,'CntrlPts','off');

view(-90,90)

grid off
xlabel('')
ylabel('')
axis equal

axis off

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('wing_definition.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end