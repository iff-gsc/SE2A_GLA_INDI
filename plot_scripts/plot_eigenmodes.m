clear all;

is_tigl_installed = addPath();

is_tikz_export_desired = false;

%% Init aircraft
[aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'pchfilename', 'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true );

%% Plot eigenmodes
mode_nums = [1,3,5,7];
Scaling = 3;

for i = 1:length(mode_nums)
    mode_num = mode_nums(i);

    figure
    wingPlotEigenmode(aircraft.wing_main,aircraft.eom_flexible.structure_red,mode_num,'Scaling',Scaling,'CntrlPts','off')
    hold on
    wingPlotEigenmode(aircraft.wing_htp,aircraft.eom_flexible.structure_red,mode_num,'Scaling',Scaling,'CntrlPts','off')
    hold on
    wingPlotEigenmode(aircraft.wing_vtp,aircraft.eom_flexible.structure_red,mode_num,'Scaling',Scaling,'CntrlPts','off')
    hold on
    fuselagePlotEigenmode(aircraft.fuselage,aircraft.eom_flexible.structure_red,mode_num,'CircRes',18,'Scaling',Scaling,'CntrlPts','off')
    
    grid off
    xlabel('');
    ylabel('');
    zlabel('');

    view(135,20)

    %% Export figure to TikZ
    if is_tikz_export_desired
        tikzwidth = '\figurewidth';
        tikzheight = '\figureheight';
        extra_axis_options = {'line width=\tikzlinewidth','ticks=none','axis line style={draw=none}','tick style={draw=none}','xlabel={}','ylabel={}','zlabel={}'};
        filename = exportFilename('control_allocation_4.tex');
        matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraAxisOptions',extra_axis_options);
    end
end
