% add folders to path
addPath();

is_tikz_export_desired = false;

%% Init aircraft
pch_filenames = {...
    'na_Se2A-MR-Ref-v4-twist_25g_GFEM_MTOAa_S103_DMIG.pch';
    'na_Se2A-MR-Ref-v4-twist_25g_GFEM_MOOee_S103_DMIG.pch';
    'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch';
    'na_Se2A-MR-Ref-v4-twist_GFEM_MOOee_S103_DMIG.pch' ...
    };
names = { '25g_MTOW', '25g_OEW', '20g_MTOW', '20g_OEW' };
grid_foldername = 'GRID_SE2A_MR_BWD_swept_V4_twist';

i = 3;

pch_filename = pch_filenames{i};
[aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'unsteady', true, 'stall', true, 'Mach', 0.76, ...
    'pchfilename',pch_filename,'gridfoldername',grid_foldername);

%% Plot mass distribution
figure
structurePlot(structure,'MassColor',[0.8,0.2,0.2],'StructureColor',[0,0,0],'MassSize',0.75,'LineWidth',0.33,'NodeSize',0.3);
grid off
xlabel('');
ylabel('');
zlabel('');
% xlim([-35,-10])
% ylim([-21,21])
% zlim([-4,-2])

view(135,35)
% axis off

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    extra_axis_options = {'line width=\tikzlinewidth','ticks=none','axis line style={draw=none}','tick style={draw=none}','xlabel={}','ylabel={}','zlabel={}'};
    filename = exportFilename(['structure_mass_',names{i},'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraAxisOptions',extra_axis_options);

    % Make TiKZ mass spheres transparent 
    str = fileread( filename );
    str = strrep(str,'fill=red!60!gray,','fill=red!60!gray, fill opacity=0.5,');
    fid = fopen( filename, 'wt' );
    fprintf(fid,'%s\n',str);
    fclose(fid);
end
