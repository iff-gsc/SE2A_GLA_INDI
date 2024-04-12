clear all;

is_tigl_installed = addPath();

is_tikz_export_desired = false;

%% Init aircraft
fp_spec.Altitude	= 6000; % m
fp_spec.EAS         = 177; % m/s

[Ma,V,atm] = altEas2MaTas( fp_spec.Altitude, fp_spec.EAS );

if is_tigl_installed
    [aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'Mach', Ma, 'pchfilename', 'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true );
else
    load('data/aircraft_structure.mat');
    wingSetCustomActuatorPath(aircraft.wing_main);
end

%% Plot control effectiveness
gla_indi = glaIndiCreate( aircraft, fp_spec, ...
    'ModeControlIdx', [1,7], 'WeightModes', [1,1e-1] );

G = indiCeLadVar( gla_indi.ce, atm.rho, V, Ma );

figure

xlabel('Flap number')


% matlab2tikz does not work for yyaxis, so plotyy is used instead

Delta_y = diff(aircraft.wing_main.geometry.line_25.pos(2,:));
y1 = -[G(1,1:19),NaN,NaN,G(1,20:end)];
y2 = [G(2,1:19),NaN,NaN,G(2,20:end)];
y12 = y1./Delta_y/mean(Delta_y);
y22 = y2./Delta_y/mean(Delta_y);
u = 1:1:(size(y1,2));

u_stairs = [u(1)-0.5,repelem(u(1:end-1),2)+0.5,u(end)+0.5];
y11_stairs = repelem(y1,2);
y22_stairs = repelem(y2,2);
y1_stairs = repelem(y12,2);
y2_stairs = repelem(y22,2);

[hAx,hLine1,hLine2] = plotyy( u_stairs, y11_stairs, u_stairs, y22_stairs );

ylabel(hAx(1),'Control effectiveness $-\m{G}_{a_z}$, m/s\textsuperscript{2}','interpreter','latex')
ylabel(hAx(2),'Control effectiveness $\m{G}_{\eta_1}$','interpreter','latex')
xlabel('Strip number')

hAx(1).XLim = [0.5,length(y1)+0.5];
hAx(2).XLim = [0.5,length(y1)+0.5];
hAx(1).XTick = [1,10:10:40];
hAx(2).XTick = [1,10:10:40];

hAx(2).YLim = [0,120];
hAx(2).YTick = 0:30:120;

hAx(1).YTick = 0:2.5:10;

grid on
box on

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=3'};
    filename = exportFilename('control_effectiveness.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end


%% TiKZ ticks font size must be adjusted

str = fileread( filename );
str = strrep(str,'/.append style={font=\color{mycolor','/.append style={font=\tikzfontsize\color{mycolor');
fid = fopen( filename, 'wt' );
fprintf(fid,'%s\n',str);
fclose(fid);
