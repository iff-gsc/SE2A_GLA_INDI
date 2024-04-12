clear all;

% add folders to path
is_tigl_installed = addPath();

is_tikz_export_desired = false;

%% Model initialization

% flight point
fp_spec.Altitude	= 6000; % m
fp_spec.EAS         = 177; % m/s

[Ma,~] = altEas2MaTas( fp_spec.Altitude, fp_spec.EAS );

% aircraft parameters
if is_tigl_installed
    [aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'unsteady', true, 'stall', false, 'Mach', Ma, 'pchfilename','na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true, 'ControlsMainFile', 'wingControls_params_mainTefRedCm' );
else
    load('data/aircraft_structure.mat');
    wingSetCustomActuatorPath(aircraft.wing_main);
end

% environment parameters
envir = envirLoadParams('envir_params_default');

% Trim aircraft
tp_indi = trimFlexibleUnsteady(aircraft,structure,fp_spec,'lin_flexible_steady');

ic = tpGenerateIC(tp_indi);


%% WRBM contributions for specified trim point

T_wrbm=structureGetCutLoadTrafoAt(aircraft.eom_flexible.structure_red,structure,0.1,'Mx',0);
T_wrbm(1:6)=[];

WRBM = T_wrbm * tpGetState(tp_indi,'eta__')';


figure

bar(T_wrbm.*tpGetState(tp_indi,'eta__') / WRBM);

xlim([0,length(T_wrbm)+1])
grid on

xlabel('Mode shape number');
ylabel('Relative WRBM contribution')

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    extra_axis_options2 = [extra_axis_options,{'legend columns=2'}];
    filename = exportFilename('wrbm_mode_contributions.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options2);
end