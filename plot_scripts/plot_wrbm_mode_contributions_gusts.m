clear all;

% add folders to path
is_tigl_installed = addPath();

is_tikz_export_desired = false;

%% Simulink model initialization

model = 'sim_flexible_unsteady_indi';

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

% Temporary airplane pitch rate controller
pitch_rate_controller = loadParams( 'pitchRateController_params_se2a' );

% GLA INDI controller parameters
gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', 500, ...
    'BoostServos', 3, 'BoostUnstAeroMin', 3, 'BoostUnstAeroMax', 8 );

% Trim aircraft
tp_indi = trimFlexibleUnsteady(aircraft,structure,fp_spec,model);

% init controller steady state
tp_indi = tpInitController(tp_indi,gla_indi);

ic = tpGenerateIC(tp_indi);


%% Run simulations

simout = {};

gust_grad_dist_1  = [30,45,67.5,101.25,151.875,227.8125,350];
gust_grad_dist_vec = sort([gust_grad_dist_1,gust_grad_dist_1(1:end-1)+diff(gust_grad_dist_1)/2]);
time = 1.0;
is_gla_enabled = false;
is_failure = false;

for i = 1:length(gust_grad_dist_vec)
    gust_grad_dist = gust_grad_dist_vec(i);
    simout{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);
end

%% Plot WRBM contributions for specified trim point

figure

T_wrbm=structureGetCutLoadTrafoAt(aircraft.eom_flexible.structure_red,structure,0.1,'Mx',0);
T_wrbm(1:6)=[];

mode_idx = [1,3,5,7,16];
wrbm_max = [];
Legend = {};
WRBM_0 = T_wrbm*simout{1}.eta.Data(1,7:end)';
for i = 1:length(simout)
    [WRBM_max(i),idx_max] = max(T_wrbm*simout{i}.eta.Data(:,7:end)');
%     wrbm_max(i,:) = max(T_wrbm(mode_idx).*simout{i}.eta.Data(:,6+mode_idx));
    wrbm_max(i,:) = T_wrbm(mode_idx).*simout{i}.eta.Data(idx_max,6+mode_idx);
end
Legend{1} = 'All modes';
for j = 1:length(mode_idx)
    Legend{j+1} = ['Mode ',num2str(mode_idx(j))];
end
% WRBM_ref = WRBM_max;
WRBM_ref = WRBM_0;
plot(gust_grad_dist_vec,WRBM_max./WRBM_ref)
hold on
plot(gust_grad_dist_vec,wrbm_max./WRBM_ref')
legend(Legend{:},'location','east')
grid on
xlim([min(gust_grad_dist_vec),max(gust_grad_dist_vec)]);

xlabel('Gust gradient distance, ft');
ylabel('Max. rel. WRBM contribution');

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=2'};
    filename = exportFilename('wrbm_mode_contributions_gusts.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end