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

tic

open(model);
set_param(model,'SimulationCommand','update');
set_param(model,"FastRestart","on");

time = 0.8;

gust_grad_dist_1  = [30,45,67.5,101.25,151.875,227.8125,350];
gust_grad_dist = sort([gust_grad_dist_1,gust_grad_dist_1(1:end-1)+diff(gust_grad_dist_1)/2]);
is_failure      = false;
is_gla_enabled  = true;
omega_sens      = 500;

T_delay_1 = [100,0.16,0.08,0.04,0.02,0.01];
T_delay = sort([T_delay_1,T_delay_1(2:end-1)+diff(T_delay_1(2:end)/2)],'descend');
simout  = {};
simout_nrl = {};

for i = 1:length(T_delay)
    
    for j = 1:length(gust_grad_dist)
        
        disp(['Current parameters: T=',num2str(T_delay(i)),'s, H=',num2str(gust_grad_dist(j)),'m']);
    
        [boost_servo,boost_aero] = delay2Booster( T_delay(i), omega_sens, gla_indi.dtc, gla_indi.atc );
        
        aircraft.actuators.LAD.defl_rate_max = deg2rad(10000);
        aircraft.actuators.LAD.defl_rate_min = -deg2rad(10000);

        gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
            'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8, ...
            'ModeControlIdx', [1,7], 'WeightModes', [1,1e-12] );
        gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));
        gla_indi.ca.W_v(1,1) = 0.0025;
        gla_indi.ca.W_v(2,2) = 1;
        gla_indi.ca.W_v(3,3) = 1e-9;
        gla_indi.ca.gamma = 100;

        simout_nrl{i,j} = simGust(gust_grad_dist(j),time,is_gla_enabled,is_failure);
        
        
        aircraft.actuators.LAD.defl_rate_max = deg2rad(100);
        aircraft.actuators.LAD.defl_rate_min = -deg2rad(100);
        
        gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
            'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8, ...
            'ModeControlIdx', [1,7], 'WeightModes', [1,1e-12] );
        gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));
        gla_indi.ca.W_v(1,1) = 0.0025;
        gla_indi.ca.W_v(2,2) = 1;
        gla_indi.ca.W_v(3,3) = 1e-9;
        gla_indi.ca.gamma = 100;
        
        
        simout{i,j} = simGust(gust_grad_dist(j),time,is_gla_enabled,is_failure);
        
        disp(['Progress: ',num2str(100*((i-1)/length(T_delay)+1/length(T_delay)*j/length(gust_grad_dist))),'%'])
        
    end
    
end

set_param(model,"FastRestart","off");
toc


%% Post-processing
WRBM_rel = [];
Load_factor = [];
WRBM_rel_nrl = [];
Load_factor_nrl = [];
gust_grad = linspace(min(gust_grad_dist),max(gust_grad_dist),100);
gust_grad_max = [];
gust_grad_max_nrl = [];
WRBM_max = [];
WRBM_max_nrl = [];
Load_factor_max = [];
Load_factor_max_nrl = [];
for i = 1:size(simout,1)
    for j = 1:size(simout,2)
%         WRBM_rel(i,j) = 1 + max(abs(simout{i,j}.WBM.Data(:,5)-simout{i,j}.WBM.Data(1,5))/simout{i,j}.WBM.Data(1,5));
%         Load_factor(i,j) = 1 + max(abs(-simout{i,j}.acc.Data)/9.81);
%         WRBM_rel_nrl(i,j) = 1 + max(abs(simout_nrl{i,j}.WBM.Data(:,5)-simout_nrl{i,j}.WBM.Data(1,5))/simout_nrl{i,j}.WBM.Data(1,5));
%         Load_factor_nrl(i,j) = 1 + max(abs(-simout_nrl{i,j}.acc.Data)/9.81);
        WRBM_rel(i,j) = 1 + max((simout{i,j}.WBM.Data(:,5)-simout{i,j}.WBM.Data(1,5))/simout{i,j}.WBM.Data(1,5));
        Load_factor(i,j) = 1 + max((-simout{i,j}.acc.Data)/9.81);
        WRBM_rel_nrl(i,j) = 1 + max((simout_nrl{i,j}.WBM.Data(:,5)-simout_nrl{i,j}.WBM.Data(1,5))/simout_nrl{i,j}.WBM.Data(1,5));
        Load_factor_nrl(i,j) = 1 + max((-simout_nrl{i,j}.acc.Data)/9.81);
    end
    WRBM_gust_grad = interp1( gust_grad_dist, WRBM_rel(i,:), gust_grad, 'makima' );
    [WRBM_max(i),gust_grad_max_idx] = max( WRBM_gust_grad );
    gust_grad_max(i) = gust_grad(gust_grad_max_idx);
    Load_factor_gust_grad = interp1( gust_grad_dist, Load_factor(i,:), gust_grad, 'makima' );
    [Load_factor_max(i),~] = max( Load_factor_gust_grad );
    WRBM_gust_grad_nrl = interp1( gust_grad_dist, WRBM_rel_nrl(i,:), gust_grad, 'makima' );
    [WRBM_max_nrl(i),gust_grad_max_idx_nrl] = max( WRBM_gust_grad_nrl );
    gust_grad_max_nrl(i) = gust_grad(gust_grad_max_idx_nrl);
    Load_factor_gust_grad_nrl = interp1( gust_grad_dist, Load_factor_nrl(i,:), gust_grad, 'makima' );
    [Load_factor_max_nrl(i),~] = max( Load_factor_gust_grad_nrl );
end

%% Plot WRBM vs. gust gradient distance
Legend = {};
figure
is_mem_dist = ismember(gust_grad_dist,gust_grad_dist_1);
is_mem_T = ismember(T_delay,T_delay_1);
h1=plot(2*gust_grad_dist,WRBM_rel(is_mem_T,:));
hold on
for i = 2:length(h1)
    WRBM_rel_nrl_1 = WRBM_rel_nrl(is_mem_T,:);
    plot(2*gust_grad_dist,WRBM_rel_nrl_1(i,:),'LineStyle','--','Color',h1(i).Color)
    Legend{i} = ['$T=',num2str(T_delay_1(i)),'$\,s'];
end
Legend{1} = 'Open loop';
grid on
box on
xlim([min(2*gust_grad_dist),max(2*gust_grad_dist)])
ylim([1,4.2])
xlabel('Gust length, ft')
ylabel('Max. relative WRBM')
legend(h1,Legend{:},'interpreter','latex','location','northwest','NumColumns',2)

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=2'};
    filename = exportFilename('gust_wrbm_distance_rate.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Plot load factor vs. gust gradient distance
Legend = {};
figure
h1=plot(2*gust_grad_dist,Load_factor(is_mem_T,:));
hold on
for i = 2:length(h1)
    Load_factor_nrl_1 = Load_factor_nrl(is_mem_T,:);
    plot(2*gust_grad_dist,Load_factor_nrl_1(i,:),'LineStyle','--','Color',h1(i).Color)
    Legend{i} = ['$T=',num2str(T_delay_1(i)),'$\,s'];
end
Legend{1} = 'Open loop';
grid on
box on
xlim([min(2*gust_grad_dist),max(2*gust_grad_dist)])
ylim([1,4.2])
xlabel('Gust length, ft')
ylabel('Max. load factor')
% legend(h1,Legend{:},'interpreter','latex','location','north','NumColumns',2)

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=2'};
    filename = exportFilename('gust_acc_distance_rate.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Plot WRBM vs. control system time delay
figure
hold on
plot(T_delay(2:end),WRBM_max(2:end),'k')
plot(T_delay(2:end),WRBM_max_nrl(2:end),'k--')
xlabel('Time delay, s')
ylabel('Max. relative WRBM')
ylim([1,3.2])
grid on
box on
legend('With rate limit','Without rate limit','location','southeast')

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_wrbm_rate.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Plot load factor vs. control system time delay
figure
hold on
plot(T_delay(2:end),Load_factor_max(2:end),'k')
plot(T_delay(2:end),Load_factor_max_nrl(2:end),'k--')
xlabel('Time delay, s')
ylabel('Max. load factor')
ylim([1,3.2])
grid on
box on
% legend('With rate limit','Without rate limit','location','southeast')

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_acc_rate.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
