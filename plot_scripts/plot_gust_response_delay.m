clear all;

% add folders to path
addPath();

is_tikz_export_desired = false;

%% Simulink model initialization

model = 'sim_flexible_unsteady_indi';

% flight point
fp_spec.Altitude	= 6000; % m
fp_spec.EAS         = 177; % m/s

[Ma,~] = altEas2MaTas( fp_spec.Altitude, fp_spec.EAS );

% aircraft parameters
[aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'unsteady', true, 'stall', false, 'Mach', Ma, 'pchfilename', 'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true, 'ControlsMainFile', 'wingControls_params_mainTefRedCm' );

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

%% Run simulations (open loop)

time = 1.0;

gust_grad_dist  = 180;
is_failure      = false;
is_gla_enabled  = true;
omega_sens      = 500;

simout_open_loop = simGust(gust_grad_dist,time,false,is_failure);

T_delay = [0.16,0.08,0.04,0.02,0.01];
%% Run simulations

% no servo rate limit
aircraft.actuators.LAD.defl_rate_max = 100;
aircraft.actuators.LAD.defl_rate_min = -100;
% aircraft.actuators.LAD.defl_rate_max = deg2rad(100);
% aircraft.actuators.LAD.defl_rate_min = -deg2rad(100);
    
tic
open(model);
set_param(model,"FastRestart","on");

simout  = {};

gla_indi = glaIndiCreate( aircraft, fp_spec );

for i = 1:length(T_delay)
    
    [boost_servo,boost_aero] = delay2Booster( T_delay(i), omega_sens, gla_indi.dtc, gla_indi.atc );
    
    gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
        'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8, ...
        'ModeControlIdx', [1,7], 'WeightModes', [1,1e-3] );
    gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));
    
    gla_indi.ca.W_v(1,1) = 2.5e-3;
    gla_indi.ca.W_v(3,3) = 1e-9;
    gla_indi.ca.gamma = 100;

    simout{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);
    
end

set_param(model,"FastRestart","off");
toc

%% Plot WRBM
ds = 5;
h = {};
Legend = {};
fig1=figure;
hold on
h_ol = plot(simout_open_loop.WBM.Time(1:ds:end),simout_open_loop.WBM.Data(1:ds:end,5)/simout_open_loop.WBM.Data(1,5));
for i = 1:length(T_delay)
    h{i} = plot(simout{i}.WBM.Time(1:ds:end),simout{i}.WBM.Data(1:ds:end,5)/simout{i}.WBM.Data(1,5));
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end

xlabel('Time, s')
ylabel('Relative WRBM')
grid on
box on

legend([h_ol,h{:}],'Open loop',Legend{:},'interpreter','latex')


%% Plot load factor
ds = 5;
h = {};
Legend = {};
fig2=figure;
hold on
h_ol = plot(simout_open_loop.acc.Time(1:ds:end),-simout_open_loop.acc.Data(1:ds:end)/9.81+1);
for i = 1:length(T_delay)
    h{i} = plot(simout{i}.acc.Time(1:ds:end),-simout{i}.acc.Data(1:ds:end)/9.81+1);
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end

xlabel('Time, s')
ylabel('Load factor')
grid on
box on

ax1 = get(fig1,'CurrentAxes');
ax2 = get(fig2,'CurrentAxes');
if ax2.YLim(2) > ax1.YLim(2)
    ax1.YLim(2) = ax2.YLim(2);
else
    ax2.YLim(2) = ax1.YLim(2);
end
if ax2.YLim(1) < ax1.YLim(1)
    ax1.YLim(1) = ax2.YLim(1);
else
    ax2.YLim(1) = ax1.YLim(1);
end

% legend([h_ol,h{:}],'Open loop',Legend{:},'interpreter','latex')

%% Export figure to TikZ
figure(fig1)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_delay_wrbm.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig2)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_delay_acc.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
