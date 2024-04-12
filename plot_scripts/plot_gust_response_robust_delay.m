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

aircraft.actuators.LAD.defl_rate_max = deg2rad(100);
aircraft.actuators.LAD.defl_rate_min = -deg2rad(100);

gla_indi = glaIndiCreate( aircraft, fp_spec );

time = 1.0;
is_failure = false;
is_gla_enabled = true;
gust_grad_dist = 180;

T_delay = 0.02;

omega_sens = 500;

[boost_servo,boost_aero] = delay2Booster( T_delay, omega_sens, gla_indi.dtc, gla_indi.atc );

gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
    'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8 );
gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));

gla_indi.ca.W_v(1,1) = 2.5e-3;
gla_indi.ca.gamma = 100;
gla_indi.ca.W_v(3,3) = 1e-1; % improve robustness

transport_delay = [0,1,2,3]/600;

open(model);
set_param(model,'SimulationCommand','update');
set_param(model,"FastRestart","on");
for i = 1:length(transport_delay)
    simout{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure,'TransportDelay',transport_delay(i));
end
set_param(model,"FastRestart","off");

%% Plot WRBM
ds = 5;
Legend = {};
fig1=figure;
hold on
for i = 1:length(transport_delay)
    plot(simout{i}.WBM.Time(1:ds:end),simout{i}.WBM.Data(1:ds:end,5)/simout{i}.WBM.Data(1,5));
    Legend{i} = ['$\tau=',num2str(transport_delay(i)/gla_indi.ts),'/',num2str(1/gla_indi.ts),'$\,s'];
end

xlabel('Time, s')
ylabel('Relative WRBM')
grid on
box on
ylim([0.5,2.0])

legend(Legend{:},'location','northeast','interpreter','latex');

%% Plot load factor
ds = 5;
Legend = {};
fig2=figure;
hold on
for i = 1:length(transport_delay)
    plot(simout{i}.acc.Time(1:ds:end),-simout{i}.acc.Data(1:ds:end)/9.81+1);
    Legend{i} = ['$\tau=',num2str(transport_delay(i)/gla_indi.ts),'/',num2str(1/gla_indi.ts),'$\,s'];
end

xlabel('Time, s')
ylabel('Load factor')
grid on
box on
ylim([0.5,2.0])

% legend(Legend{:},'location','northeast','interpreter','latex');

%% Export figure to TikZ
figure(fig1)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_transport_delay_wrbm.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig2)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_transport_delay_acc.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
