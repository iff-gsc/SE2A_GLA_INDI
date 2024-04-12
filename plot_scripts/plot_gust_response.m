clear all;

% add folders to path
is_tigl_installed = addPath();
is_tigl_installed = false;

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

time = 1.4;
is_failure = false;
omega_sens = 500;

boost_servo = 1;
boost_aero = 1;

gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
    'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8, 'ModeControlIdx', [1,7], 'WeightModes', [1,1e-3] );
gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));

gla_indi.ca.W_v(1,1) = 2.5e-3;
gla_indi.ca.W_v(3,3) = 1e-9;
gla_indi.ca.gamma = 100;

gust_grad_dist = [60,120,240,360,480,700]/2;

simout_0  = {};
simout_1  = {};

open(model);
set_param(model,'SimulationCommand','update');
set_param(model,"FastRestart","on");
for i = 1:length(gust_grad_dist)
    simout_0{i} = simGust(gust_grad_dist(i),time,0,is_failure);
    simout_1{i} = simGust(gust_grad_dist(i),time,1,is_failure);
end
set_param(model,"FastRestart","off");

%% Plot WRBM

WBM0 = simout_0{1}.WBM.Data(1,5);
ds = 5;
h0 = {};
h1 = {};
Legend = {};
figtmp = figure;
h = plot([0,1],rand(2,length(gust_grad_dist)));
colors = {h.Color};
close(figtmp)
fig1 = figure;
hold on
for i = 1:length(gust_grad_dist)
    h0{i} = plot(simout_0{i}.WBM.Time(1:ds:end),simout_0{i}.WBM.Data(1:ds:end,5)/WBM0,'Color',colors{i},'LineStyle','--');
    h1{i} = plot(simout_1{i}.WBM.Time(1:ds:end),simout_1{i}.WBM.Data(1:ds:end,5)/WBM0,'Color',colors{i},'LineStyle','-');
    Legend{i} = ['$\lambda=',num2str(2*gust_grad_dist(i)),'$\,ft'];
end
grid on
box on
xlim([0,time])
xlabel('Time, s')
ylabel('Relative WRBM')

legend([h1{:}],Legend{:},'interpreter','latex')

%% Plot load factor

ds = 5;
h0 = {};
h1 = {};
Legend = {};
figtmp = figure;
h = plot([0,1],rand(2,length(gust_grad_dist)));
colors = {h.Color};
close(figtmp)
fig2 = figure;
hold on
for i = 1:length(gust_grad_dist)
    h0{i} = plot(simout_0{i}.acc.Time(1:ds:end),-simout_0{i}.acc.Data(1:ds:end)/9.81+1,'Color',colors{i},'LineStyle','--');
    h1{i} = plot(simout_1{i}.acc.Time(1:ds:end),-simout_1{i}.acc.Data(1:ds:end)/9.81+1,'Color',colors{i},'LineStyle','-');
%     Legend{i} = ['$\lambda=',num2str(2*gust_grad_dist(i)),'$\,ft'];
end
grid on
box on
xlim([0,time])
xlabel('Time, s')
ylabel('Load factor')

% legend([h1{:}],Legend{:},'interpreter','latex')


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


%% Export figure to TikZ
figure(fig1)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_max_wrbm.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig2)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_max_acc.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
