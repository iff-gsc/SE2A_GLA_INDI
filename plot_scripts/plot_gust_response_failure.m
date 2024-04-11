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

%% Run simulations

aircraft.actuators.LAD.defl_rate_max = deg2rad(100);
aircraft.actuators.LAD.defl_rate_min = -deg2rad(100);

gla_indi = glaIndiCreate( aircraft, fp_spec );

time = 1.0;

T_delay = 0.02;

omega_sens = 500;

[boost_servo,boost_aero] = delay2Booster( T_delay, omega_sens, gla_indi.dtc, gla_indi.atc );

gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
    'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8 );
gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));

gla_indi.ca.W_v(1,1) = 2.5e-3;
gla_indi.ca.gamma = 100;
gla_indi.ca.W_v(3,3) = 1e-1; % improve robustness

gust_grad_dist = 180;
is_gla_enabled = true;
is_failure = false;
simout_healthy = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);
is_failure = true;
simout_failure = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);

%% Plot WRBM
ds = 5;
fig1=figure;
ah = axes;
hhl=plot(simout_healthy.WBM.Time(1:ds:end),simout_healthy.WBM.Data(1:ds:end,5)/simout_healthy.WBM.Data(1,5));
hold on
hfl=plot(simout_failure.WBM.Time(1:ds:end),simout_failure.WBM.Data(1:ds:end,5)/simout_failure.WBM.Data(1,5));
hfr=plot(simout_failure.WBM.Time(1:ds:end),simout_failure.WBM.Data(1:ds:end,6)/simout_failure.WBM.Data(1,6),'Color',hfl.Color,'LineStyle','--');

xlabel('Time, s')
ylabel('Relative WRBM')
grid on

legend(ah,[hhl,hfl,hfr],...
    'Healthy','Fault (faulty side)','Fault (healthy side)','location','northeast');


%% Plot load factor
ds = 5;
h = {};
Legend = {};
fig2=figure;
hold on
plot(simout_healthy.acc.Time(1:ds:end),-simout_healthy.acc.Data(1:ds:end)/9.81+1);
plot(simout_failure.acc.Time(1:ds:end),-simout_failure.acc.Data(1:ds:end)/9.81+1);

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

legend('Healthy','Fault','interpreter','latex')

%% Export figure to TikZ
figure(fig1)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_failure.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig2)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_failure_acc.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
