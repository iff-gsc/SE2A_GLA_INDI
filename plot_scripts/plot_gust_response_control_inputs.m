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
[aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'unsteady', true, 'stall', false, 'Mach', Ma, 'pchfilename', 'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true, 'ControlsMainFile','wingControls_params_mainTefRedCm' );

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

omega_sens = 500;
T_delay = 0.02;
gust_grad_dist = 180;
time = 0.8;
is_gla_enabled = true;
is_failure = false;


[boost_servo,boost_aero] = delay2Booster( T_delay, omega_sens, gla_indi.dtc, gla_indi.atc );

aircraft.actuators.LAD.defl_rate_max = deg2rad(10000);
aircraft.actuators.LAD.defl_rate_min = -deg2rad(10000);

gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', 500, ...
	'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8, ...
	'ModeControlIdx', [1,7], 'WeightModes', [1,1e-3] );

gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));

gla_indi.ca.W_v(1,1) = 2.5e-3;
gla_indi.ca.W_v(3,3) = 1e-9;
gla_indi.ca.gamma = 100;

simout = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);

%% Plot local control inputs
fig_temp = figure;
h = plot(rand(2,3),rand(2,3));
c = get(h,'Color');
colors_default = reshape([c{:}],3,[]);
close(fig_temp);


figure
hold on

idx_flap_leg = 1:6:19;
idx_flap = 1:19;
ds = 5;

xlabel('Time, s')
ylabel('Flap commands')

Legend = {};
hl = {};


colors = interp1([1,9,19]',colors_default(:,end:-1:1)',1:19)';
for i = 1:length(idx_flap)
    h = plot(simout.u.Time(1:ds:end),simout.u.Data(1:ds:end,idx_flap(i)),'Color',colors(:,i));
    if any(i==idx_flap_leg)
        hl{end+1} = h;
        Legend{end+1} = ['Strip ',num2str(idx_flap(i))];
    end
end

legend([hl{:}],Legend{:},'location','southeast')
grid on
box on

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_control_input.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Plot local lift coefficient
figure
hold on

idx_flap_leg = 1:6:19;
idx_flap = 1:19;

xlabel('Time, s')
ylabel('Local lift coefficient')

Legend = {};
hl = {};
for i = 1:length(idx_flap)
	h=plot(simout.c_L.Time(1:ds:end),squeeze(simout.c_L.Data(1,idx_flap(i),1:ds:end))','Color',colors(:,i));
    if any(i==idx_flap_leg)
        hl{end+1} = h;
        Legend{end+1} = ['Strip ',num2str(idx_flap(i))];
    end
end

% legend([hl{:}],Legend{:},'location','south')
grid on
box on

ylim([0,inf])

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=2'};
    filename = exportFilename('gust_local_c_L.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
