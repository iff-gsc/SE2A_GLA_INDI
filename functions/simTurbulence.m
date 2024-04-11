function simout = simTurbulence(turbulence_rms,time,is_gla_enabled,is_failure)

model = 'sim_flexible_unsteady_indi_turbulence';

load_system(model);
turbulence_block = find_system(model,'SearchDepth',2,'LookUnderMasks','on','Name','2D Von Karman Turbulence');

set_param(turbulence_block{1},'sigma',num2str(turbulence_rms));

gain_block = find_system(model,'SearchDepth',1,'Name','LAD Gain');
if is_failure
    gain_val = '[ones(1,5),zeros(1,7),ones(1,7),ones(1,19)]';
elseif ~is_gla_enabled
    gain_val = '0';
else
    gain_val = '1';
end

set_param(gain_block{1},'Gain',gain_val);

out = sim(model,'StopTime',num2str(time));

simout = out;

set_param(gain_block{1},'Gain','1');
set_param(turbulence_block{1},'sigma','6.5');

end