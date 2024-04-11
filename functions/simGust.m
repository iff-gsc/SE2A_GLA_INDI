function simout = simGust(gust_grad_dist,time,is_gla_enabled,is_failure,varargin)

transport_delay = 0;
for i = 1:length(varargin)
    if strcmp(varargin{i},'TransportDelay')
        transport_delay(:) = varargin{i+1};
    end
end

model = 'sim_flexible_unsteady_indi';

load_system(model);
gust_block = find_system(model,'SearchDepth',1,'Name',['Environment',newline,'with successive 1-cos gusts']);

set_param(gust_block{1},'gustGradDist',num2str(gust_grad_dist));

gain_block = find_system(model,'SearchDepth',1,'Name','LAD Gain');
if is_failure
    gain_val = '[ones(1,5),zeros(1,7),ones(1,7),ones(1,19)]';
elseif ~is_gla_enabled
    gain_val = '0';
else
    gain_val = '1';
end

set_param(gain_block{1},'Gain',gain_val);

manual_switch_block = find_system(model,'SearchDepth',2,'Name','Manual Switch');
transport_delay_block = find_system(model,'SearchDepth',2,'Name','Transport Delay');
if transport_delay == 0
    set_param(manual_switch_block{1},'sw','0');
    set_param(transport_delay_block{1},'DelayTime','gla_indi.ts');
else
    set_param(manual_switch_block{1},'sw','1');
    set_param(transport_delay_block{1},'DelayTime',num2str(transport_delay));
end

out = sim(model,'StopTime',num2str(time));

simout = out;

set_param(gain_block{1},'Gain','1');
set_param(gust_block{1},'gustGradDist','100');

set_param(manual_switch_block{1},'sw','0');
set_param(transport_delay_block{1},'DelayTime','gla_indi.ts');

end