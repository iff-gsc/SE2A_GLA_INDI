clear all;

addPath();

is_tikz_export_desired = false;

%%
T = 0.1;
bf = 2;

act=tf(1,[T,1]);

act_des=tf(1,[T/bf,1]);

boost=tf([bf,bf/T],[1,bf/T]);

%% Time domain

t = 0:0.01:0.5;

y1 = step(tf(1,1),t);
y2 = step(act,t);
y3 = step(boost,t);
y4 = step(boost*act,t);

figure
hold on

color_order = get(gca,'colororder');

plot(t,y1,'Color',color_order(3,:));
plot(t,y2,'Color',color_order(1,:));
plot(t,y3,'Color',color_order(2,:));
plot(t,y4,'Color',color_order(4,:));

grid on
box on
xlabel('Time, s')
ylabel('Command / Response')
legend('Actuator command $u$','Actuator response $\bar{u}$','Booster response $u_\mathrm{boost}$','Manip. actuator response $\bar{u}_\mathrm{boost}$','interpreter','latex')


%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('actuator_boost_example_complex.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end


%% Frequency domain

[z0,p0]=tf2zp(act.Numerator{:},act.Denominator{:});
% boosted_act = boost*act;
[zb,pb]=tf2zp(boost.Numerator{:},boost.Denominator{:});
marker_size = 8;
figure
hold on
% ho=plot(real(z0),imag(z0),'ko','MarkerSize',marker_size);
hx=plot(real(p0),imag(p0),'kx','MarkerSize',marker_size);
h0x=plot(real(p0),imag(p0),'x','MarkerSize',marker_size);
ho=plot(real(zb),imag(zb),'ko','MarkerSize',marker_size);
hbo=plot(real(zb),imag(zb),'o','MarkerSize',marker_size);
hbx=plot(real(pb),imag(pb),'Marker','x','Color',hbo.Color,'MarkerSize',marker_size);

h0x=plot(0,0,'LineStyle','-','Color',h0x.Color);
hbo=plot(0,0,'LineStyle','-','Color',hbo.Color);
% 
axis equal
sgrid
legend([ho,hx,h0x,hbo],'Zeros','Poles','Actuator','Booster')
xlabel('Real part')
ylabel('Imaginary part')

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}' };
    filename = exportFilename('actuator_boost_example_complex.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
    changeTexNodeFontSize(filename)
end


%%
function [] = changeTexNodeFontSize(filename)
str = fileread( filename );
% str = strrep(str,'line width=2.0pt','line width=1.0pt');
str = strrep(str,'{ylabel style={font=\color{white!15!black}}','{ylabel style={font=\color{white!15!black}\tikzfontsize}');
fid = fopen( filename, 'wt' );
idx_line_end = strfind( str, newline );
num_lines = length( idx_line_end );
for current_line_idx = 1:num_lines
    if current_line_idx == 1
        current_idx = 1;
    else
        current_idx = idx_line_end(current_line_idx-1)+1;
    end
    current_line_content = str(current_idx:idx_line_end(current_line_idx));
    if length(current_line_content)>=5
        if strcmp('\node',current_line_content(1:5))
            new_line_content = strrep(current_line_content,'font=','font=\tikzfontsize');
            str = [str(1:current_idx-1),new_line_content,str(idx_line_end(current_line_idx)+1:end)];
            idx_line_end(current_line_idx:end) = idx_line_end(current_line_idx:end) + 13;
        end
    end
end
fprintf(fid,'%s\n',str);
fclose(fid);
end