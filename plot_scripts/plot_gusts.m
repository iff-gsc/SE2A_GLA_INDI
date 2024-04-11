
% add folders to path
addPath();

is_tikz_export_desired = false;

gust_grad_dist_vec  = [30,45,67.5,101.25,151.875,227.8125,350];
gust_grad_dist_vec = linspace(30,350,6);

F_g = 1;

h = 6000;

U_ref = oneMinusCosGustUref(h);

figure

hold on

for i = 1:length(gust_grad_dist_vec)
    
    gust_grad_dist = gust_grad_dist_vec(i);
    
    x = linspace(0,2*gust_grad_dist,100);
    
    U_ds = oneMinusCosGustUds(U_ref,ft2m(gust_grad_dist),F_g);
    
    U = U_ds/2 * (1-cos(2*pi*x/(2*gust_grad_dist)));
    U(x>2*gust_grad_dist) = 0;
    U(x<0) = 0;
    
    h1=plot(x,U,'k-');
    
end

x = linspace(gust_grad_dist_vec(1),gust_grad_dist_vec(end),50);
h2=plot(x,oneMinusCosGustUds(U_ref,ft2m(x),F_g),'k--');

xlim([0,max(2*gust_grad_dist_vec)])

xlabel('Gust length, ft')
ylabel('Gust velocity, m/s')

grid on
box on

legend([h1,h2],'$U$','$U_\mathrm{ds}$','interpreter','latex')

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style = {font=\tikzfontsize}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_definition.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
