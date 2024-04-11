clear all;

addPath();

[aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'pchfilename', 'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true );

%%

figure

wingPlotGeometry(aircraft.wing_main,'CntrlPts','off')
hold on
wingPlotGeometry(aircraft.wing_vtp,'CntrlPts','off')
hold on
wingPlotGeometry(aircraft.wing_htp,'CntrlPts','off')
hold on
fuselagePlotGeometry(aircraft.fuselage)

axis off

filename = exportFilename('fdm_aerodynamics');
plot2Pdf( filename, 21, 12 )

%%
figure
structurePlot(structure,'MassColor',[0.8,0.2,0.2],'StructureColor',[0,0,0],'MassSize',0.5,'LineWidth',0.33,'NodeSize',0.5);
grid off
axis off

filename = exportFilename('fdm_structure');
plot2Pdf( filename, 21, 12 )

%%
figure
t=-1:0.1:2*pi+1;
y=1-cos(t);
y(t<0)=0;
y(t>2*pi)=0;
plot(t,y,'k-')

axis off

filename = exportFilename('fdm_cos_gust');
plot2Pdf( filename, 21, 12 )