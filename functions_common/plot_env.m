function plot_env(lenKu)
D = 2.25; % ø‚Œª…Ó∂»
plot([-7,7],[5,5],'k','linewidth',2);
xlim([-7 7])
ylim([-4 6])
grid on;hold on;
plot([-lenKu,-lenKu],[-D,0],'k','linewidth',2);
plot([-7,-lenKu],[0,0],'k','linewidth',2);
plot([-7,7],[-D,-D],'k','linewidth',2);
plot([0,0],[-D,0],'k','linewidth',2);
plot([0,7],[0,0],'k','linewidth',2);
end