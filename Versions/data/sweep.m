dataSet = importdata("quad_145554.log");
idx_min = 110;
idx_max = 5828;

outputAngle = dataSet(idx_min:idx_max, 1);
commandAngle = dataSet(idx_min:idx_max, 5);

outputFFT = fft(outputAngle);
commandFFT = fft(commandAngle);



L = idx_max-idx_min + 1;
sampleFrequency = L/60;

time = (idx_min:idx_max)/sampleFrequency;

P2_command = abs(commandFFT/L);
P1_command = P2_command(1:L/2+1);
P1_command(2:end-1) = 2*P1_command(2:end-1);

P2_output = abs(outputFFT/L);
P1_output = P2_output(1:L/2+1);
P1_output(2:end-1) = 2*P1_output(2:end-1);

f = sampleFrequency*(0:(L/2))/L;

% figure(1);
% plot(time, outputAngle, 'b', LineWidth=1);
% hold on
% plot(time, commandAngle, 'r', LineWidth=1);
% hold off
% ylabel('Roll Angle (deg)')
% ylim([-11, 11])
% legend("Output", "Command");

resp = outputFFT./commandFFT;

figure;
half = floor(L/2);
semilogx(f, 20*log10(abs(P1_output./P1_command)));
xlim([0, 2])
