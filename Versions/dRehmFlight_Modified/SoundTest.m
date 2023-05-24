deviceReader = audioDeviceReader;
setup(deviceReader);
fileWriter = dsp.AudioFileWriter('controller.wav', FileFormat='WAV');
disp("SPEAK!");

tic
while toc < 10
    acquiredAudio = deviceReader();
    fileWriter(acquiredAudio);
end
disp('Recording complete.')

release(deviceReader)
release(fileWriter)

