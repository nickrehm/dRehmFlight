function [] = plotQuadData(fname, axis, gains)
    
    if axis == 1
        axisText = 'Roll';
    elseif axis == 2
        axisText = 'Pitch';
    else
        error("Invalid axis: Must be 1 or 2")
    end

    kp = gains(1);
    ki = gains(2);
    kd = gains(3);

    pltTitle = [axisText, ':', ' Kp = ', num2str(kp), ...
                ' Ki = ', num2str(ki), ...
                ' Kd = ', num2str(kd)];

    dataSet = importdata(fname);
    outputAngle = dataSet(:, axis);
    commandAngle = dataSet(:, axis + 4);
    
    figure();
    plot(outputAngle, 'b', LineWidth=1);
    hold on
    plot(commandAngle, 'r', LineWidth=1);
    hold off
    ylabel([axisText, ' Angle (deg)'])
    title(pltTitle)
    ylim([-20, 20])
    legend("Output", "Command");
end