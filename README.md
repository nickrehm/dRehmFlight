This is a fork of dRehm flight 1.3 beta: https://github.com/nickrehm/dRehmFlight

At the moment the servo scales have been reworked on lines 1127-1133.

From s1_command_PWM = s1_command_scaled*180;

To s1_command_PWM = 90 + (s1_command_scaled*90);

To prevent half of the servo rotation from being cut, in some cases. 
