% Demo for 9DOF AHRS Enabled Sparkfun Sensor
% Shahriar Shahramian, Sept. 2011
% The Signal Path: http://www.TheSignalpath.com
delete(instrfindall); clear; clc; close;

MyCOM = serial('COM7', 'Baudrate', 115200); % Wired FTDI
fopen(MyCOM);
fprintf(MyCOM, 'k');

%%
i = 1;
test = 0;
instring = fscanf(MyCOM,'%s'); % Throw away first string.

time = 1:1000;
yawplot = zeros(1000,1);
pitchplot = zeros(1000,1);
rollplot = zeros(1000,1);

q0plot = yawplot; q1plot=yawplot; q2plot=yawplot; q3plot=yawplot;
figure('units','normalized','outerposition',[0 0 1 1])
graf = plot(time, q0plot, time,  q0plot, time, q0plot, time, q0plot);
axis([0 1000 900 1100]);
%legend('Stamp/x','Rull/y','Gir/z');
legend('Trykk [mbar]');
xlabel('Tid [x100 millisek]');
grid;

while (i<=1000)
    instring = fscanf(MyCOM,'%s');
    Q_STRING = strsplit(instring, ',');
    Q = zeros(1,4);
    for teller = 1:4
        Q(teller) = hex2dec(Q_STRING(teller));
        if(Q(teller)>32768)
           Q(teller) = Q(teller) - 65536; 
        end
        Q(teller) = Q(teller) / 10.0;
    end
    
    q0 = Q(1);
    q1 = Q(2);
    q2 = Q(3);
    q3 = Q(4);

% UTREGNING HENTET FRA STM32F3 EKSEMPEL FRA MCD APPLICATION TEAM ----------
    test = q1*q2+q3*q0;
    if (test > 0.499)
       yaw = 2*atan2(q1, q0);
       pitch = pi*180/(2*pi); 
       roll = 0;
    elseif (test< -0.499)
       yaw = -2*atan2(q1, q0);
       pitch = -pi*180/(2*pi);
       roll = 0;
    else
        yaw = atan2(2*q2*q0 - 2*q1*q3, 1 - 2*q2*q2 - 2*q3*q3);
        pitch = asin(2*q1*q2 + 2*q3*q0);
        roll = atan2(2*q1*q0 - 2*q2*q3, 1 - 2*q1*q1 - 2*q3*q3);
    end
    
% UTREGNING BASERT PÅ QUATERNION-NOTAT. -----------------------------------
%     roll = atan2(2*(q2*q3-q0*q1),2*q0^2 - 1 + 2*q3^2);
%     pitch = -atan(2*(q1*q3 + q0*q2)/sqrt(1 - (2*q1*q3 + 2*q0*q2)^2));
%     yaw = atan2(2*(q1*q2-q0*q3), 2*q0^2 - 1 + 2*q1^2);

%     yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0+2*q1*q1-1);
%     pitch = -asin(2*q1*q3+2*q0*q2);
%     roll = atan2(2*q2*q3-2*q0*q1, 2*q0*q0+2*q3*q3-1);

    
% UTREGNING KUN FRA AKSELEROMETER -----------------------------------------
%     ayz_abs = sqrt(q1^2 + q2^2);
%     pitch = -atan2(q0, ayz_abs);
%     if (q2 ~= 0)
%         roll = atan2(q1, q2);
%     end
%     yaw = 0;

% PLOTTING AV PITCH, ROLL, YAW ELLER Q0..Q3 -------------------------------
% ROLL, PITCH, YAW:
%     yawplot(i) = rad2deg(yaw);
%     pitchplot(i) = rad2deg(pitch);
%     rollplot(i) = rad2deg(roll);
%     set(graf(1), 'YData',pitchplot(1:i));
%     set(graf(2), 'YData',rollplot(1:i));
%     set(graf(3), 'YData',yawplot(1:i));
%     set(graf(4), 'YData',zeros(i,1));
% 
% % Q0...Q3:
    q0plot(i) = q0;
    q1plot(i) = q1;
    q2plot(i) = q2;
    q3plot(i) = q3;
    set(graf(1), 'YData',q0plot(1:i));
    set(graf(2), 'YData',q1plot(1:i));
    set(graf(3), 'YData',q2plot(1:i));
    set(graf(4), 'YData',q3plot(1:i));
% 
% % X-AKSE:
    set(graf(1), 'XData', time(1:i));
    set(graf(2), 'XData', time(1:i));
    set(graf(3), 'XData', time(1:i));
    set(graf(4), 'XData', time(1:i));

% 3D-PLOT -----------------------------------------------------------------
%    plot3D (0,0,0,pitch, roll, yaw,0.1,0, 'shuttle');
%    plot3D (0,0,0,yaw, roll, pitch,0.1,0, 'shuttle');
   drawnow
   i = i + 1;
end



%%
fprintf(MyCOM, 's');
fclose(MyCOM);