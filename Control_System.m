%Name: Peter Bassett
%Subject: MMAN4020
%Function: The following program uses the locomotive data attained from the
%Simulink model to illustrate the control systems step-input performance.

function Control_System(GPS)
clf; close all;

%Global variable for GUI buttons
global ABCD;           
ABCD.flagPause=1;
ABCD.Next = 0;

if ~exist('file','var'), file1 ='GPS_Step.mat'; end;
load(file1);

time = GPS.Time;
GPS = GPS.Data; %(X, Y, Z)

%Figure for plotting the altitude controller response
figure(1);
mh.h1  = plot(0,0); %Simulated data
hold on
mh.h2 = plot(0,0,'r'); %Step input
xlabel('time (s)')
ylabel('Altitude (m)');
title('Altitude Response');
axis([0, 40, 0, 12]);
grid on;
legend('Vertical Displacement', 'Input Signal', 'Location', 'southeast')
uicontrol('Style','pushbutton','String','Play/Pause','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
uicontrol('Style','pushbutton','String','Next','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});

%Figure for plotting the heading controllers response
figure(2);
mh.h3  = plot(0,0); %Horizontal X
hold on
mh.h4  = plot(0,0, 'g'); %Horizontal Y
mh.h5 = plot(0,0, 'r'); %Step input
xlabel('time (s)')
ylabel('Horizontal Displacement (m)');
title('Response of Heading Controllers');
axis([0, 40, 0, 12]);
grid on;
legend('Horrizontal X Displacment', 'Horizontal Y Disaplacement', 'Input Data', 'Location', 'southeast')


input = zeros(1,length(time))+10; %Experiment uses a step input of 10m for all experiments
i = 1;

%Iterate and plot UAV's position for time (t>0)
%Used for real-time interpretation
while(1)
    if (ABCD.flagPause), pause(0.2) ; continue ; end;
    if (ABCD.Next), close all; break; end;
    if(i<length(time))
        set(mh.h1, 'xdata', time(1:i), 'ydata', -GPS(1:i,3));
        set(mh.h2, 'xdata', time(1:i), 'ydata', input(1:i));
        set(mh.h3, 'xdata', time(1:i), 'ydata', GPS(1:i,1));
        set(mh.h4, 'xdata', time(1:i), 'ydata', GPS(1:i,2));
        set(mh.h5, 'xdata', time(1:i), 'ydata', input(1:i));
    end
    i = i +5;
    drawnow();
end

EKF_Sim() %Begin EKF simulation

end

%% Function for to operate the GUI buttons
function MyCallBackA(~,~,x)   
    global ABCD;
        
    if (x==1)
       ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
       return;
    end;
    if (x==2)
        
        ABCD.Next = ~ABCD.Next;
        return;
    end;

    return;    
end

