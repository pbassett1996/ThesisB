%Name: Peter Bassett
%Subject: MMAN4020
%Function: The following program uses data attained from the Simulink model
%of an octocopter to illustrate the performance of an Extended Kalman
%Filter (EKF) for localisation.


function EKF_Sim(file1, file2)
    clf; close all;
    
    %Global variables for GUI interrupts
    global ABCD;           
    ABCD.flagPause=0;
    ABCD.end = 0;
    ABCD.Shadow = 1;
    
    if ~exist('file','var'), file1 ='IMU.mat'; end;
    load(file1);
    
    if ~exist('file','var'), file2 ='GPS.mat'; end;
    load(file2);

    %Figure for plotting vehicle movements in real-time
    figure(1)
    hold on
    
    %Trail line plots for vehicle
    mh.h2(1) = plot(0, 0,'r.'); %EKF
    mh.h2(2) = plot(0, 0,'b.'); %GPS
    mh.h2(3) = plot(0, 0,'.','Color',1/255*[0,104,87]); %IMU - DR
    
    %Waypoint plots
    mh.h1(1) = plot(0,0, 'o','MarkerFaceColor', 'k');
    mh.h1(2) = plot(0,10, 'o','MarkerFaceColor', 'k');
    mh.h1(3) = plot(10,10, 'o','MarkerFaceColor', 'k');
    mh.h1(4) = plot(10,0, 'o','MarkerFaceColor', 'k');
    
    %Real-time illustration of vehicle movement
    offset = (25*1/72)*(2/3); %Size of the arrow
    mh.h3(1) = quiver(0,offset,0,0,'r' ,'MaxHeadSize', 1,'ShowArrowHead', 'on', 'Autoscale','on','AutoscaleFactor',1.5);  
    mh.h3(2) = quiver(0,offset,0,0,'b' ,'MaxHeadSize', 1,'ShowArrowHead', 'on', 'Autoscale','on','AutoscaleFactor',1.5);
    mh.h3(3) = quiver(0,offset,0,0,'Color',1/255*[0,104,87] ,'MaxHeadSize', 1,'ShowArrowHead', 'on', 'Autoscale','on','AutoscaleFactor',1.5);
    
    xlabel('X (m)');
    ylabel('Y (m)');
    axis([-5 15 -5 15]);
    grid on; zoom on;
    title('EKF simulation trace results');
    legend('Kalman Filter Estimate', 'Actual', 'Dead Reckoning')
    
    uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
    uicontrol('Style','pushbutton','String','Next','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});
    uicontrol('Style','pushbutton','String','GPS Shadow','Position',[170,1,80,20],'Callback',{@MyCallBackA,3});
    
    %Covariance of sensors 
    stdDevAx = 0.5; %(m/s^2)
    stdevGPS = 0.25; % (m)
   
    %IMU data from Simulink model
    time = IMU.Time;
    idx = find(time>0);
    time = time(idx);
    Data_IMU = IMU.Data/2;
    ax = Data_IMU(idx,1);
    ay = Data_IMU(idx,2);
    az = Data_IMU(idx,3);
    
    %GPS data from simulink model
    Data_GPS = GPS.Data;
    X_GPS = Data_GPS(idx,1)+rand(length(time),1)*stdevGPS*2;
    Y_GPS = Data_GPS(idx,2)+rand(length(time),1)*stdevGPS*2;
    
    %EKF and DR state vectors
    states = zeros(length(ax)-1,2);
    states_IMU = zeros(length(ax)-1,2);
    states_GPS = zeros(length(ax)-1,2);
    X = 0;
    Y = 0;
    X_IMU = 0;
    Y_IMU = 0;
    vx = 0;
    vy = 0;
    vx_IMU = 0;
    vy_IMU = 0;
    heading = pi/2;

    %Noise covariance matrix
    Q = [0.00039^2, 0.00039^2];
    
    %Initialisation of process model covariance
    P1 = zeros(2,2);
    P2 = zeros(2,2);
            
    j = 0;
    k = 0;
    for i = 1:length(ax)-1
        if (ABCD.flagPause), pause(0.2) ; continue ; end;
        if (ABCD.end), close all; break; end;
        dt = time(i+1)-time(i);
        
        %EKF PREDICTION
        %Obtain expected value of Prior PDF
        [X,Y, vx, vy, heading] = process_model(ax(i+1), ay(i+1), dt, X, Y, vx, vy, heading);
        [X_IMU,Y_IMU, vx_IMU, vy_IMU, heading_IMU] = process_model(ax(i+1), ay(i+1), dt, X_IMU, Y_IMU, vx_IMU, vy_IMU, heading);
        
        %Obtain noise covariance of Prior PDF
        Fu = [dt, dt];
        Pu = [stdDevAx^2;stdDevAx^2];
        J = [1,0;0,1];
        P1 = J*P1*J'+ Q+Fu*Pu*Fu';
        P2 = J*P2*J'+ Q+Fu*Pu*Fu';

       if(ABCD.Shadow) %Toggle GPS shadowing event
           
           %EKF UPDATE
           %Difference between observation and liklihood function of
           %expected value
            H = [1,0;0,1];
            z1 = [X_GPS(i+1)-X,0;0, Y_GPS(i+1)-Y];
            v_x_gps = (X_GPS(i+1)-X_GPS(i))/(dt);
            v_y_gps = (Y_GPS(i+1)-Y_GPS(i))/(dt);
            z2 = [v_x_gps - vx,0;0, v_y_gps-vy];

            %Although the noise values for the IMU and GPS information are known
            %from the model, larger covariance matrices are used for more
            %realistic performance
            R = [stdevGPS*stdevGPS*4,0;0,stdevGPS*stdevGPS*4];
            
            %Some intermediate calculations
            S1 =R+H*P1*H';
            S2 = R+H*P2*H';

            is1 = inv(S1);
            is2 = inv(S2);
            
            %Kalman Gain
            K1 = P1*H'*is1;
            K2 = P2*H'*is2;

            kz1 = K1*z1;
            kz2 = K2*z2;
            
            %Update of horizontal positions
            X = X +kz1(1);
            Y = Y +kz1(4);
            
            %Update of horizontal velocities
            vx = vx+kz2(1);
            vy = vy+kz2(4);
            
            %Update of observation covariance
            P1 = P1-K1*H*P1;
            P2 = P2-K2*H*P2;
       end
       
       %Plotting
            if(i > 600 && k > 5) %Movement doesn't begin until 6s (600ms)
                %Update trail lines
                if(j == 5)
                    states(i,1) = X;
                    states(i,2) = Y;
                    states_IMU(i,1) = X_IMU;
                    states_IMU(i,2) = Y_IMU;
                    states_GPS(i,1) = Data_GPS(i,1);
                    states_GPS(i,2) = Data_GPS(i,2);
                
                    j= 0;
                end
                %Plot data
                set(mh.h3(1), 'xdata', X, 'ydata', Y, 'udata', 0.5*cos(heading), 'vdata', 0.5*sin(heading));
                set(mh.h3(2), 'xdata', Data_GPS(i,1), 'ydata', Data_GPS(i,2),'udata', 0.5*cos(heading), 'vdata', 0.5*sin(heading));
                set(mh.h3(3), 'xdata', X_IMU, 'ydata', Y_IMU, 'udata', 0.5*cos(heading_IMU), 'vdata', 0.5*sin(heading_IMU));
                set(mh.h2(1), 'xdata', states(:,1), 'ydata', states(:,2));
                set(mh.h2(2), 'xdata', states_GPS(:,1), 'ydata', states_GPS(:,2));
                set(mh.h2(3), 'xdata', states_IMU(:,1), 'ydata', states_IMU(:,2));
   
                pause(0.05);
                j = j+1;
                k = 0;
            end
            k = k + 1;

    end
    obstacle_avoidance() %Begin obstacle avoidance simulation
    
    %For error comparison between the different localisation methods     
%     figure(2); clf;
% 
%     subplot(411);plot(time(1:end-1), states(:,1)-Data_GPS(1:(end-2),1)); ylabel('x_K_F error'); xlim([0,70]); grid on;
%     title('Error performance of EKF vs DR');
%     subplot(412);plot(time(1:end-1), states(:,2)-Data_GPS(1:(end-2),2)); ylabel('y_K_F error'); xlim([0,70]); grid on;
%     subplot(413);plot(time(1:end-1), states_IMU(:,1)-Data_GPS(1:(end-2),1)); ylabel('x_D_R error'); xlim([0,70]);grid on;
%     subplot(414);plot(time(1:end-1), states_IMU(:,2)-Data_GPS(1:(end-2),2)); ylabel('y_D_R error'); xlabel('time (s)'); xlim([0,70]);grid on;
    
end

%% Process model - used to determine the predicted expected value of the
%horizontal position and velocities at current time.
function [X_,Y_, vx, vy, heading_]= process_model(acc_x, acc_y ,dt, X, Y, vel_x, vel_y, heading)
     vx = vel_x+acc_x*dt;
     vy = vel_y+acc_y*dt;
     X_ = X + vx*dt;
     Y_ = Y + vy*dt;
     angle = -atan2(-(Y_-Y),(X_-X));
     if(angle < 0)
         angle = angle+2*pi;
     end
     anglediff = wrapToPi(heading-angle);
     heading_ = wrapToPi(heading-anglediff*dt);
end

%% Function used for GUI button interrupts
function MyCallBackA(~,~,x)   
    global ABCD;
        
    if (x==1)
       ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
       return;
    end;
    if (x==2)
        
        ABCD.end = ~ABCD.end;
        return;
    end;
     if (x==3)
        
        ABCD.Shadow = ~ABCD.Shadow;
        sprintf("GPS Shadowing")
        return;
    end;
    return;    
end