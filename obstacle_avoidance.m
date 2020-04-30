%Author: Peter Bassett
%Subject: MMAN4020
%Program: Simulation for a obstacle detection aglorithm. 
%The laser data is clusterd using a hierarchal clustering approach and
%converted to the cartesian plane. The vehicle's navigate through obstacles
%using a potential field alogorithm. The laser data and some aspects of the
%code can be credited to Jose Guivant who provided it during the MTRN4010
%course.

%%

function obstacle_avoidance(file)

clc; close all;

%Global variable used for the UI button controls
global ABCD;           
ABCD.flagPause=0;
ABCD.end = 0;

%Definition of the goal position
global GoalPos;
GoalPos.X = 4; %(m)
GoalPos.Y = 7.5; %(m)

%Load Data
if ~exist('file','var'), file ='Laser__2.mat'; end;
load(file); 

%Figure 1 - Show laser points
figure(1) ;
MyGUIHandles.handle1 = plot(0,0,'b.');
hold on
MyGUIHandles.handle12 = plot(0,0,'r.');
axis([0,180,0,20]);                        
xlabel('angle (degrees)');
ylabel('range (meters)');
MyGUIHandles.handle2 = title('');           
zoom on ;  grid on;

%Figure 2 - Showing moving UAVs
%Three different UAVs provided with differing buffer values
figure(2) ;
hold on
MyGUIHandles.handle11(1) = plot(0, 0, 'b.');
MyGUIHandles.handle11(2) = plot(0, 0, 'g.');
MyGUIHandles.handle11(3) = plot(0, 0, 'r.');
MyGUIHandles.handle9 = plot(GoalPos.X,GoalPos.Y,'o', 'MarkerSize', 10); %To display processing time
MyGUIHandles.handle10 = plot(0,0,'o'); %OOI at t = 0
MyGUIHandles.handle3 = plot(0,0,'b.');      % to be used for showing the laser points
MyGUIHandles.handle4 = plot(0,0,'r.');      %to show reflective points
MyGUIHandles.handle6 = plot(0,0,'+', 'MarkerSize', 5); %to show OOIS

offset = (25*1/72)*(2/3); %Size of the arrow
MyGUIHandles.handle8(1) = quiver(0,offset,0,0,'b' ,'MaxHeadSize', 1,'ShowArrowHead', 'on', 'Autoscale','on','AutoscaleFactor',1.5); %Arrow illustrating the vehicle's movements  
MyGUIHandles.handle8(2) = quiver(0,offset,0,0,'g' ,'MaxHeadSize', 1,'ShowArrowHead', 'on', 'Autoscale','on','AutoscaleFactor',1.5);
MyGUIHandles.handle8(3) = quiver(0,offset,0,0,'r' ,'MaxHeadSize', 1,'ShowArrowHead', 'on', 'Autoscale','on','AutoscaleFactor',1.5);

axis([-5,6,0,10]);
xticks([-5:1:6]);
xlabel('X (m)');
ylabel('Y (m)');
title('Potential Field Obstacle Avoidance');
legend('Buffer = 10cm', 'Buffer = 20cm', 'Buffer = 30cm', 'Goal','Obstacles', 'Location', 'northwest')
zoom on ;  grid on;

uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});

%Transform laser data
angles = dataL.angles*180/pi ;
MaskLow13Bits = uint16(2^13-1);    
MaskHigh3Bits = bitshift(uint16(7),13);
i = 1;

%Data structure for vehicle states
for k=1:3
    Vehicle(k).X = 0;
    Vehicle(k).Y = 0;
    Vehicle(k).heading = pi/2;
    Vehicle(k).maxspeed = 0.2; %m/s
    Vehicle(k).Xarray = [];
    Vehicle(k).Yarray = [];
    Vehicle(k).count = 0;
end

buffer = [1, 2 ,3]; %Buffer definition [10cm, 20cm, 30cm]

while(~ABCD.end)
     %Check if any buttons pressed
    if (ABCD.flagPause), pause(0.2) ; continue ; end;
    if (ABCD.end), close all; return; end;
    
    %Masks used to extract range and intensity from dataL
    t =  double(dataL.times(i)-dataL.times(1))/10000; %Time interval in seconds
    
    scan = dataL.Scans(:,i);%Extract Laser scans
    rangesA = bitand(scan,MaskLow13Bits) ; %Extract ranges and...
    ranges    = 0.01*double(rangesA);      %Convert them to meters

    intensities = bitand(scan,MaskHigh3Bits); %Extract intensity of data

    PolarScan(ranges,angles,intensities,t,MyGUIHandles,i);   %2D points expressed in polar

    OOIs = ExtractOOIs(ranges, intensities);    %Extraction of OOIs
    PlotOOIs(OOIs, MyGUIHandles);               %Plot of OOIs
    
    %Apply Potential Field Algorithm and Update Heading
    for j=1:3
        if(sqrt((GoalPos.X-Vehicle(j).X)^2+(GoalPos.Y-Vehicle(j).Y)^2) > 0.5)

            field = potentialfield(OOIs, buffer(j), Vehicle(j)); %Determine field properties
            
            temp = VehiclePos(Vehicle(j),0.2, Vehicle(j).maxspeed, field); %Update vehicle position
            
            %Store vehicle position for future calculations
            Vehicle(j).X = temp.X;
            Vehicle(j).Y = temp.Y;
            Vehicle(j).heading = temp.heading;
            
            %Plot current vehicle position
            set(MyGUIHandles.handle8(j), 'xdata', Vehicle(j).X, 'ydata', Vehicle(j).Y, 'udata', 0.5*cos(Vehicle(j).heading), 'vdata', 0.5*sin(Vehicle(j).heading));
            
            %Save vehicle position for trail line history
            if(Vehicle(j).count == 10)
                Vehicle(j).Xarray = [Vehicle(j).Xarray, Vehicle(j).X];
                Vehicle(j).Yarray  = [Vehicle(j).Yarray , Vehicle(j).Y];
                Vehicle(j).count  = 0;
            end
            
            %Plot trail lines
            set(MyGUIHandles.handle11(j), 'xdata', Vehicle(j).Xarray, 'ydata', Vehicle(j).Yarray );
        end
        pause(0.03) ;                   %wait for interrupts
        Vehicle(j).count = Vehicle(j).count+1;
    end

   i = i+1;

end
close all;

end

%% Potential field algorithm
%The following algorithm is an adaption of that described in Michael A.
%Goodrich's - "Potential Fields Tutorial".
%https://www.semanticscholar.org/paper/Potential-Fields-Tutorial-Goodrich/725efa1af22f41dcbecd8bd445ea82679a6eb7c6
function f =  potentialfield(obst, buffer, vh)

    f.v_angle = 0;
    f.speed = 0;
    
    for i = 1:obst.N
        
        %Relative position between vehicle and obstacles
        relX = obst.Centers(i,1) - vh.X;
        relY = obst.Centers(i,2) - vh.Y;
        
        %Distance between vehicle and obstacle
        dist = sqrt((relX.^2) +(relY.^2));
        obst_radius = obst.Sizes(i)/2;
        
        if(dist < (obst_radius + buffer))
            angle = -atan2(-relY, relX); %Bearing towards obstacles
            
            %Wrapping to pi
            if(angle < 0)
                 angle = (angle+2*pi);
            end
            
            anglediff = (vh.heading-angle); %Difference between current heading and obstacle bearing
            
            %Application of Equation (1.15) in thesis - calculation of yaw
            %rate
            if(anglediff > 0)
                if(anglediff < pi)
                    f.v_angle = f.v_angle -0.3*(obst_radius+buffer)/dist;
                else
                    f.v_angle = f.v_angle + 0.3*(obst_radius+buffer)/dist;
                end
            else
                if(anglediff > -pi)
                        f.v_angle = f.v_angle + 0.3*(obst_radius+buffer)/dist;
                    else
                        f.v_angle = f.v_angle -0.3*(obst_radius+buffer)/dist;
                    end
                    f.speed = 0;
            end
            f.v_angle = wrapToPi(f.v_angle);
        end
    end
end

%% Update vehicle position using kinematic model for horizontal displacement
function temp = VehiclePos(vh ,dt, speed,f)
    global GoalPos;
    
    temp.X = 0;
    temp.Y = 0;
    temp.heading = 0;
    
    relX = GoalPos.X - vh.X;
    relY = GoalPos.Y - vh.Y;
    d = sqrt(((relX).^2) + ((relY).^2));
    angle = -atan2(-relY, relX);
    
    if(angle < 0)
        angle = (angle+2*pi);
    end
    anglediff = wrapToPi(vh.heading - angle);
    
    if(d > 0.2)
        dL = dt*(speed+f.speed);
        temp.heading = wrapToPi((vh.heading-anglediff*dt)-(f.v_angle*dt));
        temp.X = vh.X+dL*cos(vh.heading);
        temp.Y = vh.Y+dL*sin(vh.heading);
    end
end

%% Function to plot polar points on figure 1
function PolarScan(ranges,angles,intensities, t,mh,i)
    ii = find(intensities~=0);   %intensity index
    
    %Plot polar points on figure 1
    set(mh.handle1,'xdata',angles,'ydata',ranges);
    set(mh.handle12,'xdata',angles(ii),'ydata',ranges(ii));
    % set title
    s= sprintf('Laser Scans: Polar Representation');
    set(mh.handle2,'string',s);
        
    return;
end

% Function to find clusters of points and extract OOI information
function r = ExtractOOIs(ranges,intensities)
    %Initialise data structure
    r.N = 0;
    r.Colors = [];
    r.Centers = [];
    r.Sizes   = [];
    
    ii = find(intensities~=0); %intensity index
    
    OOIflag = 0; %If flag = 1, cluster is an OOI
    
    angles = [0:360]'*0.5 ; %Angles in degrees
    %Polar to cartesian again
    X = cos(angles*pi/180).*ranges;
    Y = sin(angles*pi/180).*ranges;
    data = [X,Y];
    
    %Hierarchical Clustering approach 
    %Theory provided by Nielsen Frank - "Hierarchical Clustering"
    %https://www.researchgate.net/publication/314700681_Hierarchical_Clustering
    
    A = pdist(data); %Calculate the distance between pairs of objects in data set
    Z = linkage(A); %Link pairs of objects that are close together
    threshold = 1; %Cutoff threshold
    T = cluster(Z, 'Cutoff', threshold); %Group clusters together, output is an array of integers
    
    %Loop through clusters - extract information for OOIs
    for k = 1:max(T)
        cond = find(T == k);
        idx = find(ismember(cond, ii));
        if(sum(idx) > 1)
            OOIflag = 1;
        end
        %Calculate the OOI parameters
        [Centers, Sizes] = CalculateOOI(data(cond,1), data(cond,2));       
         if(Sizes >= 0.01 & Sizes <= 1)
             if(OOIflag)
                r.Centers = [r.Centers; Centers];
                r.Sizes = [r.Sizes, Sizes];
                r.Colors = [r.Colors, OOIflag];
                r.N = r.N + 1;
             end
        end

        OOIflag = 0;
    end
        
end 

%% Function to calculate the Centroid and Size of OOIs
function [Centroid, Size] = CalculateOOI(X, Y)
    
    data = [X, Y];
    Centroid = [mean(X), mean(Y)]; %Simple centroid mean
    dif = data(end,:)-data(1,:);
    Size = sqrt(sum(dif.^2,2)); %Distance between two futhest points
    

end

 %% Function plots OOI in figure 1  
function PlotOOIs(OOIs,mh)
    if OOIs.N<1, return ; end;
    
    i = 1;
    idx = [];
    %if the data is reflective + 0.05m < size < 0.2m then we have found an
    %OOI!
    while i <= OOIs.N
        if(OOIs.Colors(i) == 1)
           idx = [idx, i];
        end
        i = i+1;
    end
    %Plot OOIs as a cross in figure 2
    if (sum(idx) > 0)
        set(mh.handle6, 'xdata', OOIs.Centers(idx,1), 'ydata', OOIs.Centers(idx,2));
        set(mh.handle10, 'xdata', OOIs.Centers(idx,1), 'ydata', OOIs.Centers(idx,2));
    end
return;
end

%% Callback function for GUI buttons
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
    return;    
end


