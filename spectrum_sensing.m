%Name: Peter Bassett
%Subject: MMAN4020
%Function: The following program simulates a controlled experiment for a
%spectrum sensing algorithm. 

%2.4 GHz band ranges from 2400MHz - 2483MHz
%4.8 GHz band ranges from 5725MHz - 5850MHz

function CR_Simulation(file1, file2, file3, file4)
clf; close all; clc;

global ABCD;           
ABCD.next = 1;

t = 0:0.000001:0.01; %Time

bandwidth = 20; %MHz
MHz = 10^6;

%Creation of carrier signals
CF = [2400:1:2450]*MHz;
CF_1 = [2600:1:2800]*MHz;
CF_2 = 6000*MHz;

Fs = 12000*MHz; %Sampling frequency
x1 = cos(2*pi*1000*t); %Input message signal

SNR  = 1;  %Signal to Noise Ratio DB
signal_power = 30; %dBW
attenuation = 10/100; %Signal Attentuation (%)

sum = 0;

%Band criteria (i.e. frequency range to use and resolution)
sample_start = 2300*MHz;
sample_range = 20*MHz;
sample_end = 3000*MHz;
sample_incr = 20*MHz;

Energy_distribution = []; %Initalistion of energy vector

y = 0;

%Amplitude modulation of the carrier signals
for(i = 1:length(CF))
    y1 = ammod(x1, CF(i), Fs);
    y = y+y1;
end
for(i = 1:length(CF_1))
    y2 = ammod(x1, CF_1(i), Fs);
    y = y+y2;
end
y3 = ammod(x1, CF_2, Fs);

%Spectrum sensing algorithm
%Iterate through unlicensed band and calculate signal energies
while(sample_start <= sample_end)
    for j =1:100
        energy = 0;
       
        temp_y = awgn(y, SNR, signal_power); %Add Gaussian White Noise (for realistic simulatioN)
        temp_y = temp_y.*(1-attenuation); %Add signal attenuation

        L = length(temp_y);
        
        %Fast fourier transformation
        X = fft(temp_y);
        P2 = abs(X/L);
        P1 = P2(1:round((L)/2+1));
        f = Fs*(0:(L/2))/L;

        Y2 = X(f>=sample_start & f<=sample_start+sample_range); %selected sample range for CED
        
        %Classical energy detection (CED) algorithm
        for(k=1:length(Y2))
            energy = energy+abs(Y2(k).^2);
        end

        sum = (sum + energy);

    end

    mean = sum/100;
    Energy_distribution = [Energy_distribution, mean];
    sample_start = sample_start+sample_incr;
    sum = 0;

end


Freq = [2300*MHz:sample_incr:sample_end];
% P = periodogram(y);
% Hpsd = dspdata.psd(P,'Fs',Fs);
L = length(temp_y);
P2 = abs(X/L);
P1 = P2(1:(L)/2+1);
f = Fs*(0:(L/2))/L;
f1 = find(f>2300*MHz & f < 3000*MHz);

idx = find(Freq > 2400*MHz & Freq < 2480*MHz);
band = min(Energy_distribution(idx));
idx = find(Energy_distribution == band);
sprintf("Ideal frequency range [%f - %f] MHz", Freq(idx)/(MHz), (Freq(idx))/(MHz)+20)

figure(1)
plot(f(f1)/(10^9),P1(f1)*100)
xlabel('Frequency (MHz)');
ylabel('Amplitude (Dbm)');
title('Frequency vs Amplitude of 2300-3000MHz range');
uicontrol('Style','pushbutton','String','Next.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});

figure(2)
plot(Freq/(10^9), Energy_distribution);
xlabel('Frequency (MHz)');
ylabel('Energy');
title('Energy Distribution of 2300-3000MHz range');
axis([2.3 3 0 3.5*(10^8)]);

figure(3)
plot(t*1000, temp_y)
xlabel('time (ms)')
ylabel('Amplitude (dB)');
title('Amplitude Modulated Signals');
axis([0 10 -100 150]);

while(ABCD.next)
    pause(0.05);
end

close all; clf;

Control_System()

end

function MyCallBackA(~,~,x)   
    global ABCD;
        
    if (x==1)
       ABCD.next = ~ABCD.next;
       return;
    end;

end

