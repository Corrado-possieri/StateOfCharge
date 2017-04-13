clear all
close all
clc

% sampling time
Fs = 100;             
T = 1/Fs;

% used resistence
RR = 10;

% length of the simulation
totTime = 240;
lll = totTime*Fs;

% filters parameters
alph = pi;
alph2 = 1;

% HG-observers param
order1 = 5; 
outorder1 = order1;
polyroots1 = -1*ones(1,order1+2);
polycoeff1 = real(poly(polyroots1(2:end-1)));
kappa1 = polycoeff1(2:end);
epsilon1 = 4e-1;
ye01 = ones(order1,1);

order2 = order1;
outorder2 = order1;
polyroots2 = -1*ones(1,order2+2);
polycoeff2 = real(poly(polyroots2(2:end-1)));
kappa2 = polycoeff2(2:end);
epsilon2 = 4e-1;
ye02 = ones(order2,1);

% definition of the observers matrices
A1 = zeros(order1);
A1(1:end-1,2:end) = eye(order1-1);
B1 = zeros(order1,1);
for ii = 1:order1
    B1(ii) = kappa1(ii)/(epsilon1^ii);
end
A1(:,1) = -B1;
C1 = eye(outorder1,order1);

A2 = zeros(order2);
A2(1:end-1,2:end) = eye(order2-1);
B2 = zeros(order2,1);
for ii = 1:order2
    B2(ii) = kappa2(ii)/(epsilon2^ii);
end
A2(:,1) = -B2;
C2 = eye(outorder2,order2);

% initialize vectors
Ie5 = zeros(order1,lll);
Ie5(:,1) = ye01;
Ve5 = zeros(order2,lll);
Ve5(:,1) = ye01;

val = zeros(4,lll);
fM1 = zeros(1,lll);
fM2 = zeros(1,lll);
fM3 = zeros(1,lll);

hVoc = zeros(1,lll);
hVocf = zeros(1,lll);

figure(1)
xlim([0, totTime]);
ylim([0, 15]);
hold on
grid on
box on

% create serial object for Arduino
s = serial('COM3');
s.BaudRate = 115200;

% initialize arduino comunication
fopen(s);

% initialize serial comunication
connected = 0;
while connected == 0
    % read data from the seria port
    readData = fscanf(s,'%d');
    disp(readData)
    pause(1);
    % if the value read is A
    if strcmp(readData(1), 'A')
        % establish comunication
        fwrite(s,'A');
        % clean the serial buffer
        flushinput(s);
        % now we are connected
        connected = 1;
    end
end

% read the first data
val(1,1) = fscanf(s,'%d')*T;
val(2,1) = fscanf(s,'%d')/1023*5/RR;
val(3,1) = fscanf(s,'%d')/1023*15;
val(4,1) = fscanf(s,'%d')/1023*15;

% initialize the data
fm1(1) = val(1,1);
fm1(2) = val(2,1);
fm1(3) = val(3,1);

for ii = 2:lll
    % read 3 values from the serial port
    val(1,ii) = fscanf(s,'%d')*T;
    val(2,ii) = fscanf(s,'%d')/1023*5/RR;
    val(3,ii) = fscanf(s,'%d')/1023*15;
    val(4,ii) = fscanf(s,'%d')/1023*15;
    % filter the signals
    fM2(ii) = (1-alph*T)*fM2(ii-1) + alph*T*val(2,ii-1);
    fM3(ii) = (1-alph*T)*fM3(ii-1) + alph*T*val(3,ii-1);
    % estimate the derivatives through the HG-observer
    Ie5(:,ii) = Ie5(:,ii-1) ...
        + (val(1,ii)-val(1,ii-1))*(A1*Ie5(:,ii-1) + B1*fM2(ii-1));
    Ve5(:,ii) = Ve5(:,ii-1) ...
        + (val(1,ii)-val(1,ii-1))*(A2*Ve5(:,ii-1) + B2*fM3(ii-1));
    i0 = Ie5(1,ii);
    i1 = Ie5(2,ii);
    i2 = Ie5(3,ii);
    i3 = Ie5(4,ii);
    i4 = Ie5(5,ii);

    yy0 = Ve5(1,ii);
    yy1 = Ve5(2,ii);
    yy2 = Ve5(3,ii);
    yy3 = Ve5(4,ii);
    yy4 = Ve5(5,ii);
    
    % compute the Voc
    hhx2 = (i3.^2.*yy1+(-1).*i2.*i4.*yy1+(-1).*i2.*i3.*yy2+i1.*i4.*yy2+i2.^2.* ...
      yy3+(-1).*i1.*i3.*yy3).^(-1).*((-1).*i3.^2.*yy2+i2.*i4.*yy2+i2.*i3.* ...
      yy3+(-1).*i1.*i4.*yy3+(-1).*i2.^2.*yy4+i1.*i3.*yy4);


    hhx3 = (i3.^2.*yy1+(-1).*i2.*i4.*yy1+(-1).*i2.*i3.*yy2+i1.*i4.*yy2+i2.^2.*yy3+ ...
      (-1).*i1.*i3.*yy3).^(-1).*(i3.^2.*yy1.^2+(-1).*i2.*i4.*yy1.^2+(-1).* ...
      i3.^2.*yy0.*yy2+i2.*i4.*yy0.*yy2+(-1).*i2.*i3.*yy1.*yy2+i1.*i4.*yy1.*yy2+ ...
      i1.*i3.*yy2.^2+(-1).*i0.*i4.*yy2.^2+i2.*i3.*yy0.*yy3+(-1).*i1.*i4.* ...
      yy0.*yy3+i2.^2.*yy1.*yy3+(-2).*i1.*i3.*yy1.*yy3+i0.*i4.*yy1.*yy3+(-1).* ...
      i1.*i2.*yy2.*yy3+i0.*i3.*yy2.*yy3+i1.^2.*yy3.^2+(-1).*i0.*i2.*yy3.^2+( ...
      -1).*i2.^2.*yy0.*yy4+i1.*i3.*yy0.*yy4+i1.*i2.*yy1.*yy4+(-1).*i0.*i3.* ...
      yy1.*yy4+(-1).*i1.^2.*yy2.*yy4+i0.*i2.*yy2.*yy4);
  
  hVoc(ii) = hhx3./hhx2;
  
  % filter the signals
  if isnan(hVoc(ii-1)) == 0
    hVocf(ii) = max((1-alph2*T)*hVocf(ii-1) + alph2*T*hVoc(ii-1),0);
  else 
    hVocf(ii) = max(hVocf(ii-1),0);
  end
  
  if ii > 2
     delete(h); 
     delete(h2);
  end
  
  % plot the graph
  % h = plot(val(1,2:ii),hVoc(2:ii),'r');
  hold on
  h = plot(val(1,2:ii),val(4,2:ii),'b--');
  h2 = plot(val(1,2:ii),hVocf(2:ii),'r');
  
  drawnow
end

% initialize the data
name = strcat('data_',datestr(now,'dd_mmm_yyyy_HH_MM_SS'),'_100Hz.csv');
% write data on a csv
dlmwrite(name,val,'delimiter',',');

fclose(s);
