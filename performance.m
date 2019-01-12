





iref=

imeas=

time = 

pwm = 

n = length(iref);

energy = 0;
plotEnergy = zeros(1, n-1);

for i=2:1:n
    energy = energy + pwm(i-1)*0.01;  %*(time(i)-time(i-1))
    plotEnergy(i-1) = energy;
end
disp(energy);

Cerror = 0;
plotCerror = zeros(1, n);

for i=1:1:n
    Cerror= Cerror+max(iref(i)-imeas(i), 0);
    plotCerror(i) = (1/i)  * Cerror;
end
disp(plotCerror(n));



cflicker = 0;
plotFlicker = zeros(1, n-2);

for i=3:1:n
    l1 = imeas(i);
    l2 = imeas(i-1);
    l3 = imeas(i-2);
    
    if (l1-l2)*(l2-l3) < 0
        fi = (abs(l1-l2)+abs(l2-l3))/(2*0.01);
    else
        fi = 0;
    end
    
    cflicker = (1/i)*fi;
    plotFlicker(i-2)= cflicker;
    
end

t = 1:0.01:n;
figure(1);
plot(t(1:n-1),plotEnergy);

figure(2);
plot(t, plotCerror);

figure(3);
plot(t(1:n-2), plotFlicker);



