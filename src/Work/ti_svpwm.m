% http://www.cnblogs.com/nixianmin/p/4791428.html
theta = 0:1:360;
vd = 0.0;
vq = 1.0;
vmax = 0;
vmin = 0;

Valpha =  zeros(size(theta)); 
Vbeta =  zeros(size(theta));

va = zeros(size(theta));
vb = zeros(size(theta));
vc = zeros(size(theta));

vcom = zeros(size(theta));

Vx = zeros(size(theta));
Vy = zeros(size(theta));
Vz = zeros(size(theta));

N = length(theta);

for i=1:N
    Valpha(i) = vd*cos(i/180*pi)-vq*sin(i/180*pi);
    Vbeta(i) = vq*cos(i/180*pi)+vd*sin(i/180*pi);
    
    va(i) = Valpha(i);
    vb(i) = -0.5*Valpha(i) +sqrt(3)/2*Vbeta(i);
    vc(i) = -0.5*Valpha(i) -sqrt(3)/2*Vbeta(i);
    
    if (va(i) > vb(i))
        vmax = va(i);
        vmin = vb(i);
    else
        vmax = vb(i);
        vmin = va(i);
    end
    
    if(vc(i) > vmax)
        vmax = vc(i);
    elseif (vc(i) < vmin)
        vmin = vc(i);
    end

    vcom(i) = ( vmax + vmin ) / 2;
    Vx(i) = vcom(i) - va(i);
    Vy(i) = vcom(i) - vb(i);
    Vz(i) = vcom(i) - vc(i);
        
end

%plot(theta, Valpha);
hold on
%plot(theta, Vbeta);
hold on

plot(theta, va,'Color','red');
hold on
plot(theta, vb,'Color','yellow');
hold on
plot(theta, vc,'Color','green');
hold on

plot(theta, Vx,'Color','red');
hold on
%plot(theta, Vy,'Color','yellow');
%hold on
%plot(theta, Vz,'Color','green');
%hold on

plot(theta, com,'Color','blue');