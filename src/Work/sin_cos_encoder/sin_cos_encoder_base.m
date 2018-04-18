theta = 0:1:360*4;

va = zeros(size(theta));
vb = zeros(size(theta));
angle = zeros(size(theta));
angle360 = zeros(size(theta));

buffer = zeros(0);

N = length(theta);

first_run = 1;
n = 1;
m=1;
k=1;

for i=1:N
    va(i) = k*sin( i * ( pi/180) );
    
    if k < 1
        k = k + 0.001;
    end
    
    buffer(n) = va(i);
    
    if first_run == 0 && 1
        if n > 1
            vb(m) = buffer( n - 1 );
        else
            vb(m) = buffer( 90 );
        end
        
        m = m + 1;
    end
    
    if n < 90
        n = n + 1;    
    else
        first_run = 0;
        n = 1;
    end
    
    %vb(i) = 1.2*k*cos( i * ( pi/180) );
end

sector = zeros(size(theta));
max = 1.57;

for i=1:N
    angle(i) = atan(va(i)/vb(i));
    if(max<angle(i))
        %max = angle(i);
    end
        
    if angle(i)<0
        angle(i) = -angle(i);
    end
    
    if  va(i)>0 && vb(i)> 0
        sector(i) = 1;
    end
    
    if va(i)>0 && vb(i) < 0
        sector(i) = 2;
    end
    
    if va(i)<0 && vb(i) < 0
        sector(i) = 3;
    end
    
    if va(i)<0 && vb(i) >0
        sector(i) = 4;
    end
    
    angle_deg = 90 * ( angle(i) / max );
    
    switch sector(i) 
        case 1
            angle360(i) = angle_deg;
        case 2
            angle360(i) = 180 - angle_deg;
        case 3
            angle360(i) = 180 + angle_deg;
        case 4
            angle360(i) = 360 - angle_deg;
    end
    
    angle360(i) = angle360(i) / 360;
end

plot(theta, va,'Color','red');
hold on;
plot(theta, vb,'Color','green');

hold on;
plot(theta, angle,'Color','blue');
hold on;
plot(theta, sector/4,'Color','blue');

hold on;
plot(theta, angle360,'Color','black');