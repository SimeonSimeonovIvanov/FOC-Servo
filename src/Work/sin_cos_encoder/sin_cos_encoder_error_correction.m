theta = 0:1:360*4;
max = 1.57;

va = zeros(size(theta));
vb = zeros(size(theta));
angle = zeros(size(theta));
sector = zeros(size(theta));
angle360 = zeros(size(theta));

sin_err = zeros(size(theta));
cos_err = zeros(size(theta));

N = length(theta);

for i=1:N
    va(i) = sin( i * ( pi/180) );
    vb(i) = cos( i * ( pi/180) );
    
    %sin_err(i) = 0.2 * va(i);
    %cos_err(i) = 0.2 * vb(i);
    
    sin_err(i) = 0.1 * sin( ( i + 10 ) * ( pi/180) );
    cos_err(i) = 0.1 * cos( ( i + 10 ) * ( pi/180) );
    
    va(i) = va(i) - cos_err(i);
    vb(i) = vb(i) - sin_err(i);
end

for i=1:N
    va(i) = va(i) + cos_err(i);
    vb(i) = vb(i) + sin_err(i);

    angle(i) = atan( va(i) / vb(i) );

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