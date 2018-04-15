max_atan = 1.5708; % Const = ??? (YES)

theta = 0:1:360*4;

va = zeros(size(theta));
vb = zeros(size(theta));
angle = zeros(size(theta));
angle360 = zeros(size(theta));
sector = zeros(size(theta));
N = length(theta);

buffer = zeros(91);
first_run = 1;

k=0.0;
i = 1;
n = 1;

for t = 1:N
    va(t) = k * sin( t / 180 * pi );
    
    % Ramp generator:
    if k < 1
        k = k + 0.0025;
    end
    
    % Create 90 deg (for this sample rate) buffer - cos:
    buffer(n) = va(t);
    if n <= 91
        n = n + 1;    
    else
        first_run = 0;
        n = 1;
    end

    if first_run == 0
        if n > 1
            vb(i) = buffer( n - 1 );
        else
            vb(i) = buffer( 91 );
        end
    
        % angle = atan( arrSin[n] / arrCos[n-91] )
        angle(i) = atan( buffer( n ) / vb( i ) );
    
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

        angle_deg = 90.0 * ( angle(i) / max_atan );

        switch sector(i) 
            case 1
                angle360(i) = angle_deg;
            case 2
                angle360(i) = 180 + angle_deg;
            case 3
                angle360(i) = 180 + angle_deg;
            case 4
                angle360(i) = 360 + angle_deg;
        end

        angle360(i) = angle360(i) / 360;
        
        i = i + 1;
    end
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