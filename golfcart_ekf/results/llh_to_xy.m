function [ x, y ] = llh_to_xy(lon, lat, alt)

    lon_home = -96.3719188820;
    lat_home =  30.6738726717;
    
    a = 6378137.0000;
    b = 6356752.3142;

    x = zeros(length(lon), 1);
    y = zeros(length(lat), 1);
    
    for ii = 1:length(lon)

        R = sqrt((  power( power(a,2) * cos(lat(ii)*pi/180), 2)+ ...
                    power( power(b,2) * sin(lat(ii)*pi/180), 2))/ ...
                   (power( a * cos(lat(ii)*pi/180), 2)+ ...
                    power( b * sin(lat(ii)*pi/180), 2))) + alt(ii);

        dlon = lon(ii) - lon_home;
        dlat = lat(ii) - lat_home;
        x(ii) = R * deg2rad(dlon);
        y(ii) = R * deg2rad(dlat);
    end
    
end

