function geocLat = geocentricLatitude(lat)
f= 1/298.257223560;
t = (1 - f)^2;
geocLat= atan2(t*sin(lat), cos(lat));
end
