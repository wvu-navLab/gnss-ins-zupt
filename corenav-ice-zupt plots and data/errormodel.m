function err = errormodel(a, b, dt, d)

% source: Liyu, Amin ,"Tracking performance and average error analysis of GPS discriminators in multipath"  

if a < 1
    if dt >= 0 && dt < (1 + 2*a*cos(b) + a^2)*d/(2*(1+a*cos(b)))
        
        err = a*(a + cos(b))*dt/(1 + 2*a*cos(b)+ a^2);
        
    elseif dt >= (1 + 2*a*cos(b) + a^2)*d/(2*(1+a*cos(b))) && dt < ((a*d*cos(b) + 1)*(2-d) - d*(1 - d/2 - a^2*d/2))/(a*d*cos(b)+2-d)
        
        err = (((a*cos(b)*(1-dt)- a^2*d/2 + 1 - d/2)^2 + 2*a^2*d*cos(b)^2*(1 - d/2) + 2*a^3*d*cos(b)*(1- dt))^0.5 - (a*cos(b)*(1-dt)- a^2*d/2 + 1 - d/2))/(2*a*cos(b));
        
    elseif dt >=  ((a*d*cos(b) + 1)*(2-d) - d*(1 - d/2 - a^2*d/2))/(a*d*cos(b)+2-d) && dt < 1 + d/2
        
        err = ((a*cos(b)-a^2)*dt + a^2*(1+d/2) - a*d*cos(b) - 2 + d + (a^2*cos(b)^2*dt^2 + 2*a*(2-d)*(a - cos(b))*dt - 4*a^2*cos(b)^2*dt + ...
            (2-d)*(2 - d + 2*a*d*cos(b)) + 4*a^2*(d^2/4 - sin(b)^2))^0.5)/(2*a*cos(b)-a^2); 
    elseif dt > 1+ d/2
        
        err = 0;
    end
end

end