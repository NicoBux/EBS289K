function q = bycicle_model(u,q,dt,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints)
% u(1) = gamma; u(2) = velocity -> what you want to set
% q(1) = x; q(2) = y; q(3) = theta; q(4) = velocity; q(5) = gamma
% dt = Euller integration; DT = Controller integration; L = Wheel base;
% s = slip; tal_v and tal_gamma = controller delay;
% delta1 and delta2 = slip angles; constraints = max gamma and max velocity


for i=1:dt:DT-dt
    Vl = (1-s)*q(4);
    Vy = tan(delta2)*Vl; %Skid lateral velocity
    q(1) = q(1) + dt*(Vl*cos(q(3))-Vy*sin(q(3)));
    q(2) = q(2) + dt*(Vl*sin(q(3))+Vy*cos(q(3)));
    q(3) = q(3) + dt*((Vl*tan(q(5)+delta1)-Vy)/L);
    if tal_v < 0.0001
        q(4) = u(2);
    else
        q(4) = q(4) + dt*((-Vl+(1-s)*u(2))/tal_v);
    end
    if tal_gamma < 0.001
        q(5) = u(1);
    else
    q(5) = q(5) + dt*((-q(5)+u(1))/tal_gamma);
    end
end


if q(5) > constraints(1)
    q(5) = constraints(1);
end

if q(4) > constraints(2)
    q(4) = constraints(2);
end





% xp = Vl*cos(q(3))-Vy*sin(q(3));
% yp = Vl*sin(q(3))+Vy*cos(q(3));
% thetap = (Vl*tan(q(5)+delta1)-Vy)/L;
% vp = (-Vl+(1-s)*u(2))/tal_v;
% gammap = (-q(5)+u(1))/tal_gamma;



    

