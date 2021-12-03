%%% Car Model Linearization

%% Linearize Equations
syms X u Y v psi r Fx deltaf m Nw f Iz a b By Cy Dy Ey Shy Svy g

alphaf = deltaf - atan((v + a*r)/u);
alphar = -atan((v - b*r)/u);

phiyf = (1 - Ey)*(alphaf + Shy) + Ey/By*atan(By*(alphaf + Shy));
phiyr = (1 - Ey)*(alphar + Shy) + Ey/By*atan(By*(alphar + Shy));

Fzf = b / (a + b) * m*g;
Fyf = Fzf*Dy*sin(Cy*atan(By*phiyf)) + Svy;
Fzr = a / (a + b) * m*g;
Fyr = Fzr*Dy*sin(Cy*atan(By*phiyr)) + Svy;


zdot = [u*cos(psi) - v*sin(psi);
        1/m*(-f*m*g + Nw*Fx - Fyf*sin(deltaf)) + v*r;
        u*sin(psi) + v*cos(psi);
        1/m*(Fyf*cos(deltaf) + Fyr) - u*r;
        r;
        1/Iz*(a*Fzf*cos(deltaf) - b*Fyr)];

z = [X; u; Y; v; psi; r];
u = [deltaf; Fx];

A = jacobian(zdot,z);
B = jacobian(zdot,u);

