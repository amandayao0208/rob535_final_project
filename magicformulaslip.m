clear all;
close all;

%this script will loop through some values for road friction (mu)
mus=[0.25,0.5,0.9];

for mu=mus
m = 1400; 
f = 0.01;
Iz = 2420; 
bw = 0; 
fw = 0;
Nw = 2; %number of driven wheels
rw = 0.31;
J = 2.65;
a = 1.14;
L = 2.54;
b = L - a;


% we make some assumptions about how the varying the road friction affects
% the magic formula. Neither of these assumptions are exact, but the
% behavior is close to what we see, and it allows us to simulate
% different road surfaces without wiriting out a full set of parameters for
% each mu

%the peak of our tire force is scaled proportionally with road friction
Dx = mu;
Dy = mu;

%we assume the stiffness (where we get the nice linear behavior at low slip) is
%is not affected by the reduction in friction.
if mu==0
    Bx=0;
    By=0;
else
Bx = 25*1/mu;
By = 0.27*1/mu;
end

Cx = 1.35;
Shx  = 0;
Svx = 0;
Ex = -2.9;

Cy = 1.2;
Ey = -1.6;
Shy = 0;
Svy = 0;

g=9.86055;

%normal forces with steady acceleration state assumption
%normal force for 2 front wheels
Fzf = m*g*b/L;
%normal force fr 2 real wheels
Fzr = m*g*a/L;
%normal force per rear wheel
Fz=Fzr/2;

for i = 1:101
    lambda( i ) = ( i - 1) * 0.002;
    phix( i ) = ( 1 - Ex ) * ( lambda( i ) + Shx ) + Ex/Bx * atan( Bx * ( lambda( i ) + Shx ) );
    Fx( i ) = Fz*Dx * sin( Cx * atan( Bx * phix( i ) ) ) + Svx;
end

for i = 1:151
    alphaf( i ) = ( i - 1 ) * 0.1; %% in degrees
    phiyf( i ) = ( 1 - Ey ) * ( alphaf( i ) + Shy ) + Ey/By * atan( By * ( alphaf( i ) + Shy ) );
    Fyf( i ) = Fzf*Dy * sin( Cy * atan( By * phiyf( i ) ) ) + Svy;
end


figure(1); plot( lambda, Fx, '-r' );
hold on


figure(2); plot( alphaf, Fyf, '-r' );
hold on

end

figure(1)
plot(lambda,Fz*ones(size(lambda)),'k--')
xlabel( 'Longitudinal Slip','FontSize', 18 );
ylabel( 'Longitudinal Force Fx (N) from 1 rear wheel ','FontSize', 18 );

figure(2)
plot(alphaf,Fzf*ones(size(alphaf)),'k--')
xlabel( 'Tire Slip Angle (deg)','FontSize', 18 );
ylabel( 'Front Lateral Force Fx (N) from 2 front wheels','FontSize', 18 );


