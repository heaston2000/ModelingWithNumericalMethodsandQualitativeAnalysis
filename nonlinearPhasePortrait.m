function nonlinearPhasePortrait(f,g,xBounds,yBounds,tSpan,FPs,IC)
% function nonlinearPortrait(f,g,xInt,yInt,tSpan,IC)
%
% This function creates a plot of the direction field and phase portrait
% for a 2x2 first-order, autonomous system of differential equations of
% form:
%
%               dx/dt = f(x,y),
%               dy/dt = g(x,y).
%
% Phase portrait trajectories are plotted using the solver ode15s. Input
% values are as follows:
%
%               f = f(x,y), given as an anonymous function handle,
%               g = g(x,y), given as an anonymous function handle,
%               xInt = [a,b] is x-interval for phase portrait,
%               yInt = [c,d] is y-interval for phase portrait,
%               tSpan = [t0,tf] is the t-interval for trajectories,
%               IC is matrix of initial values for trajectories.
%
% Define f and g using componentwise arithmetic (.*, ./, .^).  If n
% trajectories are desired, IC must be an nx2 matrix of the following form:
%
%               IC = [x0 y0; x1 y1; x2 y2; ... ; xn yn],
%
% so that the pair (xi,yi) represents the initial value (x(t0),y(t0)) for
% the ith trajectory.  The trajectories are plotted using the t-interval
% [t0-l, t0+l] where l = tf-t0 (t-interval is centered at t0).
%

n = length(IC(:,1));
t0 = tSpan(1,1);
tFinal = tSpan(1,2);
l = t0-tFinal;
tLeft = [t0-l,t0];
a = xBounds(1,1);
b = xBounds(1,2);
c = yBounds(1,1);
d = yBounds(1,2);
[X,Y] = meshgrid(linspace(a,b,20),linspace(c,d,20));
F = f(X,Y);
G = g(X,Y);
L = sqrt(F.^2+G.^2);
odefun = @(t,x) [f(x(1),x(2)); g(x(1),x(2))];

figure;
quiver(X,Y,F./L, G./L,0.5)
axis tight

hold on;
for i = 1:n
    [T,Xsoln] = ode15s(odefun,tSpan,IC(i,:));
    plot(Xsoln(:,1),Xsoln(:,2),'b', 'linewidth',2)
%     [T,Xsoln] = ode15s(odefun,tLeft,IC(i,:));
%     plot(Xsoln(:,1),Xsoln(:,2),'r','linewidth', 2)
    plot(IC(i,1),IC(i,2),'ko')
end
axis([a,b,c,d])
xlabel('x')
ylabel('y')