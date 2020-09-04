function sol = NR_klann(q,n,tol,iter_max,fun)
coords = zeros(iter_max,n+1);
flag = 0;sol = [];
q = [-0.5 0.5 q(3) -1.5 1.5 pi/18 -2.6616-0.5 -1.30+0.5 3*pi/4 -2.6616-1 .6145+1 3*pi/4 -5 1.5 pi/4 -4 -0.1 pi/19 -5 -2.5 pi/4]';
for i = 1:iter_max
    [Phi,D] = fun(q);
    D = [D(:,1:2) D(:,4:21)];
    err = sqrt(Phi'*Phi);
    coords(i,:) = [err q'];
	if err<tol
       flag = 1;
       sol = coords(i,2:n+1)';
       break;
	end
    delta_q = -D\Phi;
    q = q + [delta_q(1:2)',0,delta_q(3:20)']';
end
if flag == 0
    disp('Convergence failed in Newton-Raphson');
    return;
end
end