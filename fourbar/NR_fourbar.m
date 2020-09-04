function sol = NR_fourbar(q,n,tol,iter_max,fun)
coords = zeros(iter_max,n+1);
flag = 0;sol = [];
q=[1 1 q(3) 2 2 0 5 5 pi/2]'; % Guess value
for i = 1:iter_max
    [Phi,D] = fun(q);
    D = [D(:,1:2) D(:,4:9)];
    err = sqrt(Phi'*Phi);
    coords(i,:) = [err q'];
    if err<tol
       flag = 1;
       sol = coords(i,2:n+1)';
       break;
	end
    delta_q = -D\Phi;
    delta_q = [delta_q(1:2,1)' 0 delta_q(3:8,1)']';
    q = q + delta_q;
end
if flag == 0
    disp('Convergence failed in Newton-Raphson');
    return;
end
end