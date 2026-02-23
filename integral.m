function inter = integral(I, x0, xf,npoints)
x_coord = linspace(x0,xf,npoints);
inter = trapz(x_coord,I(x_coord));
end