%Conical Helix
%conical spiral with angular frequency a on a cone of height h and radius r
%is a space curve given by the parametric equations
function [Px, Py, Pz] = generate_conicalspiral(K, horizontal_n)
a = 0.08  ;  % angle frequence
h = 20; %heigh
r = 10 ;% radius
verical = linspace(20, 0 , K);
horizontal = linspace(8, 0, horizontal_n);


x = [];
y = [];
z = [];
for t = horizontal
    x = [x, t];
    y = [y,0];
    z = [z, 20];
end    
c= 0;
for t = verical
    z = [z, t];
    x = [x, (h - t)/h * r * cos(a * c * 0.2) ];
    y = [y, (h - t)/h * r * sin(a * c * 0.2) ];
    c = c +1;
end

for i = 1:20
    z = [z, z(end)];
    x = [x, x(end)];
    y = [y, y(end)];
end   
Px = x;
Py = y;
Pz = z;
%plot3(x, y, z)