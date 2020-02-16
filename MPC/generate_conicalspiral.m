%Conical Helix
%conical spiral with angular frequency a on a cone of height h and radius r
%is a space curve given by the parametric equations
function [Px, Py, Pz] = generate_conicalspiral(K)
a = 0.08  ;  % angle frequence
h = 20; %heigh
r = 10 ;% radius
%H = 10 : 0.5 : 0
verical = linspace(20, 0 , K);
horizontal = linspace(8, 0, 10);


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
Px = x;
Py = y;
Pz = z;
%plot3(x, y, z)