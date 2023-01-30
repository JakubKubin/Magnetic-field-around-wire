clear
I = 1.0;
u0 = 1.257e-6;
L = .3;
nL = 50;

xmin = -0.15;
xmax = -xmin;
ymin = xmin;
ymax = -xmin;
zmin = xmin;
zmax = -xmin;
dx = 0.028;

rx = linspace(-L/2,L/2,nL+1);
%ry = zeros(1,nL+1);
ry = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
rz = zeros(1,nL+1);

Points = transpose([rx;ry;rz]);

x = xmin:dx:xmax;
y = ymin:dx:ymax;
z = zmin:dx:zmax;

[X,Y,Z] = meshgrid(x,y,z);

Bx=zeros(size(X));
By=zeros(size(X));
Bz=zeros(size(X));

for it=1:nL
    rx=X-Points(it,1);
    %ry = Y;
    ry = Y-Points(it,2);
    rz = Z;

    R = sqrt(rx.^2+ry.^2+rz.^2);
    R = R.*(R>=dx)+dx*(R<dx);

    dLx = Points(it+1,1)-Points(it,1);
    dLy=0;
    dLz=0;
    
    Bx = Bx+u0*I/(4*pi)*(dLy.*rz-dLz.*ry).*R.^(-3);
    By = By+u0*I/(4*pi)*(dLz.*rx-dLx.*rz).*R.^(-3);
    Bz = Bz+u0*I/(4*pi)*(dLx.*ry-dLy.*rx).*R.^(-3);

    %clf;
    %plot3(Points(:,1),Points(:,2),Points(:,3),'-k','LineWidth',2)
    %axis([xmin xmax ymin ymax zmin zmax])
    %hold on 
    %quiver3(X,Y,Z,Bx,By,Bz,'AutoScaleFactor',0.9,'Color','r')
    %quiver3(-.25*xmax,0,.1*zmax,0.5*xmax,0,0,'LineWidth',3,'MaxHeadSize',3)
    %hold off
    %rotate3d on
    %axis square
    %xlabel('x')
    %ylabel('y')
    %zlabel('z')
    %view(-30,30)
    %title('Pole elektromagnetyczne')
    %shg; 
end

plot3(Points(:,1),Points(:,2),Points(:,3),'-k','LineWidth',2)
axis([xmin xmax ymin ymax zmin zmax])
hold on 
quiver3(X,Y,Z,Bx,By,Bz,'Color','b')
quiver3(-.25*xmax,0,.1*zmax,0.5*xmax,0,0,'LineWidth',3,'MaxHeadSize',3)
hold off
rotate3d on
axis square
xlabel('x')
ylabel('y')
zlabel('z')
view(-30,30)
title('Pole elektromagnetyczne')
