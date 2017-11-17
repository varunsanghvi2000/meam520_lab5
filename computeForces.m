% Check for collisions with objects in the environment and compute the
% total force on the end effector

function F = computeForces()
F = [0;0;0];

%spring constant
k=0.5;
button_stiffness=0.9;
c=0.2;
global posEE;
global large_button_1
global floor1_flat
global floor2_texturesurface1
global floor3_viscussurface
global floor4_texturesurface2
global floor5_flat
%global floor6_attractionfield
global floor7_flat
global floor8_gravity_fall
global floor9_flat
global floor10_button
global vsmooth
global rad_sph

v=vsmooth
x=posEE(1);
y=posEE(2);
z=posEE(3);

%large_button_1
xc=large_button_1.XData;
yc=large_button_1.YData;
zc=large_button_1.ZData;
if(x<xc(1) && x<xc(2) && x<xc(3) && x<xc(4)&& ...
   y<yc(1) && y<yc(2) && y>yc(3) && y>yc(4)&& ...
   z>zc(1) && z<zc(2) && z<zc(3) && z>zc(4))
    if((x-x(1))<10)
    fx=-k*button_stiffness*(x-x(1));
    elseif((x-x(1))>10&&(x-x(1))<20)
            fx=0;
    else
        fx=-k*(x-x(1));
    end
fy=0;
fz=0;
F=[fx;fy;fz];
end

%floor1_flat
xc=floor1_flat.XData;
yc=floor1_flat.YData;
zc=floor1_flat.ZData;
if(x<xc(1) && x>xc(2) && x>xc(3) && x<xc(4)&& ...
   y<yc(1) && y<yc(2) && y>yc(3) && y>yc(4)&& ...
   z<zc(1) && z<zc(2) && z<zc(3) && z<zc(4))
fx=0;
fy=0;
fz=-k*(z-zc(1));
F=[fx;fy;fz];
end

%floor2_texturesurface1
xc=floor2_texturesurface1.XData;
yc=floor2_texturesurface1.YData;
zc=floor2_texturesurface1.ZData;
if(x<xc(1) && x>xc(2) && x>xc(3) && x<xc(4)&& ...
        y<yc(1) && y<yc(2) && y>yc(3) && y>yc(4)&& ...
        z<(((xc(1)-x)*1.5)+zc(1)))
    Fn = k *(z-zc(1));
    fx=-Fn*v(1)*sin(pi + z-zc(1));
    fy=-Fn*v(2)*sin(pi + z-zc(1));
    fz=-Fn*v(3)*sin(pi + z-zc(1));
    F=[fx;fy;fz];
end

%floor3_viscussurface
xc=floor3_viscussurface.XData;
yc=floor3_viscussurface.YData;
zc=floor3_viscussurface.ZData;
if(x<xc(1) && x>xc(2) && x>xc(3) && x<xc(4)&& ...
        y<yc(1) && y<yc(2) && y>yc(3) && y>yc(4)&& ...
        z<zc(1) && z<zc(2) && z<zc(3) && z<zc(4))
    % Have to modify c, m to check what value looks good upon displaying, and
    % t after incorporating tic-toc timeval in code
    Fn = k * (z-zc(1));
    % Send in velocity vector as a [x, y, z] form, i.e., 1 x 3 
    fx=-c*Fn*v(1);
    fy=-c*Fn*v(2);
    fz=-c*Fn*v(3);
    F=[fx;fy;fz];
end

%floor4_texturesurface2
xc=floor4_texturesurface2.XData;
yc=floor4_texturesurface2.YData;
zc=floor4_texturesurface2.ZData;
if(x>xc(1) && x>xc(2) && x<xc(3) && x<xc(4)&& ...
        y<yc(1) && y>yc(2) && y>yc(3) && y<yc(4)&& ...
        z<((((yc(1)-y)*1))+zc(1)))
    Fn = k * (z-zc(1));
    fx=-Fn*v(1)*sin(pi + z+zc(1));
    fy=-Fn*v(2)*sin(pi + z+zc(1));
    fz=-Fn*v(3)*sin(pi + z+zc(1));    
    F=[fx;fy;fz];
end


%floor5_flat
xc=floor5_flat.XData;
yc=floor5_flat.YData;
zc=floor5_flat.ZData;
if(x>xc(1) && x>xc(2) && x<xc(3) && x<xc(4)&& ...
   y<yc(1) && y>yc(2) && y>yc(3) && y<yc(4)&& ...
   z<zc(1) && z<zc(2) && z<zc(3) && z<zc(4))
fx=0;
fy=0;
fz=-k*(z-zc(1));
F=[fx;fy;fz];
end

%ball_coll
xc=spx;
yc=spy;
zc=spz;
if (((x-xc)^2+(y-yc)^2+(z-zc)^2)<rad_sph^2)
    % Have to change attractive field based on how good it looks when
    % displayed
    
    fx =-k*(x-xc(1));
    fy=-k*(y-yc(1));
    fz=-k*(z-zc(1));
    F=[fx;fy;fz];
end

%attractive todo
xc=spx;
yc=spy;
zc=spz;
if (((x-xc)^2+(y-yc)^2+(z-zc)^2)<rad_sph^2)
    % Have to change attractive field based on how good it looks when
    % displayed
    
    fx =-k*(x-xc(1));
    fy=-k*(y-yc(1));
    fz=-k*(z-zc(1));
    F=[fx;fy;fz];
end

%floor7_flat
xc=floor7_flat.XData;
yc=floor7_flat.YData;
zc=floor7_flat.ZData;
if(x<xc(1) && x<xc(2) && x>xc(3) && x>xc(4)&& ...
   y<yc(1) && y>yc(2) && y>yc(3) && y<yc(4)&& ...
   z<zc(1) && z<zc(2) && z<zc(3) && z<zc(4))
fx=0;
fy=0;
fz=-k*(z-zc(1));
F=[fx;fy;fz];
end

%floor8_gravity_fall
xc=floor8_gravity_fall.XData;
yc=floor8_gravity_fall.YData;
zc=floor8_gravity_fall.ZData;
if(x>xc(1) && x>xc(2) && x<xc(3) && x<xc(4)&& ...
   y<yc(1) && y>yc(2) && y>yc(3) && y<yc(4)&& ...
   z<zc(1) && z<zc(2) && z<zc(3) && z<zc(4))
fx=0;
fy=0;
fz=-k*(z-zc(1));
F=[fx;fy;fz];
end

%floor9_flat
xc=floor9_flat.XData;
yc=floor9_flat.YData;
zc=floor9_flat.ZData;
if(x<xc(1) && x<xc(2) && x>xc(3) && x>xc(4)&& ...
   y<yc(1) && y>yc(2) && y>yc(3) && y<yc(4)&& ...
   z<zc(1) && z<zc(2) && z<zc(3) && z<zc(4))
fx=0;
fy=0;
fz=-k*(z-zc(1));
F=[fx;fy;fz];
end

%floor10_button
xc=floor10_button.XData;
yc=floor10_button.YData;
zc=floor10_button.ZData;
if(x<xc(1) && x<xc(2) && x>xc(3) && x>xc(4)&& ...
   y<yc(1) && y>yc(2) && y>yc(3) && y<yc(4)&& ...
   z<zc(1) && z<zc(2) && z<zc(3) && z<zc(4))
fx=0;
fy=0;
   if((z-zc(1))<10)
    fz=-k*button_stiffness*(z-zc(1));
    elseif((z-zc(1))>10&&(z-zc(1))<20)
            fz=0;
    else
        fz=-k*(z-zc(1));
   end
F=[fx;fy;fz];
end


% Fill this in

end

