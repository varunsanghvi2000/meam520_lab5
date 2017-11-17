% Fill this script in with your position tracking, force computation,
% and graphics for the virtual environment

close all

%% Run on hardware or simulation
hardwareFlag = false;

%% Plot end effector in environment
global qs % configuration (NOTE: This is only 3 angles now)
global posEE % position of end effectr
global large_button_1
global floor1_flat
global floor2_texturesurface1
global floor3_viscussurface
global floor4_texturesurface2
global floor5_flat
%global floor6_attractionfield
global floor7_flat
global floor8_gravity_fall
global floor9_viscussurface1
global floor10_button
global vsmooth
global spx
global spy
global spz

figClosed = 0;
qs = [0,0,0]; % initialize robot to zero pose
posEE = [0,0,0];  % initialize position of end effector

hold on; scatter3(0, 0, 0, 'kx', 'Linewidth', 2); % plot origin
h1 = scatter3(0, 0, 0, 'r.', 'Linewidth', 10); % plot end effector position
h2 = quiver3(0, 0, 0, 0, 0, 0, 'b'); % plot output force
if ~hardwareFlag
    h_fig = figure(1);
    set(h_fig, 'Name','Haptic environment: Close figure to quit.' ,'KeyPressFcn', @(h_obj, evt) keyPressFcn(h_obj, evt));
end

%% Create Environment here:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create static objects and interactive objects in their initial state

% Example of a flat plane
%start button
sx=-0.2;
sy=0.3;
sz=0.3;
shx=120;
shy=-50;
shz=0;


large_button_1 = fill3( [sx*600+shx sx*600+shx sx*600+shx sx*600+shx],[sy*0+shy sy*0+shy sy*-400+shy sy*-400+shy], [sz*600+shz sz*800+shz sz*800+shz sz*600+shz], [1 0 0], 'facealpha', 0.8);
floor1_flat = fill3( [sx*200+shx sx*600+shx sx*600+shx sx*200+shx], [sy*0+shy sy*0+shy sy*-400+shy sy*-400+shy],[sz*600+shz sz*600+shz sz*600+shz sz*600+shz], [0 1 0], 'facealpha', 0.5);
 %loor1button1 = fill3([-100 -100 -300 -300], [425 550 550 425], [601 601 601 601], [0.7 0 0], 'facealpha', 0.3);
 %floor1button2 = fill3([-100 -100 -300 -300], [250 375 375 250], [601 601 601 601], [0.7 0 0], 'facealpha', 0.3);
floor2_texturesurface1 = fill3( [sx*-200+shx sx*200+shx sx*200+shx sx*-200+shx],[sy*0+shy sy*0+shy sy*-400+shy sy*-400+shy], [sz*200+shz sz*600+shz sz*600+shz sz*200+shz], [1 1 0], 'facealpha', 0.5);
floor3_viscussurface = fill3( [sx*-600+shx sx*-200+shx sx*-200+shx sx*-600+shx],[sy*0+shy sy*0+shy sy*-400+shy sy*-400+shy], [sz*200+shz sz*200+shz sz*200+shz sz*200+shz], [0 0 1], 'facealpha', 0.5);

floor4_texturesurface2 = fill3( [sx*-200+shx sx*-200+shx sx*-600+shx sx*-600+shx], [sy*400+shy sy*0+shy sy*0+shy sy*400+shy],[sz*-200+shz sz*200+shz sz*200+shz sz*-200+shz], [1 1 0], 'facealpha', 0.5);
floor5_flat = fill3( [sx*-200+shx sx*-200+shx sx*-600+shx sx*-600+shx],[sy*800+shy sy*400+shy sy*400+shy sy*800+shy], [sz*-200+shz sz*-200+shz sz*-200+shz sz*-200+shz], [0 1 0], 'facealpha', 0.5);
%floor6_attractionfield = fill3( [sx*-600+shx sx*-600+shx sx*-600+shx sx*-600+shx],[sy*400+shy sy*800+shy sy*800+shy sy*400+shy], [sz*-200+shz sz*-200+shz sz*0+shz sz*0+shz], [0 1 0], 'facealpha', 0.5);

floor7_flat = fill3( [sx*-200+shx sx*-200+shx sx*-100+shx sx*-100+shx],[sy*800+shy sy*400+shy sy*400+shy sy*800+shy], [sz*-200+shz sz*-200+shz sz*-200+shz sz*-200+shz], [0 1 0], 'facealpha', 0.5);
floor8_gravity_fall = fill3( [sx*0+shx sx*0+shx sx*-100+shx sx*-100+shx],[sy*800+shy sy*400+shy sy*400+shy sy*800+shy], [sz*-300+shz sz*-300+shz sz*-300+shz sz*-300+shz], [0 1 0], 'facealpha', 0.5);
floor9_viscussurface1 = fill3([sx*0+shx sx*0+shx sx*200+shx sx*200+shx],[sy*800+shy sy*400+shy sy*400+shy sy*800+shy],  [sz*-200 sz*-200 sz*-200 sz*-200], [0 0 1], 'facealpha', 0.5);

floor10_button = fill3( [sx*200+shx sx*200+shx sx*400+shx sx*400+shx], [sy*800+shy sy*400+shy sy*400+shy sy*800+shy], [sz*-200+shz sz*-200+shz sz*-200+shz sz*-200+shz], [1 0 0], 'facealpha', 0.8);
told=0;
%attractive
[x,y,z] = sphere;
  r=30;
   spx=200;
   spy=130;
   spz=-50;
 hsphere2= surf(x*r+spx, y*r+spy, z*r+spz, 'facealpha', 0.8);
 hsphere2.EdgeColor = 'none';
 %ball

 [x,y,z] = sphere;
  r=15;
   spx=150;
   spy=180;
   spz=-45;
 hsphere1= surf(x*r+spx, y*r+spy, z*r+spz);
%hsphere1.get
%hsphere1.XData
%floor1_flat = fill3([200 100 100 200], [-100 -100 100 100], [200 200 200 200], [0.7 0 0], 'facealpha', 0.3);

hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% set camera properties
axis([-100 300 -200 200 -200 300]);
view([150,14]);
count=0;
vsmooth = zeros();

i = 0; frameSkip = 3; % plotting variable - set how often plot updates
while(1)
    tic
    %% Read potentiometer values, convert to angles and end effector location
    if hardwareFlag
        qs = lynxGetAngles();
    end
    
    %% Calculate current end effector position
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    posEEold= posEE;
    posEE = computeEEposition();
    if count==0
        v=0 ;
    else
        v=(posEE-posEEold)/told;
    end
    vprevsmooth = vsmooth;
    vsmooth= 0.1*v +(1-0.1)*vprevsmooth;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% Calculate desired force based on current end effector position
    % Check for collisions with objects in the environment and compute the total force on the end effector
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F = computeForces();
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(spy==80)
    dir_step=5;
    elseif(spy==180)
    dir_step=-5;
    end
    %% Plot Environment
    if i == 0
        figClosed = drawLynx(h1, h2, F);
        [xhan,yhan,zhan]=sphere;
        spy=spy+dir_step;
        set(hsphere1, 'XData',xhan*r+spx, 'YData',yhan*r+spy, 'ZData',zhan*r+spz);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Set handles for interactive objects you make here
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		%set()
        drawnow
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Compute torques from forces and convert to currents for servos
    Tau = computeTorques(F);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if hardwareFlag
        if figClosed % quit by closing the figure
            lynxDCTorquePhysical(0,0,0,0,0,0);
            return;
        else
            currents = torquesToCurrents(Tau);
            lynxDCTorquePhysical(currents(1),currents(2),currents(3),0,0,0);
        end
    end
    
    if (figClosed) % quit by closing the figure
        return;
    end
    
    %% Debugging
    %posEE
    %qs
    %F'
    %Tau'
    %[posEE, qs, F', Tau']
     count=count+1;
     told=toc;
    i = mod(i+1, frameSkip);
end