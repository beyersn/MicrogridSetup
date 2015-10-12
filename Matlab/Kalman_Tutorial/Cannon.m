classdef Cannon < handle
    %Cannon
    %   Detailed explanation goes here
    
    properties
        angle = 45;
        muzzle_velocity = 100;
        accel = [0, -9.81];
        loc = [0, 0];
        ts;
        noise;
        velocity;
    end
    
    methods
        %Constructor for the cannon
        function obj = Cannon(ts_,noise_)
            obj.ts = ts_;
            obj.noise = noise_;
            obj.velocity = [obj.muzzle_velocity*cos(degtorad(obj.angle)),obj.muzzle_velocity*sin(degtorad(obj.angle))];
        end
        %Model the physics of the cannon
        function step(obj)
            obj.velocity = obj.velocity + obj.ts*obj.accel;
            obj.loc = obj.loc + obj.ts*obj.velocity;
        end
        %Return x pos
        function xpos = getXPos(obj)
            xpos = obj.loc(1);
        end
        %Return y pos
        function ypos = getYPos(obj)
            ypos = obj.loc(2);
        end
        %Return x pos
        function xposn = getXWNoise(obj)
            xposn = obj.loc(1) + obj.noise*randn();
        end
        %Return y pos
        function yposn = getYWNoise(obj)
            yposn = obj.loc(2) + obj.noise*randn();
        end
        %Return x velocity
        function xvel = getXVel(obj)
            xvel = obj.velocity(1);
        end
        %Return y velocity
        function yvel = getYVel(obj)
            yvel = obj.velocity(2);
        end
    end
    
end

