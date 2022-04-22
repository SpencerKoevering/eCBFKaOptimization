classdef intersection_car_smooth < handle
   properties
      State {mustBeNumeric};
      l_r;
      l_f;
      w;
      ux;
      sx;
      uy;
      sy;
      mv;
      nloptions;
      nlobj;
      ref;
      ref_index;
      turnepoch;
      xbounds;
      ybounds;
      
      start_State {mustBeNumeric};
   end
   methods
      function obj = intersection_car_smooth(state, l_r, l_f, w, ref, Ts, ux, uy,sx, sy, xbounds, ybounds, px, py, ptheta, pa, pB, turnstartepoch, maxa, max_angle)
         obj.State = state;
         obj.start_State = state;
         obj.l_r = l_r;
         obj.l_f = l_f;
         obj.w = w;
         obj.ref=ref;
         obj.ref_index = 1;
         obj.ux = ux;
         obj.uy = uy;
         obj.sx = sx;
         obj.sy = sy;
         obj.turnepoch=turnstartepoch;
         obj.xbounds=xbounds;
         obj.ybounds=ybounds;
         
         obj.mv = [0;0];
         nx = 4;
         ny = 3;
         nu = 2;
         obj.nlobj = nlmpc(nx, ny, nu);
         obj.nlobj.Ts = Ts;
         obj.nlobj.PredictionHorizon = 10;
         obj.nlobj.ControlHorizon = 2;
         obj.nlobj.Model.IsContinuousTime = false;
         obj.nlobj.Model.OutputFcn = @(x, u, Ts) [obj.State(1); obj.State(2); obj.State(3)];
         obj.nlobj.Model.NumberOfParameters = 1;
         obj.nlobj.Model.StateFcn="kinematicbicyclefcn";

         maxB=atan(1/2*tan(max_angle)); %max steering angle is enforced through B. this value corresponds to pi/2 max
         
         obj.nlobj.MV(1).Min = -maxa;      %
         obj.nlobj.MV(1).Max = maxa;       %
         obj.nlobj.MV(2).Min = -maxB;   % 
         obj.nlobj.MV(2).Max = maxB;    %
         obj.nlobj.States(4).Min = -1;      
         obj.nlobj.States(4).Max = 1;
         obj.nlobj.States(3).Min = -3*pi/2;      
         obj.nlobj.States(3).Max = 0;
         obj.nlobj.States(1).Min = xbounds(1);      
         obj.nlobj.States(1).Max = xbounds(2);
         obj.nlobj.States(2).Min = ybounds(1);      
         obj.nlobj.States(2).Max = ybounds(2); 
         obj.nlobj.Weights.ManipulatedVariablesRate = [pa pB];
         obj.nlobj.Weights.OutputVariables = [px py ptheta];

         obj.nloptions = nlmpcmoveopt;
         obj.nloptions.Parameters = {Ts};
         
%          validateFcns(obj.nlobj,obj.State,obj.mv, [], {Ts});
      end
      function reset(obj)
         obj.State = obj.start_State;
         obj.ref_index = 1;
         obj.nlobj.States(1).Min = obj.xbounds(1);      
         obj.nlobj.States(1).Max = obj.xbounds(2);
         obj.nlobj.States(2).Min = obj.ybounds(1);      
         obj.nlobj.States(2).Max = obj.ybounds(2); 
      end
      function [a, B] = nlmpc_inc_prob_get_input(obj)
            yref = obj.ref(:,obj.ref_index)';
            [mvn, nloptionsn] = nlmpcmove(obj.nlobj, obj.State, obj.mv ,yref,[], obj.nloptions);
            obj.mv = mvn;
            obj.nloptions = nloptionsn;
            a=mvn(1);
            B=mvn(2);
      end
      function r = nlmpc_inc_prob_apply_input(obj, dt_dyn)
            yref = obj.ref(:,obj.ref_index)';
            psi = obj.State(3);
            v = obj.State(4);
            f = [v*cos(psi); v*sin(psi);0;0;];
            g = [0 -v*sin(psi) 1 0; 0 v*cos(psi) 0 1; 0 v/obj.l_r 0 0; 1 0 0 0;];
            input = [obj.mv; normrnd(obj.ux, obj.sx);normrnd(obj.uy, obj.sy);];
            obj.State = obj.State+dt_dyn*(f+g*input);
            V = obj.State(1:2)-yref(1:2)';
            if(V*V' < 4)
                dims = size(obj.ref);
                if obj.turnepoch == obj.ref_index
                    obj.nlobj.States(1).Min = obj.xbounds(3);      
                    obj.nlobj.States(1).Max = obj.xbounds(4);
                    obj.nlobj.States(2).Min = obj.ybounds(3);      
                    obj.nlobj.States(2).Max = obj.ybounds(4);
                end
                obj.ref_index = min(obj.ref_index+1, dims(2));
            end
            r = obj.State;
      end
   end
end