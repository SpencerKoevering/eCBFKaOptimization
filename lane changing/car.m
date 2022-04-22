classdef car
   properties
      State {mustBeNumeric};
      l_r;
      l_f;
   end
   methods
      function obj = car(state, l_r, l_f)
         obj.State = state;
         obj.l_r = l_r;
         obj.l_f = l_f;
      end
      function r = increment(obj, a, B)
         f = [obj.State(4)*cos(obj.State(3));
             obj.State(4)*sin(obj.State(3));
             0;
             0;];
         g = [0 -obj.State(4)*sin(obj.State(3)); 
             0 obj.State(4)*cos(obj.State(3)); 
             0 obj.State(4)/obj.l_r; 
             1 0;];
         obj.State = obj.State+f+g*[a B].';
         r=obj.State;
      end
      function r = increment_probx(obj, a, B, xmean, xsd)
         e1 = normrnd(xmean, xsd);
         f = [obj.State(4)*cos(obj.State(3));
             obj.State(4)*sin(obj.State(3));
             0;
             0;];
         g = [0 -obj.State(4)*sin(obj.State(3)) 1; 
             0 obj.State(4)*cos(obj.State(3)) 0; 
             0 obj.State(4)/obj.l_r 0; 
             1 0 0;];
         obj.State = obj.State+f+g*[a B e1].';
         r=obj.State;
      end
      function r = increment_prob(obj, a, B, xmean, xsd, ymean, ysd)
         e1 = normrnd(xmean, xsd);
         e2 = normrnd(ymean, ysd);
         f = [obj.State(4)*cos(obj.State(3));
             obj.State(4)*sin(obj.State(3));
             0;
             0;];
         g = [0 -obj.State(4)*sin(obj.State(3)) 1 0; 
             0 obj.State(4)*cos(obj.State(3)) 0 1; 
             0 obj.State(4)/obj.l_r 0 0; 
             1 0 0 0;];
         obj.State = obj.State+f+g*[a B e1 e2].';
         r=obj.State;
      end
      function [r, a, B] = lyapunv_inc_prob(abs_target_speed, target_y, target_x, isvertical)
         e1 = normrnd(xmean, xsd);
         e2 = normrnd(ymean, ysd);
         x = obj.State(1);
         y = obj.State(2);
         psi = obj.State(3);
         v = obj.State(4);
         f = [obj.State(4)*cos(obj.State(3));
             obj.State(4)*sin(obj.State(3));
             0;
             0;];
         g = [0 -obj.State(4)*sin(obj.State(3)) 1 0; 
             0 obj.State(4)*cos(obj.State(3)) 0 1; 
             0 obj.State(4)/obj.l_r 0 0; 
             1 0 0 0;];
         
         target_speed = min([fc.State(4)*1.25, abs_target_speed]);
         
         if isvertical
             %% lateral position CLF
             h_y = y - target_y;
             V_y = h_y^2;
             phi0_y = 2 * h_y * (v * sin(psi)) + alpha_y * V_y;
             phi1_y = [0, 2 * h_y * v * cos(psi)];
         else
             %% x position CLF
             h_x = x - target_x;
             V_x = h_x^2;
             phi0_x = 2 * h_x * (v * cos(psi)) + alpha_y * V_x;
             phi1_x = [0, 2 * h_x * v * -sin(psi)];
         end
             
         %% velocity CLF
         h_v = v - target_speed;
         V_v = h_v^2;
         phi0_v = alpha_v * V_v;
         phi1_v = [2 * h_v * 1, 0];

         %% yaw angle CLF
         h_yaw = psi;
         V_yaw = h_yaw^2;
         phi0_yaw = alpha_yaw * V_yaw;
         phi1_yaw = [0, 2 * h_yaw * v * l_er];

         A_u = [1, 0, 0, 0, 0,0,0,0,0,0,0; ...
             -1, 0, 0, 0, 0,0,0,0,0,0,0; ...
             0, 1, 0, 0, 0,0,0,0,0,0,0; ...
             0, -1, 0, 0, 0,0,0,0,0,0,0; ...
             cos(psi + beta), 0, 0, 0, 0,0,0,0,0,0,0; ...
             -cos(psi + beta), 0, 0, 0, 0,0,0,0,0,0,0];
         A_u0 = [0, 1, 0, 0, 0,0,0,0,0,0,0; ...
             0, -1, 0, 0, 0,0,0,0,0,0,0];
         b_u = [lim_acc; lim_acc; lim_beta; lim_beta; 0.5 * 0.9 * 9.81; 0.5 * 0.9 * 9.81];
         b_u0 = [beta + 1 * lim_slip_rate * dt; -beta + 1 * lim_slip_rate * dt];
        %%QP
         if isvertical
             %% lateral position CLF
             Aclf = [phi1_y, -1, 0, 0,0,0,0,0,0,0; phi1_v, 0, -1,0,0,0,0,0,0, 0;phi1_yaw, 0, 0, -1,0,0,0,0,0,0;];
             bclf = [-phi0_y;-phi0_v; -phi0_yaw];
         else
             %% lateral position CLF
             Aclf = [phi1_x, -1, 0, 0,0,0,0,0,0,0; phi1_v, 0, -1,0,0,0,0,0,0, 0;phi1_yaw, 0, 0, -1,0,0,0,0,0,0;];
             bclf = [-phi0_x;-phi0_v; -phi0_yaw];
         end

         Constraint_A = [Aclf; A_u; A_u0];
         Constraint_b = [bclf; b_u; b_u0];
         
         objective_fun=@(x) x*H*x.';
         [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[]);
         obj.State = obj.State+f+g*[output(1) output(2) e1 e2].';
         r = obj.State;
         a = output(1);
         B = output(2);
      end
   end
end