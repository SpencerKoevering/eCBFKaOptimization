function [originput,fval,exitflag,output] = doFmincon_det_const(objective_fun,Constraint_A,Constraint_b, options, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, prev_alphafc, prev_alphaft, prev_alphabt, updateKa)  
    if(updateKa)
        nonlcon=@(input) tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, [], [], []);
        [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[], nonlcon, options);
        if exitflag == 1 || 2
            return
        end

        if ~isempty(prev_alphafc)
            nonlcon=@(input) tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, prev_alphafc, [], []);
            [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[], nonlcon, options);
        end
        if exitflag == 1 || 2
            return
        end

        if ~isempty(prev_alphaft)
            nonlcon=@(input) tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, [], prev_alphaft, []);
            [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[], nonlcon, options);
        end
        if exitflag == 1 || 2
            return
        end

        if ~isempty(prev_alphabt)
            nonlcon=@(input) tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, [], [], prev_alphabt);
            [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[], nonlcon, options);
        end
        if exitflag == 1 || 2
            return
        end

        if ~isempty(prev_alphafc) && ~isempty(prev_alphaft)
            nonlcon=@(input) tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, prev_alphafc, prev_alphaft, []);
            [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[], nonlcon, options);
        end
        if exitflag == 1 || 2
            return
        end

        if ~isempty(prev_alphafc) && ~isempty(prev_alphabt)
            nonlcon=@(input) tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, prev_alphafc, [], prev_alphabt);
            [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[], nonlcon, options);
        end
        if exitflag == 1 || 2
            return
        end

        if ~isempty(prev_alphaft) && ~isempty(prev_alphabt)
            nonlcon=@(input) tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, [], prev_alphaft, prev_alphabt);
            [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[], nonlcon, options);
        end
        if exitflag == 1 || 2
            return
        end

        if ~isempty(prev_alphafc) && ~isempty(prev_alphaft) && ~isempty(prev_alphabt)
            nonlcon=@(input) tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, prev_alphafc, prev_alphaft, prev_alphabt);
            [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[], nonlcon, options);
        end
        if exitflag == 1 || 2
            return
        end
    else
         nonlcon=@(input) tECBF_det_optim(input, car, fc, ft, bt, l_er, l_ef, l_fcr, l_fcf, l_ftr, l_ftf, l_btr, l_btf, afc, Bfc, aft, Bft, abt, Bbt, usefc, useft, usebt,rval, prev_alphafc, prev_alphaft, prev_alphabt);
         [originput,fval,exitflag,output] = fmincon(objective_fun,[0 0 100 100 100 2 4 2 4 2 4],Constraint_A,Constraint_b,[],[],[],[], nonlcon, options);
    end
    
end