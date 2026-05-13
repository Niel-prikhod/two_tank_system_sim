function set_mfunction_block(mdl, blockName, code)
%SET_MATLAB_FUNCTION_CODE  Programmatically sets code inside a MATLAB
%                          Function block
%   set_matlab_function_code('TwoTankValveModel', 'ODE', codeString)

path = [mdl '/' blockName];
cfg = get_param(path, 'MATLABFunctionConfiguration');
cfg.FunctionScript = code;

end
