%% BO1808NBH2B PMSM simulation setup entry point
% Run this file in MATLAB from this folder.

init_bo1808_params;
build_bo1808_modular_model;
build_bo1808_foc_id0_model;
build_bo1808_svpwm_model;
build_bo1808_svpwm_sfunc_model;
build_bo1808_foc_svpwm_model;

fprintf('\nSetup complete.\n');
fprintf('Models folder: %s\n', models_dir);
fprintf('Figures folder: %s\n', figures_dir);
fprintf('Reports folder: %s\n', reports_dir);
fprintf('Open-loop modular model: %s\n', fullfile(models_dir,'bo1808_fig1_9_modular.slx'));
fprintf('Closed-loop id=0 FOC model: %s\n', fullfile(models_dir,'bo1808_foc_id0.slx'));
fprintf('Standalone SVPWM model: %s\n', fullfile(models_dir,'bo1808_svpwm_sim.slx'));
fprintf('SVPWM S-Function model: %s\n', fullfile(models_dir,'bo1808_svpwm_sfunc_sim.slx'));
fprintf('Full FOC+SVPWM inverter model: %s\n', fullfile(models_dir,'bo1808_foc_svpwm_bo1808.slx'));
fprintf('S-Function plant file: pmsm_bo1808.m\n');
fprintf('SVPWM S-Function file: svpwm_bo1808_sfunc.m\n');
