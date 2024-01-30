clc
clear all

% User profiles data
LCW_HR = [80, 65, 61];
LCW_RR = [16, 13, 17];
HCW_HR = [95, 71, 92];
HCW_RR = [21, 14, 26];

% Calculating the mean for LCW and HCW for HR and RR
avg_LCW_HR = mean(LCW_HR);
avg_LCW_RR = mean(LCW_RR);
avg_HCW_HR = mean(HCW_HR);
avg_HCW_RR = mean(HCW_RR);

% Standard deviations for each user profile
std_LCW_HR_vals = [14, 15, 14];
std_LCW_RR_vals = [6, 4, 8];
std_HCW_HR_vals = [26, 21, 23];
std_HCW_RR_vals = [14, 5, 16];

% Calculating the average standard deviations
std_LCW_HR = mean(std_LCW_HR_vals);
std_LCW_RR = mean(std_LCW_RR_vals);
std_HCW_HR = mean(std_HCW_HR_vals);
std_HCW_RR = mean(std_HCW_RR_vals);

% Generate 100 Gaussian samples for each
samples_LCW_HR = normrnd(avg_LCW_HR, std_LCW_HR, [100, 1]);
samples_LCW_RR = normrnd(avg_LCW_RR, std_LCW_RR, [100, 1]);
samples_HCW_HR = normrnd(avg_HCW_HR, std_HCW_HR, [100, 1]);
samples_HCW_RR = normrnd(avg_HCW_RR, std_HCW_RR, [100, 1]);


deceleration_limits = [-150, -200];
gain_range_size = length(gain_range);
gain_range = [30000,50000,70000,80000,90000];

for k = 1:length(deceleration_limits)
   for j = 1:gain_range_size
    rng(0, 'twister')
    num_human_switches = 0;
    num_collisions = 0;
        for i = 20:40 
            Gain = gain_range(j); %depends on the range of values 
            starting_speed = i; 
            decelLim = deceleration_limits(k);
            [A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain);
            open_system('LaneMaintainSystem.slx')
        
            set_param('LaneMaintainSystem/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim))
            set_param('LaneMaintainSystem/VehicleKinematics/vx','InitialCondition',num2str(starting_speed))
        
            simulation_model = sim('LaneMaintainSystem.slx');
        
        %human deceleration limit is 10% higher than the deceleration limit
        %of the car
            decelLim_human = 1.1*decelLim;
            if decelLim == -200
                Reaction_Quotient = samples_LCW_HR(randi(100))/ samples_LCW_RR(randi(100));
            else
                Reaction_Quotient = samples_HCW_HR(randi(100))/ samples_HCW_RR(randi(100));
            end 

            reaction_time = 0.01*Reaction_Quotient;
        
            open_system('HumanActionModel.slx')
        
            set_param('HumanActionModel/VehicleKinematics/Saturation','LowerLimit',num2str(decelLim_human))
            set_param('HumanActionModel/VehicleKinematics/vx','InitialCondition',num2str(starting_speed));        
            set_param('HumanActionModel/Step','Time',num2str(reaction_time))
            set_param('HumanActionModel/Step','After',num2str(decelLim_human))
        
            human_action = sim("HumanActionModel.slx");
        
            action_time = human_action.sx1.Time(end);
            collision_time = simulation_model.sx1.Time(end);
        
            if simulation_model.sx1.Data(end) >=0
                total_time = reaction_time + action_time;
                if total_time < collision_time
                    num_human_switches = num_human_switches + 1;
                    if human_action.sx1.Data(end) >= 0
                        num_collisions = num_collisions + 1;
                    end
                else 
                    num_collisions = num_collisions + 1;      
                end
            else
                continue
            end
        end
        if decelLim == -200
            fprintf("Number of collisions in Low Cognitive Workload mode for gain %d = %.2f\n", gain_range(j), num_collisions);
            fprintf("Number of switches in Low Cognitive Workload mode for gain %d = %.2f\n", gain_range(j), num_human_switches);
        else
            fprintf("Number of collisions in High Cognitive Workload for gain %d = %.2f\n", gain_range(j), num_collisions);
            fprintf("Number of switches in High Coginitive Workload for gain %d = %.2f\n", gain_range(j), num_human_switches);
        end
   end
end