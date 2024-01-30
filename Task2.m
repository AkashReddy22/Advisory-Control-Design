start_speed_min = 20;
start_velocity_up = 60;
lcw_Gain = 35000;
hcw_Gain = 85000;
lcw_Dec_Limit = -200;
hcw_Dec_Limit = -150;

steps_cnt = 100;
P = [0.6 0.4; 0.85 0.15];
monte_carlo_simulation = dtmc(P);
roadCond_mc = simulate(monte_carlo_simulation,steps_cnt); % 1 is Normal, 2 is Poor'

LCW_avg_HR_User3 = 61;
LCW_avg_RR_User3 = 17;
LCW_std_HR_User3 = 14;
LCW_std_RR_User3 = 8;

HCW_avg_HR_User3 = 92;
HCW_avg_RR_User3 = 26;
HCW_std_HR_User3 = 23;
HCW_std_RR_User3 = 16;

factor = 18;
speed_init = zeros(1, steps_cnt);
reactiontimes_arr = zeros(1, steps_cnt);
coll_num = zeros(1, steps_cnt);
switch_num = zeros(1, steps_cnt);
for i = 1:steps_cnt
    speed_init(i) = start_speed_min + (start_velocity_up - start_speed_min) * rand();
    if reactiontimes_arr(i) == 1
        HR = normrnd(LCW_avg_HR_User3, LCW_std_HR_User3);
        RR = normrnd(LCW_avg_RR_User3, LCW_std_RR_User3);
        HCW_reaction_time = 0.01*(HR/RR);
    else
        HR = normrnd(HCW_avg_HR_User3, HCW_std_HR_User3);
        RR = normrnd(HCW_avg_RR_User3, HCW_std_RR_User3);
        LCW_reaction_time = 0.01*(HR/RR);
    end
    reactiontimes_arr(i) = 0.01*(HR/RR);
end
for i = 1:steps_cnt
    if(roadCond_mc(i)==1)
        HR = normrnd(LCW_avg_HR_User3, LCW_std_HR_User3);
        RR = normrnd(LCW_avg_RR_User3, LCW_std_RR_User3);
        HCW_reaction_time = 0.01*(HR/RR);
        Gain = lcw_Gain;
        decel_Lim = lcw_Dec_Limit;
        react_time = HCW_reaction_time;
    else
        HR = normrnd(HCW_avg_HR_User3, HCW_std_HR_User3);
        RR = normrnd(HCW_avg_RR_User3, HCW_std_RR_User3);
        LCW_reaction_time = 0.01*(HR/RR);
        Gain = hcw_Gain;
        decel_Lim = hcw_Dec_Limit;
        react_time = LCW_reaction_time;
    end

    [A,B,C,D,Kess, Kr, Ke, uD] = designControl(secureRand(),Gain);
    open_system('LaneMaintainSystem.slx')

    set_param('LaneMaintainSystem/VehicleKinematics/Saturation','LowerLimit',num2str(decel_Lim))
    set_param('LaneMaintainSystem/VehicleKinematics/vx','InitialCondition',num2str(speed_init(i)))

    simModel = sim('LaneMaintainSystem.slx');
    if max(simModel.sx1.Data)<0
        coll_num(i)=0;
    else
        open_system('HumanActionModel.slx')
        set_param('HumanActionModel/Step','Time',num2str(react_time*factor))
        set_param('HumanActionModel/Step','After',num2str(1.1*decel_Lim))
        set_param('HumanActionModel/VehicleKinematics/vx','InitialCondition',num2str(speed_init(i)))
        humanmodel1 = sim('HumanActionModel.slx');

        collision_dt = simModel.sx1.Time(end);
        h_stop_time = humanmodel1.sx1.Time(end);
        if h_stop_time<collision_dt
            switch_num(i) = 1;

            open_system('HumanActionModel.slx')
            set_param('HumanActionModel/Step','Time',num2str(reactiontimes_arr(i)))
            set_param('HumanActionModel/Step','After',num2str(1.1*decel_Lim))
            set_param('HumanActionModel/VehicleKinematics/vx','InitialCondition',num2str(speed_init(i)))
            human_action_Model2 = sim('HumanActionModel.slx');

            if max(human_action_Model2.sx1.Data) > 0
                coll_num(i) = 1;
            end
        else
            coll_num(i) = 0;
        end
     end
end
disp("num_Switches: " + sum(switch_num(:) == 1));
disp("num_Collisions: " + sum(coll_num(:) == 1))