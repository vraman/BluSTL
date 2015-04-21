%HR1 = hvac_room;
%HR1 = init_control(HR1,12,0, 400);
%HR1.run_adversarial(HR1.controller, HR1.adversary)
%HR1.run_deterministic(HR1.controller);
%save2pdf('hvac_det2.pdf', gcf)

HR2 = hvac_room();
HR2 = init_control(HR2,12,0, 400);
%HR2.run_adversarial(HR2.controller, HR2.adversary)
%HR2.run_deterministic(HR2.controller);
%save2pdf('hvac_nondet2.pdf', gcf)
