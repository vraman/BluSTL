HR2 = hvac_room();
HR2 = init_control(HR2,12,0.1, 50);
HR2.run_adversarial(HR2.controller, HR2.adversary)
%HR2.run_deterministic(HR2.controller);
%save2pdf('hvac_nondet2.pdf', gcf)

