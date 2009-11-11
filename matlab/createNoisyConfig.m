% A noise configuration
function c = createNoisyConfig()
v_x1 = [0.1;  0.1];
v_x2 = [1.4;  0.2];
v_l1 = [0.1; -1.1];
c = VectorConfig();
c.insert('x1', v_x1);
c.insert('x2', v_x2);
c.insert('l1', v_l1);
end