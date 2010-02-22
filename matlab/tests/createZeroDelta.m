% A zero delta configuration
function c = createZeroDelta()
v_x1 = [0; 0];
v_x2 = [0; 0];
v_l1 = [0; 0];
c = VectorConfig();
c.insert('x1', v_x1);
c.insert('x2', v_x2);
c.insert('l1', v_l1);
end