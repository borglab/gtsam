function CHECK(name,assertion)

if (assertion~=1)
  error(['CHECK ' name ' fails']);
end
