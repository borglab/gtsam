function CHECK(name,assertion)
% CHECK throw an error if an assertion fails
%
% CHECK(name,assertion)
%  - name of test
%  - assertion

if (assertion~=1)
  error(['CHECK ' name ' fails']);
end
