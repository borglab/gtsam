function EXPECT(name,assertion)
% EXPECT throw a warning if an assertion fails
%
% EXPECT(name,assertion)
%  - name of test
%  - assertion

if (assertion~=1)
    warning(['EXPECT ' name ' fails']);
end
