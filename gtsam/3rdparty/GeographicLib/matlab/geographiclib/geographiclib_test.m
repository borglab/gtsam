function geographiclib_test
%GEOGRAPHICLIB_TEST   The test suite for the geographiclib package
%
%   GEOGRAPHICLIB_TEST
%
%   runs a variety of tests and produces no output it they are successful.

  n = 0;
  i = testrand; if i, n=n+1; fprintf('testrand fail: %d\n', i); end
  i = GeoConvert0 ; if i, n=n+1; fprintf('GeoConvert0  fail: %d\n', i); end
  i = GeoConvert8 ; if i, n=n+1; fprintf('GeoConvert8  fail: %d\n', i); end
  i = GeoConvert16; if i, n=n+1; fprintf('GeoConvert16 fail: %d\n', i); end
  i = GeoConvert17; if i, n=n+1; fprintf('GeoConvert17 fail: %d\n', i); end
  i = GeodSolve0 ; if i, n=n+1; fprintf('GeodSolve0  fail: %d\n', i); end
  i = GeodSolve1 ; if i, n=n+1; fprintf('GeodSolve1  fail: %d\n', i); end
  i = GeodSolve2 ; if i, n=n+1; fprintf('GeodSolve2  fail: %d\n', i); end
  i = GeodSolve4 ; if i, n=n+1; fprintf('GeodSolve4  fail: %d\n', i); end
  i = GeodSolve5 ; if i, n=n+1; fprintf('GeodSolve5  fail: %d\n', i); end
  i = GeodSolve6 ; if i, n=n+1; fprintf('GeodSolve6  fail: %d\n', i); end
  i = GeodSolve9 ; if i, n=n+1; fprintf('GeodSolve9  fail: %d\n', i); end
  i = GeodSolve10; if i, n=n+1; fprintf('GeodSolve10 fail: %d\n', i); end
  i = GeodSolve11; if i, n=n+1; fprintf('GeodSolve11 fail: %d\n', i); end
  i = GeodSolve12; if i, n=n+1; fprintf('GeodSolve12 fail: %d\n', i); end
  i = GeodSolve14; if i, n=n+1; fprintf('GeodSolve14 fail: %d\n', i); end
  i = GeodSolve15; if i, n=n+1; fprintf('GeodSolve15 fail: %d\n', i); end
  i = GeodSolve17; if i, n=n+1; fprintf('GeodSolve17 fail: %d\n', i); end
  i = GeodSolve26; if i, n=n+1; fprintf('GeodSolve26 fail: %d\n', i); end
  i = GeodSolve28; if i, n=n+1; fprintf('GeodSolve28 fail: %d\n', i); end
  i = GeodSolve33; if i, n=n+1; fprintf('GeodSolve33 fail: %d\n', i); end
  i = GeodSolve55; if i, n=n+1; fprintf('GeodSolve55 fail: %d\n', i); end
  i = GeodSolve59; if i, n=n+1; fprintf('GeodSolve59 fail: %d\n', i); end
  i = GeodSolve61; if i, n=n+1; fprintf('GeodSolve61 fail: %d\n', i); end
  i = GeodSolve73; if i, n=n+1; fprintf('GeodSolve73 fail: %d\n', i); end
  i = GeodSolve74; if i, n=n+1; fprintf('GeodSolve74 fail: %d\n', i); end
  i = GeodSolve76; if i, n=n+1; fprintf('GeodSolve76 fail: %d\n', i); end
  i = GeodSolve78; if i, n=n+1; fprintf('GeodSolve78 fail: %d\n', i); end
  i = Planimeter0 ; if i, n=n+1; fprintf('Planimeter0  fail: %d\n', i); end
  i = Planimeter5 ; if i, n=n+1; fprintf('Planimeter5  fail: %d\n', i); end
  i = Planimeter6 ; if i, n=n+1; fprintf('Planimeter6  fail: %d\n', i); end
  i = Planimeter12; if i, n=n+1; fprintf('Planimeter12 fail: %d\n', i); end
  i = Planimeter13; if i, n=n+1; fprintf('Planimeter13 fail: %d\n', i); end
  i = TransverseMercatorProj1;
  if i, n=n+1; fprintf('TransverseMercatorProj1 fail: %d\n', i); end
  i = TransverseMercatorProj3;
  if i, n=n+1; fprintf('TransverseMercatorProj3 fail: %d\n', i); end
  i = TransverseMercatorProj5;
  if i, n=n+1; fprintf('TransverseMercatorProj5 fail: %d\n', i); end
  i = geodreckon0; if i, n=n+1; fprintf('geodreckon0 fail: %d\n', i); end
  i = gedistance0; if i, n=n+1; fprintf('gedistance0 fail: %d\n', i); end
  i = tranmerc0; if i, n=n+1; fprintf('tranmerc0 fail: %d\n', i); end
  i = mgrs0; if i, n=n+1; fprintf('mgrs0 fail: %d\n', i); end
  i = mgrs1; if i, n=n+1; fprintf('mgrs1 fail: %d\n', i); end
  i = mgrs2; if i, n=n+1; fprintf('mgrs2 fail: %d\n', i); end
  i = mgrs3; if i, n=n+1; fprintf('mgrs3 fail: %d\n', i); end
  i = mgrs4; if i, n=n+1; fprintf('mgrs4 fail: %d\n', i); end
  assert(n == 0);
end

function n = assertEquals(x, y, d)
  n = abs(x - y) <= d;
  n = sum(~n(:));
end

function n = assertNaN(x)
  n = isnan(x);
  n = sum(~n(:));
end

function n = testrand
  n = 0;
  testcases = [
      35.60777, -139.44815, 111.098748429560326, ...
      -11.17491, -69.95921, 129.289270889708762, ...
      8935244.5604818305, 80.50729714281974, 6273170.2055303837, ...
      0.16606318447386067, 0.16479116945612937, 12841384694976.432;
      55.52454, 106.05087, 22.020059880982801, ...
      77.03196, 197.18234, 109.112041110671519, ...
      4105086.1713924406, 36.892740690445894, 3828869.3344387607, ...
      0.80076349608092607, 0.80101006984201008, 61674961290615.615;
      -21.97856, 142.59065, -32.44456876433189, ...
      41.84138, 98.56635, -41.84359951440466, ...
      8394328.894657671, 75.62930491011522, 6161154.5773110616, ...
      0.24816339233950381, 0.24930251203627892, -6637997720646.717;
      -66.99028, 112.2363, 173.73491240878403, ...
      -12.70631, 285.90344, 2.512956620913668, ...
      11150344.2312080241, 100.278634181155759, 6289939.5670446687, ...
      -0.17199490274700385, -0.17722569526345708, -121287239862139.744;
      -17.42761, 173.34268, -159.033557661192928, ...
      -15.84784, 5.93557, -20.787484651536988, ...
      16076603.1631180673, 144.640108810286253, 3732902.1583877189, ...
      -0.81273638700070476, -0.81299800519154474, 97825992354058.708;
      32.84994, 48.28919, 150.492927788121982, ...
      -56.28556, 202.29132, 48.113449399816759, ...
      16727068.9438164461, 150.565799985466607, 3147838.1910180939, ...
      -0.87334918086923126, -0.86505036767110637, -72445258525585.010;
      6.96833, 52.74123, 92.581585386317712, ...
      -7.39675, 206.17291, 90.721692165923907, ...
      17102477.2496958388, 154.147366239113561, 2772035.6169917581, ...
      -0.89991282520302447, -0.89986892177110739, -1311796973197.995;
      -50.56724, -16.30485, -105.439679907590164, ...
      -33.56571, -94.97412, -47.348547835650331, ...
      6455670.5118668696, 58.083719495371259, 5409150.7979815838, ...
      0.53053508035997263, 0.52988722644436602, 41071447902810.047;
      -58.93002, -8.90775, 140.965397902500679, ...
      -8.91104, 133.13503, 19.255429433416599, ...
      11756066.0219864627, 105.755691241406877, 6151101.2270708536, ...
      -0.26548622269867183, -0.27068483874510741, -86143460552774.735;
      -68.82867, -74.28391, 93.774347763114881, ...
      -50.63005, -8.36685, 34.65564085411343, ...
      3956936.926063544, 35.572254987389284, 3708890.9544062657, ...
      0.81443963736383502, 0.81420859815358342, -41845309450093.787;
      -10.62672, -32.0898, -86.426713286747751, ...
      5.883, -134.31681, -80.473780971034875, ...
      11470869.3864563009, 103.387395634504061, 6184411.6622659713, ...
      -0.23138683500430237, -0.23155097622286792, 4198803992123.548;
      -21.76221, 166.90563, 29.319421206936428, ...
      48.72884, 213.97627, 43.508671946410168, ...
      9098627.3986554915, 81.963476716121964, 6299240.9166992283, ...
      0.13965943368590333, 0.14152969707656796, 10024709850277.476;
      -19.79938, -174.47484, 71.167275780171533, ...
      -11.99349, -154.35109, 65.589099775199228, ...
      2319004.8601169389, 20.896611684802389, 2267960.8703918325, ...
      0.93427001867125849, 0.93424887135032789, -3935477535005.785;
      -11.95887, -116.94513, 92.712619830452549, ...
      4.57352, 7.16501, 78.64960934409585, ...
      13834722.5801401374, 124.688684161089762, 5228093.177931598, ...
      -0.56879356755666463, -0.56918731952397221, -9919582785894.853;
      -87.85331, 85.66836, -65.120313040242748, ...
      66.48646, 16.09921, -4.888658719272296, ...
      17286615.3147144645, 155.58592449699137, 2635887.4729110181, ...
      -0.90697975771398578, -0.91095608883042767, 42667211366919.534;
      1.74708, 128.32011, -101.584843631173858, ...
      -11.16617, 11.87109, -86.325793296437476, ...
      12942901.1241347408, 116.650512484301857, 5682744.8413270572, ...
      -0.44857868222697644, -0.44824490340007729, 10763055294345.653;
      -25.72959, -144.90758, -153.647468693117198, ...
      -57.70581, -269.17879, -48.343983158876487, ...
      9413446.7452453107, 84.664533838404295, 6356176.6898881281, ...
      0.09492245755254703, 0.09737058264766572, 74515122850712.444;
      -41.22777, 122.32875, 14.285113402275739, ...
      -7.57291, 130.37946, 10.805303085187369, ...
      3812686.035106021, 34.34330804743883, 3588703.8812128856, ...
      0.82605222593217889, 0.82572158200920196, -2456961531057.857;
      11.01307, 138.25278, 79.43682622782374, ...
      6.62726, 247.05981, 103.708090215522657, ...
      11911190.819018408, 107.341669954114577, 6070904.722786735, ...
      -0.29767608923657404, -0.29785143390252321, 17121631423099.696;
      -29.47124, 95.14681, -163.779130441688382, ...
      -27.46601, -69.15955, -15.909335945554969, ...
      13487015.8381145492, 121.294026715742277, 5481428.9945736388, ...
      -0.51527225545373252, -0.51556587964721788, 104679964020340.318];

  lat1 = testcases(:,1); lon1 = testcases(:,2); azi1 = testcases(:,3);
  lat2 = testcases(:,4); lon2 = testcases(:,5); azi2 = testcases(:,6);
  s12 = testcases(:,7); a12 = testcases(:,8); m12 = testcases(:,9);
  M12 = testcases(:,10); M21 = testcases(:,11); S12 = testcases(:,12);
  [s12a, azi1a, azi2a, S12a, m12a, M12a, M21a, a12a] = ...
      geoddistance(lat1, lon1, lat2, lon2);
  n = n + assertEquals(azi1, azi1a, 1e-13);
  n = n + assertEquals(azi2, azi2a, 1e-13);
  n = n + assertEquals(s12, s12a, 1e-8);
  n = n + assertEquals(a12, a12a, 1e-13);
  n = n + assertEquals(m12, m12a, 1e-8);
  n = n + assertEquals(M12, M12a, 1e-15);
  n = n + assertEquals(M21, M21a, 1e-15);
  n = n + assertEquals(S12, S12a, 0.1);

  [lat2a, lon2a, azi2a, S12a, m12a, M12a, M21a, a12a] = ...
      geodreckon(lat1, lon1, s12, azi1, 2);
  n = n + assertEquals(lat2, lat2a, 1e-13);
  n = n + assertEquals(lon2, lon2a, 1e-13);
  n = n + assertEquals(azi2, azi2a, 1e-13);
  n = n + assertEquals(a12, a12a, 1e-13);
  n = n + assertEquals(m12, m12a, 1e-8);
  n = n + assertEquals(M12, M12a, 1e-15);
  n = n + assertEquals(M21, M21a, 1e-15);
  n = n + assertEquals(S12, S12a, 0.1);

  [lat2a, lon2a, azi2a, S12a, m12a, M12a, M21a, s12a] = ...
      geodreckon(lat1, lon1, a12, azi1, 1+2);
  n = n + assertEquals(lat2, lat2a, 1e-13);
  n = n + assertEquals(lon2, lon2a, 1e-13);
  n = n + assertEquals(azi2, azi2a, 1e-13);
  n = n + assertEquals(s12, s12a, 1e-8);
  n = n + assertEquals(m12, m12a, 1e-8);
  n = n + assertEquals(M12, M12a, 1e-15);
  n = n + assertEquals(M21, M21a, 1e-15);
  n = n + assertEquals(S12, S12a, 0.1);
end

function ell = ellipsoid(a, f)
  ell = [a, flat2ecc(f)];
end

function n = GeoConvert0
  n = 0;
  [x, y, zone, isnorth] = utmups_fwd(33.3, 44.4);
  mgrs = mgrs_fwd(x, y, zone, isnorth, 2);
  n = n + ~strcmp(mgrs, '38SMB4484');
end

function n = GeoConvert8
% Check fix to PolarStereographic es initialization blunder (2015-05-18)
  n = 0;
  [x, y, zone, isnorth] = utmups_fwd(86, 0);
  n = n + ~(zone == 0);
  n = n + ~(isnorth);
  n = n + assertEquals(x, 2000000, 0.5e-6);
  n = n + assertEquals(y, 1555731.570643, 0.5e-6);
end

function n = GeoConvert16
% Check MGRS::Forward improved rounding fix, 2015-07-22
  n = 0;
  mgrs = mgrs_fwd(444140.6, 3684706.3, 38, true, 8);
  n = n + ~strcmp(mgrs, '38SMB4414060084706300');
end

function n = GeoConvert17
% Check MGRS::Forward digit consistency fix, 2015-07-23
  n = 0;
  mgrs = mgrs_fwd(500000, 63.811, 38, true, 8);
  n = n + ~strcmp(mgrs, '38NNF0000000000063811');
  mgrs = mgrs_fwd(500000, 63.811, 38, true, 9);
  n = n + ~strcmp(mgrs, '38NNF000000000000638110');
end

function n = GeodSolve0
  n = 0;
  [s12, azi1, azi2] = geoddistance(40.6, -73.8, 49.01666667, 2.55);
  n = n + assertEquals(azi1, 53.47022, 0.5e-5);
  n = n + assertEquals(azi2, 111.59367, 0.5e-5);
  n = n + assertEquals(s12, 5853226, 0.5);
end

function n = GeodSolve1
  n = 0;
  [lat2, lon2, azi2] = geodreckon(40.63972222, -73.77888889, 5850e3, 53.5);
  n = n + assertEquals(lat2, 49.01467, 0.5e-5);
  n = n + assertEquals(lon2, 2.56106, 0.5e-5);
  n = n + assertEquals(azi2, 111.62947, 0.5e-5);
end

function n = GeodSolve2
% Check fix for antipodal prolate bug found 2010-09-04
  n = 0;
  ell = ellipsoid(6.4e6, -1/150.0);
  [s12, azi1, azi2] = geoddistance(0.07476, 0, -0.07476, 180, ell);
  n = n + assertEquals(azi1, 90.00078, 0.5e-5);
  n = n + assertEquals(azi2, 90.00078, 0.5e-5);
  n = n + assertEquals(s12, 20106193, 0.5);
  [s12, azi1, azi2] = geoddistance(0.1, 0, -0.1, 180, ell);
  n = n + assertEquals(azi1, 90.00105, 0.5e-5);
  n = n + assertEquals(azi2, 90.00105, 0.5e-5);
  n = n + assertEquals(s12, 20106193, 0.5);
end

function n = GeodSolve4
% Check fix for short line bug found 2010-05-21
% This also checks the MATLAB specific bug:
% Ensure that Lengths in geoddistance is not invoked with zero-length
% vectors, 2015-08-25.
  n = 0;
  s12 = geoddistance(36.493349428792, 0, 36.49334942879201, .0000008);
  n = n + assertEquals(s12, 0.072, 0.5e-3);
end

function n = GeodSolve5
% Check fix for point2=pole bug found 2010-05-03
  n = 0;
  [lat2, lon2, azi2] = geodreckon(0.01777745589997, 30, 10e6, 0);
  n = n + assertEquals(lat2, 90, 0.5e-5);
  if lon2 < 0
    n = n + assertEquals(lon2, -150, 0.5e-5);
    n = n + assertEquals(abs(azi2), 180, 0.5e-5);
  else
    n = n + assertEquals(lon2, 30, 0.5e-5);
    n = n + assertEquals(azi2, 0, 0.5e-5);
  end
end

function n = GeodSolve6
% Check fix for volatile sbet12a bug found 2011-06-25 (gcc 4.4.4
% x86 -O3).  Found again on 2012-03-27 with tdm-mingw32 (g++ 4.6.1).
  n = 0;
  s12 = geoddistance(88.202499451857, 0, ...
                     -88.202499451857, 179.981022032992859592);
  n = n + assertEquals(s12, 20003898.214, 0.5e-3);
  s12 = geoddistance(89.262080389218, 0, ...
                     -89.262080389218, 179.992207982775375662);
  n = n + assertEquals(s12, 20003925.854, 0.5e-3);
  s12 = geoddistance(89.333123580033, 0, ...
                     -89.333123580032997687, 179.99295812360148422);
  n = n + assertEquals(s12, 20003926.881, 0.5e-3);
end

function n = GeodSolve9
% Check fix for volatile x bug found 2011-06-25 (gcc 4.4.4 x86 -O3)
  n = 0;
  s12 = geoddistance(56.320923501171, 0, ...
                     -56.320923501171, 179.664747671772880215);
  n = n + assertEquals(s12, 19993558.287, 0.5e-3);
end

function n = GeodSolve10
% Check fix for adjust tol1_ bug found 2011-06-25 (Visual Studio
% 10 rel + debug)
  n = 0;
  s12 = geoddistance(52.784459512564, 0, ...
                     -52.784459512563990912, 179.634407464943777557);
  n = n + assertEquals(s12, 19991596.095, 0.5e-3);
end

function n = GeodSolve11
% Check fix for bet2 = -bet1 bug found 2011-06-25 (Visual Studio
% 10 rel + debug)
  n = 0;
  s12 = geoddistance(48.522876735459, 0, ...
                     -48.52287673545898293, 179.599720456223079643);
  n = n + assertEquals(s12, 19989144.774, 0.5e-3);
end

function n = GeodSolve12
% Check fix for inverse geodesics on extreme prolate/oblate
% ellipsoids Reported 2012-08-29 Stefan Guenther
% <stefan.gunther@embl.de>; fixed 2012-10-07
  n = 0;
  ell = ellipsoid(89.8, -1.83);
  [s12, azi1, azi2] = geoddistance(0, 0, -10, 160, ell);
  n = n + assertEquals(azi1, 120.27, 1e-2);
  n = n + assertEquals(azi2, 105.15, 1e-2);
  n = n + assertEquals(s12, 266.7, 1e-1);
end

function n = GeodSolve14
% Check fix for inverse ignoring lon12 = nan
  n = 0;
  [s12, azi1, azi2] = geoddistance(0, 0, 1, NaN);
  n = n + assertNaN(azi1);
  n = n + assertNaN(azi2);
  n = n + assertNaN(s12);
end

function n = GeodSolve15
% Initial implementation of Math::eatanhe was wrong for e^2 < 0.  This
% checks that this is fixed.
  n = 0;
  ell = ellipsoid(6.4e6, -1/150.0);
  [~, ~, ~, S12] = geodreckon(1, 2, 4, 3, ell);
  n = n + assertEquals(S12, 23700, 0.5);
end

function n = GeodSolve17
% Check fix for LONG_UNROLL bug found on 2015-05-07
  n = 0;
  [lat2, lon2, azi2] = geodreckon(40, -75, 2e7, -10, 2);
  n = n + assertEquals(lat2, -39, 1);
  n = n + assertEquals(lon2, -254, 1);
  n = n + assertEquals(azi2, -170, 1);
  [lat2, lon2, azi2] = geodreckon(40, -75, 2e7, -10);
  n = n + assertEquals(lat2, -39, 1);
  n = n + assertEquals(lon2, 105, 1);
  n = n + assertEquals(azi2, -170, 1);
end

function n = GeodSolve26
% Check 0/0 problem with area calculation on sphere 2015-09-08
  n = 0;
  ell = ellipsoid(6.4e6, 0);
  [~, ~, ~, S12] = geoddistance(1, 2, 3, 4, ell);
  n = n + assertEquals(S12, 49911046115.0, 0.5);
end

function n = GeodSolve28
% Check for bad placement of assignment of r.a12 with |f| > 0.01 (bug in
% Java implementation fixed on 2015-05-19).
  n = 0;
  ell = ellipsoid(6.4e6, 0.1);
  [~, ~, ~, ~, ~, ~, ~, a12] = geodreckon(1, 2, 5e6, 10, ell);
  n = n + assertEquals(a12, 48.55570690, 0.5e-8);
end

function n = GeodSolve33
% Check max(-0.0,+0.0) issues 2015-08-22 (triggered by bugs in Octave --
% sind(-0.0) = +0.0 -- and in some version of Visual Studio --
% fmod(-0.0, 360.0) = +0.0.
  n = 0;
  [s12, azi1, azi2] = geoddistance(0, 0, 0, 179);
  n = n + assertEquals(azi1, 90.00000, 0.5e-5);
  n = n + assertEquals(azi2, 90.00000, 0.5e-5);
  n = n + assertEquals(s12, 19926189, 0.5);
  [s12, azi1, azi2] = geoddistance(0, 0, 0, 179.5);
  n = n + assertEquals(azi1, 55.96650, 0.5e-5);
  n = n + assertEquals(azi2, 124.03350, 0.5e-5);
  n = n + assertEquals(s12, 19980862, 0.5);
  [s12, azi1, azi2] = geoddistance(0, 0, 0, 180);
  n = n + assertEquals(azi1, 0.00000, 0.5e-5);
  n = n + assertEquals(abs(azi2), 180.00000, 0.5e-5);
  n = n + assertEquals(s12, 20003931, 0.5);
  [s12, azi1, azi2] = geoddistance(0, 0, 1, 180);
  n = n + assertEquals(azi1, 0.00000, 0.5e-5);
  n = n + assertEquals(abs(azi2), 180.00000, 0.5e-5);
  n = n + assertEquals(s12, 19893357, 0.5);
  ell = ellipsoid(6.4e6, 0);
  [s12, azi1, azi2] = geoddistance(0, 0, 0, 179, ell);
  n = n + assertEquals(azi1, 90.00000, 0.5e-5);
  n = n + assertEquals(azi2, 90.00000, 0.5e-5);
  n = n + assertEquals(s12, 19994492, 0.5);
  [s12, azi1, azi2] = geoddistance(0, 0, 0, 180, ell);
  n = n + assertEquals(azi1, 0.00000, 0.5e-5);
  n = n + assertEquals(abs(azi2), 180.00000, 0.5e-5);
  n = n + assertEquals(s12, 20106193, 0.5);
  [s12, azi1, azi2] = geoddistance(0, 0, 1, 180, ell);
  n = n + assertEquals(azi1, 0.00000, 0.5e-5);
  n = n + assertEquals(abs(azi2), 180.00000, 0.5e-5);
  n = n + assertEquals(s12, 19994492, 0.5);
  ell = ellipsoid(6.4e6, -1/300.0);
  [s12, azi1, azi2] = geoddistance(0, 0, 0, 179, ell);
  n = n + assertEquals(azi1, 90.00000, 0.5e-5);
  n = n + assertEquals(azi2, 90.00000, 0.5e-5);
  n = n + assertEquals(s12, 19994492, 0.5);
  [s12, azi1, azi2] = geoddistance(0, 0, 0, 180, ell);
  n = n + assertEquals(azi1, 90.00000, 0.5e-5);
  n = n + assertEquals(azi2, 90.00000, 0.5e-5);
  n = n + assertEquals(s12, 20106193, 0.5);
  [s12, azi1, azi2] = geoddistance(0, 0, 0.5, 180, ell);
  n = n + assertEquals(azi1, 33.02493, 0.5e-5);
  n = n + assertEquals(azi2, 146.97364, 0.5e-5);
  n = n + assertEquals(s12, 20082617, 0.5);
  [s12, azi1, azi2] = geoddistance(0, 0, 1, 180, ell);
  n = n + assertEquals(azi1, 0.00000, 0.5e-5);
  n = n + assertEquals(abs(azi2), 180.00000, 0.5e-5);
  n = n + assertEquals(s12, 20027270, 0.5);
  % Check also octave-specific versions of this problem.
  % In 1.44 this returned [-2.0004e+07, -2.0004e+07, 0.0000e+00, 0.0000e+00]
  s12 = geoddistance(0,0,0,[179.5, 179.6, 180, 180]);
  n = n + assertEquals(s12, [19980862, 19989165, 20003931, 20003931], 0.5);
end

function n = GeodSolve55
% Check fix for nan + point on equator or pole not returning all nans in
% Geodesic::Inverse, found 2015-09-23.
  n = 0;
  [s12, azi1, azi2] = geoddistance(NaN, 0, 0, 90);
  n = n + assertNaN(azi1);
  n = n + assertNaN(azi2);
  n = n + assertNaN(s12);
  [s12, azi1, azi2] = geoddistance(NaN, 0, 90, 9);
  n = n + assertNaN(azi1);
  n = n + assertNaN(azi2);
  n = n + assertNaN(s12);
end

function n = GeodSolve59
% Check for points close with longitudes close to 180 deg apart.
  n = 0;
  [s12, azi1, azi2] = geoddistance(5, 0.00000000000001, 10, 180);
  n = n + assertEquals(azi1, 0.000000000000035, 1.5e-14);
  n = n + assertEquals(azi2, 179.99999999999996, 1.5e-14);
  n = n + assertEquals(s12, 18345191.174332713, 2.5e-9);
end

function n = GeodSolve61
% Make sure small negative azimuths are west-going
  n = 0;
  [lat2, lon2, azi2] = geodreckon(45, 0, 1e7, -0.000000000000000003, 2);
  n = n + assertEquals(lat2, 45.30632, 0.5e-5);
  n = n + assertEquals(lon2, -180, 0.5e-5);
  n = n + assertEquals(abs(azi2), 180, 0.5e-5);
end

function n = GeodSolve73
% Check for backwards from the pole bug reported by Anon on 2016-02-13.
% This only affected the Java implementation.  It was introduced in Java
% version 1.44 and fixed in 1.46-SNAPSHOT on 2016-01-17.
  n = 0;
  [lat2, lon2, azi2] = geodreckon(90, 10, -1e6, 180);
  n = n + assertEquals(lat2, 81.04623, 0.5e-5);
  n = n + assertEquals(lon2, -170, 0.5e-5);
  n = n + assertEquals(azi2, 0, 0.5e-5);
end

function n = GeodSolve74
% Check fix for inaccurate areas, bug introduced in v1.46, fixed
% 2015-10-16.
  n = 0;
  [s12, azi1, azi2, S12, m12, M12, M21, a12] = ...
      geoddistance(54.1589, 15.3872, 54.1591, 15.3877);
  n = n + assertEquals(azi1, 55.723110355, 5e-9);
  n = n + assertEquals(azi2, 55.723515675, 5e-9);
  n = n + assertEquals(s12,  39.527686385, 5e-9);
  n = n + assertEquals(a12,   0.000355495, 5e-9);
  n = n + assertEquals(m12,  39.527686385, 5e-9);
  n = n + assertEquals(M12,   0.999999995, 5e-9);
  n = n + assertEquals(M21,   0.999999995, 5e-9);
  n = n + assertEquals(S12, 286698586.30197, 5e-4);
end

function n = GeodSolve76
% The distance from Wellington and Salamanca (a classic failure of
% Vincenty)
  n = 0;
  [s12, azi1, azi2] = ...
      geoddistance(-(41+19/60), 174+49/60, 40+58/60, -(5+30/60));
  n = n + assertEquals(azi1, 160.39137649664, 0.5e-11);
  n = n + assertEquals(azi2,  19.50042925176, 0.5e-11);
  n = n + assertEquals(s12,  19960543.857179, 0.5e-6);
end

function n = GeodSolve78
% An example where the NGS calculator fails to converge
  n = 0;
  [s12, azi1, azi2] = geoddistance(27.2, 0.0, -27.1, 179.5);
  n = n + assertEquals(azi1,  45.82468716758, 0.5e-11);
  n = n + assertEquals(azi2, 134.22776532670, 0.5e-11);
  n = n + assertEquals(s12,  19974354.765767, 0.5e-6);
end

function n = Planimeter0
% Check fix for pole-encircling bug found 2011-03-16
  n = 0;
  pa = [89, 0; 89, 90; 89, 180; 89, 270];
  pb = [-89, 0; -89, 90; -89, 180; -89, 270];
  pc = [0, -1; -1, 0; 0, 1; 1, 0];
  pd = [90, 0; 0, 0; 0, 90];

  [area, perimeter] = geodarea(pa(:,1), pa(:,2));
  n = n + assertEquals(perimeter, 631819.8745, 1e-4);
  n = n + assertEquals(area, 24952305678.0, 1);

  [area, perimeter] = geodarea(pb(:,1), pb(:,2));
  n = n + assertEquals(perimeter, 631819.8745, 1e-4);
  n = n + assertEquals(area, -24952305678.0, 1);

  [area, perimeter] = geodarea(pc(:,1), pc(:,2));
  n = n + assertEquals(perimeter, 627598.2731, 1e-4);
  n = n + assertEquals(area, 24619419146.0, 1);

  [area, perimeter] = geodarea(pd(:,1), pd(:,2));
  n = n + assertEquals(perimeter, 30022685, 1);
  n = n + assertEquals(area, 63758202715511.0, 1);

end

function n = Planimeter5
% Check fix for Planimeter pole crossing bug found 2011-06-24
  n = 0;
  points = [89, 0.1; 89, 90.1; 89, -179.9];
  [area, perimeter] = geodarea(points(:,1), points(:,2));
  n = n + assertEquals(perimeter, 539297, 1);
  n = n + assertEquals(area, 12476152838.5, 1);
end

function n = Planimeter6
% Check fix for Planimeter lon12 rounding bug found 2012-12-03
  n = 0;
  pa = [9, -0.00000000000001; 9, 180; 9, 0];
  pb = [9, 0.00000000000001; 9, 0; 9, 180];
  pc = [9, 0.00000000000001; 9, 180; 9, 0];
  pd = [9, -0.00000000000001; 9, 0; 9, 180];

  [area, perimeter] = geodarea(pa(:,1), pa(:,2));
  n = n + assertEquals(perimeter, 36026861, 1);
  n = n + assertEquals(area, 0, 1);
  [area, perimeter] = geodarea(pb(:,1), pb(:,2));
  n = n + assertEquals(perimeter, 36026861, 1);
  n = n + assertEquals(area, 0, 1);
  [area, perimeter] = geodarea(pc(:,1), pc(:,2));
  n = n + assertEquals(perimeter, 36026861, 1);
  n = n + assertEquals(area, 0, 1);
  [area, perimeter] = geodarea(pd(:,1), pd(:,2));
  n = n + assertEquals(perimeter, 36026861, 1);
  n = n + assertEquals(area, 0, 1);
end

function n = Planimeter12
% Area of arctic circle (not really -- adjunct to rhumb-area test)
  n = 0;
  points = [66.562222222, 0; 66.562222222, 180];
  [area, perimeter] = geodarea(points(:,1), points(:,2));
  n = n + assertEquals(perimeter, 10465729, 1);
  n = n + assertEquals(area, 0, 1);
end

function n = Planimeter13
% Check encircling pole twice
  n = 0;
  points = [89,-360; 89,-240; 89,-120; 89,0; 89,120; 89,240];
  [area, perimeter] = geodarea(points(:,1), points(:,2));
  n = n + assertEquals(perimeter, 1160741, 1);
  n = n + assertEquals(area, 32415230256.0, 1);
end

function n = TransverseMercatorProj1
% Test fix to bad meridian convergence at pole with
% TransverseMercatorExact found 2013-06-26
  n = 0;
  [x, y, gam, k] = tranmerc_fwd(0, 0, 90, 75);
  n = n + assertEquals(x, 0, 0.5e-6);
  n = n + assertEquals(y, 10001965.72935, 0.5e-4);
  n = n + assertEquals(gam, 75, 0.5e-12);
  n = n + assertEquals(k, 1, 0.5e-12);
end

function n = TransverseMercatorProj3
% Test fix to bad scale at pole with TransverseMercatorExact
% found 2013-06-30 (quarter meridian = 10001965.7293127228128889202m)
  n = 0;
  [lat, lon, gam, k] = tranmerc_inv(0, 0, 0, 10001965.7293127228);
  n = n + assertEquals(lat, 90, 1e-11);
  if abs(lon) < 90
    n = n + assertEquals(lon, 0, 0.5e-12);
    n = n + assertEquals(gam, 0, 0.5e-12);
  else
    n = n + assertEquals(abs(lon), 180, 0.5e-12);
    n = n + assertEquals(abs(gam), 180, 0.5e-12);
  end
  n = n + assertEquals(k, 1.0, 0.5e-12);
end

function n = TransverseMercatorProj5
% Generic tests for transverse Mercator added 2017-04-15 to check use of
% complex arithmetic to do Clenshaw sum.
  n = 0;
  k0 = 0.9996;
  ell = ellipsoid(6.4e6, 1/150);
  [x, y, gam, k] = tranmerc_fwd(0, 0, 20, 30, ell);
  n = n + assertEquals(x * k0, 3266035.453860, 0.5e-6);
  n = n + assertEquals(y * k0, 2518371.552676, 0.5e-6);
  n = n + assertEquals(gam, 11.207356502141, 0.5e-12);
  n = n + assertEquals(k * k0, 1.134138960741, 0.5e-12);
  [lat, lon, gam, k] = tranmerc_inv(0, 0, 3.3e6 / k0, 2.5e6 / k0, ell);
  n = n + assertEquals(lat, 19.80370996793, 0.5e-11);
  n = n + assertEquals(lon, 30.24919702282, 0.5e-11);
  n = n + assertEquals(gam, 11.214378172893, 0.5e-12);
  n = n + assertEquals(k * k0, 1.137025775759, 0.5e-12);
end

function n = geodreckon0
% Check mixed array size bugs
  n = 0;
  % lat1 is an array, azi1 is a scalar: 2015-08-10
  [~, ~, ~, S12] = geodreckon([10 20], 0, 0, 0);
  if length(S12) ~= 2, n = n+1; end
  % scalar args except s12 is empty: 2017-03-26
  [~, ~, ~, S12] = geodreckon(10, 0, [], 0);
  if ~isempty(S12), n = n+1; end
  % atan2dx needs to accommodate scalar + array arguments: 2017-03-27
  lat2 = geodreckon(3, 4, [1, 2], 90);
  if length(lat2) ~= 2, n = n+1; end
end

function n = gedistance0
% gedistance(0, 0, 0, 100) wrongly return nan; 2015-09-23
  n = 0;
  s12 = gedistance(0, 0, 0, 100);
  n = n + assertEquals(s12, 11131949, 0.5);
end

function n = tranmerc0
% In 1.44, tranmerc_{fwd,inv} didn't work with array arguments.
  n = 0;
  % This used to result in an error
  [x, y, gam, k] = tranmerc_fwd(0, 0, [90,90;85,85], [10,20;10,20]);
  k0 = 0.9996;
  n = n + assertEquals(x, [0, 0; 96820.412637, 190740.935334]/k0, 0.5e-6);
  n = n + assertEquals(y, [9997964.943021, 9997964.943021;  ...
                      9448171.516284, 9473242.646190]/k0, 0.5e-6);
  n = n + assertEquals(gam, [10, 20; ...
                      9.962710901776, 19.929896900550], 0.5e-12);
  n = n + assertEquals(k, [0.9996, 0.9996; ...
                      0.999714504947, 1.000044424775]/k0, 0.5e-12);
  % This used to result in NaNs
  [x, y, gam, k] = tranmerc_fwd(0, 0, [90,90;85,85]', [10,20;10,20]');
  k0 = 0.9996;
  n = n + assertEquals(x, [0, 0; 96820.412637, 190740.935334]'/k0, 0.5e-6);
  n = n + assertEquals(y, [9997964.943021, 9997964.943021;  ...
                      9448171.516284, 9473242.646190]'/k0, 0.5e-6);
  n = n + assertEquals(gam, [10, 20; ...
                      9.962710901776, 19.929896900550]', 0.5e-12);
  n = n + assertEquals(k, [0.9996, 0.9996; ...
                      0.999714504947, 1.000044424775]'/k0, 0.5e-12);
end

function n = mgrs0
% In 1.43, mgrs_inv didn't detect illegal letter combinations.
  n = 0;
  % This used to result in finite x, y
  [x, y, zone, northp] = mgrs_inv('38RMB');
  n = n + assertNaN(x) + assertNaN(y);
  n = n + assertEquals(zone, -4, 0);
  n = n + assertEquals(northp, false, 0);
end

function n = mgrs1
% In 1.44, mgrs_fwd gives the wrong results with prec = 10 or 11 in octave
  n = 0;
  % This used to result in '38SMB-1539607552-1539607552'
  mgrs = mgrs_fwd(450000, 3650000, 38, 1, 11);
  n = n + assertEquals(mgrs{1}, '38SMB5000000000050000000000', 0);
end

function n = mgrs2
% In 1.43, mgrs_inv doesn't decode prec 11 string correctly
  n = 0;
  % This used to result in x = y = NaN
  [x, y, zone, northp] = mgrs_inv('38SMB5000000000050000000000');
  n = n + assertEquals(x, 450000.0000005, 0.5e-6);
  n = n + assertEquals(y, 3650000.0000005, 0.5e-6);
  n = n + assertEquals(zone, 38, 0);
  n = n + assertEquals(northp, true, 0);
end

function n = mgrs3
% GeoConvert16: Check MGRS::Forward improved rounding fix, 2015-07-22
  n = 0;
  mgrs = mgrs_fwd(444140.6, 3684706.3, 38, 1, 8);
  n = n + assertEquals(mgrs{1}, '38SMB4414060084706300', 0);
end

function n = mgrs4
% GeoConvert17: Check MGRS::Forward digit consistency fix, 2015-07-23
  n = 0;
  mgrs = mgrs_fwd(500000, 63.811, 38, 1, 8);
  n = n + assertEquals(mgrs{1}, '38NNF0000000000063811', 0);
  mgrs = mgrs_fwd(500000, 63.811, 38, 1, 9);
  n = n + assertEquals(mgrs{1}, '38NNF000000000000638110', 0);
end
