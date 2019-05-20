"use strict";

var assert = require("assert"),
    G = require("../geographiclib"),
    g = G.Geodesic,
    d = G.DMS,
    testcases = [
      [35.60777, -139.44815, 111.098748429560326,
       -11.17491, -69.95921, 129.289270889708762,
       8935244.5604818305, 80.50729714281974, 6273170.2055303837,
       0.16606318447386067, 0.16479116945612937, 12841384694976.432],
      [55.52454, 106.05087, 22.020059880982801,
       77.03196, 197.18234, 109.112041110671519,
       4105086.1713924406, 36.892740690445894, 3828869.3344387607,
       0.80076349608092607, 0.80101006984201008, 61674961290615.615],
      [-21.97856, 142.59065, -32.44456876433189,
       41.84138, 98.56635, -41.84359951440466,
       8394328.894657671, 75.62930491011522, 6161154.5773110616,
       0.24816339233950381, 0.24930251203627892, -6637997720646.717],
      [-66.99028, 112.2363, 173.73491240878403,
       -12.70631, 285.90344, 2.512956620913668,
       11150344.2312080241, 100.278634181155759, 6289939.5670446687,
       -0.17199490274700385, -0.17722569526345708, -121287239862139.744],
      [-17.42761, 173.34268, -159.033557661192928,
       -15.84784, 5.93557, -20.787484651536988,
       16076603.1631180673, 144.640108810286253, 3732902.1583877189,
       -0.81273638700070476, -0.81299800519154474, 97825992354058.708],
      [32.84994, 48.28919, 150.492927788121982,
       -56.28556, 202.29132, 48.113449399816759,
       16727068.9438164461, 150.565799985466607, 3147838.1910180939,
       -0.87334918086923126, -0.86505036767110637, -72445258525585.010],
      [6.96833, 52.74123, 92.581585386317712,
       -7.39675, 206.17291, 90.721692165923907,
       17102477.2496958388, 154.147366239113561, 2772035.6169917581,
       -0.89991282520302447, -0.89986892177110739, -1311796973197.995],
      [-50.56724, -16.30485, -105.439679907590164,
       -33.56571, -94.97412, -47.348547835650331,
       6455670.5118668696, 58.083719495371259, 5409150.7979815838,
       0.53053508035997263, 0.52988722644436602, 41071447902810.047],
      [-58.93002, -8.90775, 140.965397902500679,
       -8.91104, 133.13503, 19.255429433416599,
       11756066.0219864627, 105.755691241406877, 6151101.2270708536,
       -0.26548622269867183, -0.27068483874510741, -86143460552774.735],
      [-68.82867, -74.28391, 93.774347763114881,
       -50.63005, -8.36685, 34.65564085411343,
       3956936.926063544, 35.572254987389284, 3708890.9544062657,
       0.81443963736383502, 0.81420859815358342, -41845309450093.787],
      [-10.62672, -32.0898, -86.426713286747751,
       5.883, -134.31681, -80.473780971034875,
       11470869.3864563009, 103.387395634504061, 6184411.6622659713,
       -0.23138683500430237, -0.23155097622286792, 4198803992123.548],
      [-21.76221, 166.90563, 29.319421206936428,
       48.72884, 213.97627, 43.508671946410168,
       9098627.3986554915, 81.963476716121964, 6299240.9166992283,
       0.13965943368590333, 0.14152969707656796, 10024709850277.476],
      [-19.79938, -174.47484, 71.167275780171533,
       -11.99349, -154.35109, 65.589099775199228,
       2319004.8601169389, 20.896611684802389, 2267960.8703918325,
       0.93427001867125849, 0.93424887135032789, -3935477535005.785],
      [-11.95887, -116.94513, 92.712619830452549,
       4.57352, 7.16501, 78.64960934409585,
       13834722.5801401374, 124.688684161089762, 5228093.177931598,
       -0.56879356755666463, -0.56918731952397221, -9919582785894.853],
      [-87.85331, 85.66836, -65.120313040242748,
       66.48646, 16.09921, -4.888658719272296,
       17286615.3147144645, 155.58592449699137, 2635887.4729110181,
       -0.90697975771398578, -0.91095608883042767, 42667211366919.534],
      [1.74708, 128.32011, -101.584843631173858,
       -11.16617, 11.87109, -86.325793296437476,
       12942901.1241347408, 116.650512484301857, 5682744.8413270572,
       -0.44857868222697644, -0.44824490340007729, 10763055294345.653],
      [-25.72959, -144.90758, -153.647468693117198,
       -57.70581, -269.17879, -48.343983158876487,
       9413446.7452453107, 84.664533838404295, 6356176.6898881281,
       0.09492245755254703, 0.09737058264766572, 74515122850712.444],
      [-41.22777, 122.32875, 14.285113402275739,
       -7.57291, 130.37946, 10.805303085187369,
       3812686.035106021, 34.34330804743883, 3588703.8812128856,
       0.82605222593217889, 0.82572158200920196, -2456961531057.857],
      [11.01307, 138.25278, 79.43682622782374,
       6.62726, 247.05981, 103.708090215522657,
       11911190.819018408, 107.341669954114577, 6070904.722786735,
       -0.29767608923657404, -0.29785143390252321, 17121631423099.696],
      [-29.47124, 95.14681, -163.779130441688382,
       -27.46601, -69.15955, -15.909335945554969,
       13487015.8381145492, 121.294026715742277, 5481428.9945736388,
       -0.51527225545373252, -0.51556587964721788, 104679964020340.318]];

assert.approx = function(x, y, d) {
  assert(Math.abs(x-y) <= d, x + " = " + y + " +/- " + d);
};

describe("GeographicLib", function() {
  describe("GeodesicTest", function () {
    var geod = g.WGS84, i,
        check_geod_inverse, check_geod_direct, check_geod_arcdirect;

    check_geod_inverse = function(l) {
      var lat1 = l[0], lon1 = l[1], azi1 = l[2],
          lat2 = l[3], lon2 = l[4], azi2 = l[5],
          s12 = l[6], a12 = l[7], m12 = l[8],
          M12 = l[9], M21 = l[10], S12 = l[11],
          inv = geod.Inverse(lat1, lon1, lat2, G.Math.AngNormalize(lon2),
                             g.ALL | g.LONG_UNROLL);
      assert.approx(lon2, inv.lon2, 1e-13);
      assert.approx(azi1, inv.azi1, 1e-13);
      assert.approx(azi2, inv.azi2, 1e-13);
      assert.approx(s12, inv.s12, 1e-8);
      assert.approx(a12, inv.a12, 1e-13);
      assert.approx(m12, inv.m12, 1e-8);
      assert.approx(M12, inv.M12, 1e-15);
      assert.approx(M21, inv.M21, 1e-15);
      assert.approx(S12, inv.S12, 0.1);
    };

    check_geod_direct = function(l) {
      var lat1 = l[0], lon1 = l[1], azi1 = l[2],
          lat2 = l[3], lon2 = l[4], azi2 = l[5],
          s12 = l[6], a12 = l[7], m12 = l[8],
          M12 = l[9], M21 = l[10], S12 = l[11],
          dir = geod.Direct(lat1, lon1, azi1, s12, g.ALL | g.LONG_UNROLL);
      assert.approx(lat2, dir.lat2, 1e-13);
      assert.approx(lon2, dir.lon2, 1e-13);
      assert.approx(azi2, dir.azi2, 1e-13);
      assert.approx(a12, dir.a12, 1e-13);
      assert.approx(m12, dir.m12, 1e-8);
      assert.approx(M12, dir.M12, 1e-15);
      assert.approx(M21, dir.M21, 1e-15);
      assert.approx(S12, dir.S12, 0.1);
    };

    check_geod_arcdirect = function(l) {
      var lat1 = l[0], lon1 = l[1], azi1 = l[2],
          lat2 = l[3], lon2 = l[4], azi2 = l[5],
          s12 = l[6], a12 = l[7], m12 = l[8],
          M12 = l[9], M21 = l[10], S12 = l[11],
          dir = geod.ArcDirect(lat1, lon1, azi1, a12, g.ALL | g.LONG_UNROLL);
      assert.approx(lat2, dir.lat2, 1e-13);
      assert.approx(lon2, dir.lon2, 1e-13);
      assert.approx(azi2, dir.azi2, 1e-13);
      assert.approx(s12, dir.s12, 1e-8);
      assert.approx(m12, dir.m12, 1e-8);
      assert.approx(M12, dir.M12, 1e-15);
      assert.approx(M21, dir.M21, 1e-15);
      assert.approx(S12, dir.S12, 0.1);
    };

    it("check inverse", function () {
      for (i = 0; i < testcases.length; ++i) {
        check_geod_inverse(testcases[i]);
      }
    });

    it("check direct", function () {
      for (i = 0; i < testcases.length; ++i) {
        check_geod_direct(testcases[i]);
      }
    });

    it("check arcdirect", function () {
      for (i = 0; i < testcases.length; ++i) {
        check_geod_arcdirect(testcases[i]);
      }
    });

  });

  describe("DMSTest", function () {
    it("check decode", function () {
      assert.deepEqual(d.Decode("E7:33:36"), d.Decode("-7.56W"));
    });
  });

  describe("GeodesicSolve", function () {
    it("GeodSolve0", function () {
      var geod = g.WGS84,
          inv = geod.Inverse(40.6, -73.8, 49.01666667, 2.55);
      assert.approx(inv.azi1, 53.47022, 0.5e-5);
      assert.approx(inv.azi2, 111.59367, 0.5e-5);
      assert.approx(inv.s12, 5853226, 0.5);
    });

    it("GeodSolve1", function() {
      var geod = g.WGS84,
          dir = geod.Direct(40.63972222, -73.77888889, 53.5, 5850e3);
      assert.approx(dir.lat2, 49.01467, 0.5e-5);
      assert.approx(dir.lon2, 2.56106, 0.5e-5);
      assert.approx(dir.azi2, 111.62947, 0.5e-5);
    });

    it("GeodSolve2", function() {
      // Check fix for antipodal prolate bug found 2010-09-04
      var geod = new g.Geodesic(6.4e6, -1/150.0),
          inv = geod.Inverse(0.07476, 0, -0.07476, 180);
      assert.approx(inv.azi1, 90.00078, 0.5e-5);
      assert.approx(inv.azi2, 90.00078, 0.5e-5);
      assert.approx(inv.s12, 20106193, 0.5);
      inv = geod.Inverse(0.1, 0, -0.1, 180);
      assert.approx(inv.azi1, 90.00105, 0.5e-5);
      assert.approx(inv.azi2, 90.00105, 0.5e-5);
      assert.approx(inv.s12, 20106193, 0.5);
    });

    it("GeodSolve4", function() {
      // Check fix for short line bug found 2010-05-21
      var geod = g.WGS84,
          inv = geod.Inverse(36.493349428792, 0, 36.49334942879201, 0.0000008);
      assert.approx(inv.s12, 0.072, 0.5e-3);
    });

    it("GeodSolve5", function() {
      // Check fix for point2=pole bug found 2010-05-03
      var geod = g.WGS84,
          dir = geod.Direct(0.01777745589997, 30, 0, 10e6);
      assert.approx(dir.lat2, 90, 0.5e-5);
      if (dir.lon2 < 0) {
        assert.approx(dir.lon2, -150, 0.5e-5);
        assert.approx(Math.abs(dir.azi2), 180, 0.5e-5);
      } else {
        assert.approx(dir.lon2, 30, 0.5e-5);
        assert.approx(dir.azi2, 0, 0.5e-5);
      }
    });

    it("GeodSolve6", function() {
      // Check fix for volatile sbet12a bug found 2011-06-25 (gcc 4.4.4
      // x86 -O3).  Found again on 2012-03-27 with tdm-mingw32 (g++ 4.6.1).
      var geod = g.WGS84,
          inv = geod.Inverse(88.202499451857, 0,
                             -88.202499451857, 179.981022032992859592);
      assert.approx(inv.s12, 20003898.214, 0.5e-3);
      inv = geod.Inverse(89.262080389218, 0,
                         -89.262080389218, 179.992207982775375662);
      assert.approx(inv.s12, 20003925.854, 0.5e-3);
      inv = geod.Inverse(89.333123580033, 0,
                         -89.333123580032997687, 179.99295812360148422);
      assert.approx(inv.s12, 20003926.881, 0.5e-3);
    });

    it("GeodSolve9", function() {
      // Check fix for volatile x bug found 2011-06-25 (gcc 4.4.4 x86 -O3)
      var geod = g.WGS84,
          inv = geod.Inverse(56.320923501171, 0,
                             -56.320923501171, 179.664747671772880215);
      assert.approx(inv.s12, 19993558.287, 0.5e-3);
    });

    it("GeodSolve10", function() {
      // Check fix for adjust tol1_ bug found 2011-06-25 (Visual Studio
      // 10 rel + debug)
      var geod = g.WGS84,
          inv = geod.Inverse(52.784459512564, 0,
                             -52.784459512563990912, 179.634407464943777557);
      assert.approx(inv.s12, 19991596.095, 0.5e-3);
    });

    it("GeodSolve11", function() {
      // Check fix for bet2 = -bet1 bug found 2011-06-25 (Visual Studio
      // 10 rel + debug)
      var geod = g.WGS84,
          inv = geod.Inverse(48.522876735459, 0,
                             -48.52287673545898293, 179.599720456223079643);
      assert.approx(inv.s12, 19989144.774, 0.5e-3);
    });

    it("GeodSolve12", function() {
      // Check fix for inverse geodesics on extreme prolate/oblate
      // ellipsoids Reported 2012-08-29 Stefan Guenther
      // <stefan.gunther@embl.de>; fixed 2012-10-07
      var geod = new g.Geodesic(89.8, -1.83),
          inv = geod.Inverse(0, 0, -10, 160);
      assert.approx(inv.azi1, 120.27, 1e-2);
      assert.approx(inv.azi2, 105.15, 1e-2);
      assert.approx(inv.s12, 266.7, 1e-1);
    });

    it("GeodSolve14", function() {
      // Check fix for inverse ignoring lon12 = nan
      var geod = g.WGS84,
          inv = geod.Inverse(0, 0, 1, NaN);
      assert(isNaN(inv.azi1));
      assert(isNaN(inv.azi2));
      assert(isNaN(inv.s12));
    });

    it("GeodSolve15", function() {
      // Initial implementation of Math::eatanhe was wrong for e^2 < 0.  This
      // checks that this is fixed.
      var geod = new g.Geodesic(6.4e6, -1/150.0),
          dir = geod.Direct(1, 2, 3, 4, g.AREA);
      assert.approx(dir.S12, 23700, 0.5);
    });

    it("GeodSolve17", function() {
      // Check fix for LONG_UNROLL bug found on 2015-05-07
      var geod = g.WGS84,
          dir = geod.Direct(40, -75, -10, 2e7, g.LONG_UNROLL),
          line;
      assert.approx(dir.lat2, -39, 1);
      assert.approx(dir.lon2, -254, 1);
      assert.approx(dir.azi2, -170, 1);
      line = geod.Line(40, -75, -10);
      dir = line.Position(2e7, g.LONG_UNROLL);
      assert.approx(dir.lat2, -39, 1);
      assert.approx(dir.lon2, -254, 1);
      assert.approx(dir.azi2, -170, 1);
      dir = geod.Direct(40, -75, -10, 2e7);
      assert.approx(dir.lat2, -39, 1);
      assert.approx(dir.lon2, 105, 1);
      assert.approx(dir.azi2, -170, 1);
      dir = line.Position(2e7);
      assert.approx(dir.lat2, -39, 1);
      assert.approx(dir.lon2, 105, 1);
      assert.approx(dir.azi2, -170, 1);
    });

    it("GeodSolve26", function() {
      // Check 0/0 problem with area calculation on sphere 2015-09-08
      var geod = new g.Geodesic(6.4e6, 0),
          inv = geod.Inverse(1, 2, 3, 4, g.AREA);
      assert.approx(inv.S12, 49911046115.0, 0.5);
    });

    it("GeodSolve28", function() {
      // Check for bad placement of assignment of r.a12 with |f| > 0.01 (bug in
      // Java implementation fixed on 2015-05-19).
      var geod = new g.Geodesic(6.4e6, 0.1),
          dir = geod.Direct(1, 2, 10, 5e6);
      assert.approx(dir.a12, 48.55570690, 0.5e-8);
    });

    it("GeodSolve29", function() {
      // Check longitude unrolling with inverse calculation 2015-09-16
      var geod = g.WGS84,
          dir = geod.Inverse(0, 539, 0, 181);
      assert.approx(dir.lon1, 179, 1e-10);
      assert.approx(dir.lon2, -179, 1e-10);
      assert.approx(dir.s12, 222639, 0.5);
      dir = geod.Inverse(0, 539, 0, 181, g.LONG_UNROLL);
      assert.approx(dir.lon1, 539, 1e-10);
      assert.approx(dir.lon2, 541, 1e-10);
      assert.approx(dir.s12, 222639, 0.5);
    });

    it("GeodSolve33", function() {
      // Check max(-0.0,+0.0) issues 2015-08-22 (triggered by bugs in Octave --
      // sind(-0.0) = +0.0 -- and in some version of Visual Studio --
      // fmod(-0.0, 360.0) = +0.0.
      var geod = g.WGS84,
          inv = geod.Inverse(0, 0, 0, 179);
      assert.approx(inv.azi1, 90.00000, 0.5e-5);
      assert.approx(inv.azi2, 90.00000, 0.5e-5);
      assert.approx(inv.s12, 19926189, 0.5);
      inv = geod.Inverse(0, 0, 0, 179.5);
      assert.approx(inv.azi1, 55.96650, 0.5e-5);
      assert.approx(inv.azi2, 124.03350, 0.5e-5);
      assert.approx(inv.s12, 19980862, 0.5);
      inv = geod.Inverse(0, 0, 0, 180);
      assert.approx(inv.azi1, 0.00000, 0.5e-5);
      assert.approx(Math.abs(inv.azi2), 180.00000, 0.5e-5);
      assert.approx(inv.s12, 20003931, 0.5);
      inv = geod.Inverse(0, 0, 1, 180);
      assert.approx(inv.azi1, 0.00000, 0.5e-5);
      assert.approx(Math.abs(inv.azi2), 180.00000, 0.5e-5);
      assert.approx(inv.s12, 19893357, 0.5);
      geod = new g.Geodesic(6.4e6, 0);
      inv = geod.Inverse(0, 0, 0, 179);
      assert.approx(inv.azi1, 90.00000, 0.5e-5);
      assert.approx(inv.azi2, 90.00000, 0.5e-5);
      assert.approx(inv.s12, 19994492, 0.5);
      inv = geod.Inverse(0, 0, 0, 180);
      assert.approx(inv.azi1, 0.00000, 0.5e-5);
      assert.approx(Math.abs(inv.azi2), 180.00000, 0.5e-5);
      assert.approx(inv.s12, 20106193, 0.5);
      inv = geod.Inverse(0, 0, 1, 180);
      assert.approx(inv.azi1, 0.00000, 0.5e-5);
      assert.approx(Math.abs(inv.azi2), 180.00000, 0.5e-5);
      assert.approx(inv.s12, 19994492, 0.5);
      geod = new g.Geodesic(6.4e6, -1/300.0);
      inv = geod.Inverse(0, 0, 0, 179);
      assert.approx(inv.azi1, 90.00000, 0.5e-5);
      assert.approx(inv.azi2, 90.00000, 0.5e-5);
      assert.approx(inv.s12, 19994492, 0.5);
      inv = geod.Inverse(0, 0, 0, 180);
      assert.approx(inv.azi1, 90.00000, 0.5e-5);
      assert.approx(inv.azi2, 90.00000, 0.5e-5);
      assert.approx(inv.s12, 20106193, 0.5);
      inv = geod.Inverse(0, 0, 0.5, 180);
      assert.approx(inv.azi1, 33.02493, 0.5e-5);
      assert.approx(inv.azi2, 146.97364, 0.5e-5);
      assert.approx(inv.s12, 20082617, 0.5);
      inv = geod.Inverse(0, 0, 1, 180);
      assert.approx(inv.azi1, 0.00000, 0.5e-5);
      assert.approx(Math.abs(inv.azi2), 180.00000, 0.5e-5);
      assert.approx(inv.s12, 20027270, 0.5);
    });

    it("GeodSolve55", function() {
      // Check fix for nan + point on equator or pole not returning all nans in
      // Geodesic::Inverse, found 2015-09-23.
      var geod = g.WGS84,
          inv = geod.Inverse(NaN, 0, 0, 90);
      assert(isNaN(inv.azi1));
      assert(isNaN(inv.azi2));
      assert(isNaN(inv.s12));
      inv = geod.Inverse(NaN, 0, 90, 9);
      assert(isNaN(inv.azi1));
      assert(isNaN(inv.azi2));
      assert(isNaN(inv.s12));
    });

    it("GeodSolve59", function() {
      // Check for points close with longitudes close to 180 deg apart.
      var geod = g.WGS84,
          inv = geod.Inverse(5, 0.00000000000001, 10, 180);
      assert.approx(inv.azi1, 0.000000000000035, 1.5e-14);
      assert.approx(inv.azi2, 179.99999999999996, 1.5e-14);
      assert.approx(inv.s12, 18345191.174332713, 4e-9);
    });

    it("GeodSolve61", function() {
      // Make sure small negative azimuths are west-going
      var geod = g.WGS84,
          dir = geod.Direct(45, 0, -0.000000000000000003, 1e7,
                            g.LONG_UNROLL),
          line;
      assert.approx(dir.lat2, 45.30632, 0.5e-5);
      assert.approx(dir.lon2, -180, 0.5e-5);
      assert.approx(Math.abs(dir.azi2), 180, 0.5e-5);
      line = geod.InverseLine(45, 0, 80, -0.000000000000000003);
      dir = line.Position(1e7, g.LONG_UNROLL);
      assert.approx(dir.lat2, 45.30632, 0.5e-5);
      assert.approx(dir.lon2, -180, 0.5e-5);
      assert.approx(Math.abs(dir.azi2), 180, 0.5e-5);
    });

    it("GeodSolve65", function() {
      // Check for bug in east-going check in GeodesicLine (needed to check for
      // sign of 0) and sign error in area calculation due to a bogus override
      // of the code for alp12.  Found/fixed on 2015-12-19.
      var geod = g.WGS84,
          line = geod.InverseLine(30, -0.000000000000000001, -31, 180,
                                  g.ALL),
          dir = line.Position(1e7, g.ALL | g.LONG_UNROLL);
      assert.approx(dir.lat1, 30.00000  , 0.5e-5);
      assert.approx(dir.lon1, -0.00000  , 0.5e-5);
      assert.approx(Math.abs(dir.azi1), 180.00000, 0.5e-5);
      assert.approx(dir.lat2, -60.23169 , 0.5e-5);
      assert.approx(dir.lon2, -0.00000  , 0.5e-5);
      assert.approx(Math.abs(dir.azi2), 180.00000, 0.5e-5);
      assert.approx(dir.s12 , 10000000  , 0.5);
      assert.approx(dir.a12 , 90.06544  , 0.5e-5);
      assert.approx(dir.m12 , 6363636   , 0.5);
      assert.approx(dir.M12 , -0.0012834, 0.5e7);
      assert.approx(dir.M21 , 0.0013749 , 0.5e-7);
      assert.approx(dir.S12 , 0         , 0.5);
      dir = line.Position(2e7, g.ALL | g.LONG_UNROLL);
      assert.approx(dir.lat1, 30.00000  , 0.5e-5);
      assert.approx(dir.lon1, -0.00000  , 0.5e-5);
      assert.approx(Math.abs(dir.azi1), 180.00000, 0.5e-5);
      assert.approx(dir.lat2, -30.03547 , 0.5e-5);
      assert.approx(dir.lon2, -180.00000, 0.5e-5);
      assert.approx(dir.azi2, -0.00000  , 0.5e-5);
      assert.approx(dir.s12 , 20000000  , 0.5);
      assert.approx(dir.a12 , 179.96459 , 0.5e-5);
      assert.approx(dir.m12 , 54342     , 0.5);
      assert.approx(dir.M12 , -1.0045592, 0.5e7);
      assert.approx(dir.M21 , -0.9954339, 0.5e-7);
      assert.approx(dir.S12 , 127516405431022.0, 0.5);
    });

    it("GeodSolve69", function() {
      // Check for InverseLine if line is slightly west of S and that s13 is
      // correctly set.
      var geod = g.WGS84,
          line = geod.InverseLine(-5, -0.000000000000002, -10, 180),
          dir = line.Position(2e7, g.LONG_UNROLL);
      assert.approx(dir.lat2, 4.96445   , 0.5e-5);
      assert.approx(dir.lon2, -180.00000, 0.5e-5);
      assert.approx(dir.azi2, -0.00000  , 0.5e-5);
      dir = line.Position(0.5 * line.s13, g.LONG_UNROLL);
      assert.approx(dir.lat2, -87.52461 , 0.5e-5);
      assert.approx(dir.lon2, -0.00000  , 0.5e-5);
      assert.approx(dir.azi2, -180.00000, 0.5e-5);
    });

    it("GeodSolve71", function() {
      // Check that DirectLine sets s13.
      var geod = g.WGS84,
          line = geod.DirectLine(1, 2, 45, 1e7),
          dir = line.Position(0.5 * line.s13, g.LONG_UNROLL);
      assert.approx(dir.lat2, 30.92625, 0.5e-5);
      assert.approx(dir.lon2, 37.54640, 0.5e-5);
      assert.approx(dir.azi2, 55.43104, 0.5e-5);
    });

    it("GeodSolve73", function() {
      // Check for backwards from the pole bug reported by Anon on 2016-02-13.
      // This only affected the Java implementation.  It was introduced in Java
      // version 1.44 and fixed in 1.46-SNAPSHOT on 2016-01-17.
      var geod = g.WGS84,
          dir = geod.Direct(90, 10, 180, -1e6);
      assert.approx(dir.lat2, 81.04623, 0.5e-5);
      assert.approx(dir.lon2, -170, 0.5e-5);
      assert.approx(dir.azi2, 0, 0.5e-5);
    });

    it("GeodSolve74", function() {
      // Check fix for inaccurate areas, bug introduced in v1.46, fixed
      // 2015-10-16.
      var geod = g.WGS84,
          inv = geod.Inverse(54.1589, 15.3872, 54.1591, 15.3877, g.ALL);
      assert.approx(inv.azi1, 55.723110355, 5e-9);
      assert.approx(inv.azi2, 55.723515675, 5e-9);
      assert.approx(inv.s12,  39.527686385, 5e-9);
      assert.approx(inv.a12,   0.000355495, 5e-9);
      assert.approx(inv.m12,  39.527686385, 5e-9);
      assert.approx(inv.M12,   0.999999995, 5e-9);
      assert.approx(inv.M21,   0.999999995, 5e-9);
      assert.approx(inv.S12, 286698586.30197, 5e-4);
    });

    it("GeodSolve76", function() {
      // The distance from Wellington and Salamanca (a classic failure of
      // Vincenty)
      var geod = g.WGS84,
          inv = geod.Inverse(-(41+19/60.0), 174+49/60.0,
                             40+58/60.0, -(5+30/60.0));
      assert.approx(inv.azi1, 160.39137649664, 0.5e-11);
      assert.approx(inv.azi2,  19.50042925176, 0.5e-11);
      assert.approx(inv.s12,  19960543.857179, 0.5e-6);
    });

    it("GeodSolve78", function() {
      // An example where the NGS calculator fails to converge
      var geod = g.WGS84,
          inv = geod.Inverse(27.2, 0.0, -27.1, 179.5);
      assert.approx(inv.azi1,  45.82468716758, 0.5e-11);
      assert.approx(inv.azi2, 134.22776532670, 0.5e-11);
      assert.approx(inv.s12,  19974354.765767, 0.5e-6);
    });

  });

  describe("Planimeter", function () {

    var geod = g.WGS84,
        polygon = geod.Polygon(false),
        polyline = geod.Polygon(true),
        Planimeter, PolyLength;

    Planimeter = function(points) {
      var i;
      polygon.Clear();
      for (i = 0; i < points.length; ++i) {
        polygon.AddPoint(points[i][0], points[i][1]);
      }
      return polygon.Compute(false, true);
    };

    PolyLength = function(points) {
      var i;
      polyline.Clear();
      for (i = 0; i < points.length; ++i) {
        polyline.AddPoint(points[i][0], points[i][1]);
      }
      return polyline.Compute(false, true);
    };

    it("Planimeter0", function() {
      // Check fix for pole-encircling bug found 2011-03-16
      var points, a;
      points = [[89, 0], [89, 90], [89, 180], [89, 270]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 631819.8745, 1e-4);
      assert.approx(a.area, 24952305678.0, 1);

      points = [[-89, 0], [-89, 90], [-89, 180], [-89, 270]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 631819.8745, 1e-4);
      assert.approx(a.area, -24952305678.0, 1);

      points = [[0, -1], [-1, 0], [0, 1], [1, 0]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 627598.2731, 1e-4);
      assert.approx(a.area, 24619419146.0, 1);

      points = [[90, 0], [0, 0], [0, 90]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 30022685, 1);
      assert.approx(a.area, 63758202715511.0, 1);
      a = PolyLength(points);
      assert.approx(a.perimeter, 20020719, 1);
      assert(isNaN(a.area));
    });

    it("Planimeter5", function() {
      // Check fix for Planimeter pole crossing bug found 2011-06-24
      var points, a;
      points = [[89, 0.1], [89, 90.1], [89, -179.9]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 539297, 1);
      assert.approx(a.area, 12476152838.5, 1);
    });

    it("Planimeter6", function() {
      // Check fix for Planimeter lon12 rounding bug found 2012-12-03
      var points, a;
      points = [[9, -0.00000000000001], [9, 180], [9, 0]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 36026861, 1);
      assert.approx(a.area, 0, 1);
      points = [[9, 0.00000000000001], [9, 0], [9, 180]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 36026861, 1);
      assert.approx(a.area, 0, 1);
      points = [[9, 0.00000000000001], [9, 180], [9, 0]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 36026861, 1);
      assert.approx(a.area, 0, 1);
      points = [[9, -0.00000000000001], [9, 0], [9, 180]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 36026861, 1);
      assert.approx(a.area, 0, 1);
    });

    it("Planimeter12", function() {
      // Area of arctic circle (not really -- adjunct to rhumb-area test)
      var points, a;
      points = [[66.562222222, 0], [66.562222222, 180]];
      a = Planimeter(points);
      assert.approx(a.perimeter, 10465729, 1);
      assert.approx(a.area, 0, 1);
    });

    it("Planimeter13", function() {
      // Check encircling pole twice
      var points, a;
      points = [[89,-360], [89,-240], [89,-120], [89,0], [89,120], [89,240]];
      a =  Planimeter(points);
      assert.approx(a.perimeter, 1160741, 1);
      assert.approx(a.area, 32415230256.0, 1);
    });

    it("check TestEdge", function() {
      // Check fix of bugs found by threepointone, 2015-10-14
      polygon.Clear();
      polygon.AddPoint(33, 44);
      polygon.TestEdge(90, 10e3, false, true);
      polygon.AddEdge(90, 10e3, false, true);
    });

  });
});
