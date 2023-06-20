"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Sim3 unit tests.
Author: John Lambert
"""
# pylint: disable=no-name-in-module
import unittest
from typing import List, Optional

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import Point3, Pose3, Rot3, Similarity3


class TestSim3(GtsamTestCase):
    """Test selected Sim3 methods."""

    def test_align_poses_along_straight_line(self):
        """Test Align Pose3Pairs method.

        Scenario:
           3 object poses
           same scale (no gauge ambiguity)
           world frame has poses rotated about x-axis (90 degree roll)
           world and egovehicle frame translated by 15 meters w.r.t. each other
        """
        Rx90 = Rot3.Rx(np.deg2rad(90))

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose3(Rot3(), np.array([5, 0, 0]))
        eTo1 = Pose3(Rot3(), np.array([10, 0, 0]))
        eTo2 = Pose3(Rot3(), np.array([15, 0, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world/city "w" frame)
        wTo0 = Pose3(Rx90, np.array([-10, 0, 0]))
        wTo1 = Pose3(Rx90, np.array([-5, 0, 0]))
        wTo2 = Pose3(Rx90, np.array([0, 0, 0]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = gtsam.Pose3Pairs(list(zip(wToi_list, eToi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity3.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses_along_straight_line_gauge(self):
        """Test if Align Pose3Pairs method can account for gauge ambiguity.

        Scenario:
           3 object poses
           with gauge ambiguity (2x scale)
           world frame has poses rotated about z-axis (90 degree yaw)
           world and egovehicle frame translated by 11 meters w.r.t. each other
        """
        Rz90 = Rot3.Rz(np.deg2rad(90))

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose3(Rot3(), np.array([1, 0, 0]))
        eTo1 = Pose3(Rot3(), np.array([2, 0, 0]))
        eTo2 = Pose3(Rot3(), np.array([4, 0, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world/city "w" frame)
        wTo0 = Pose3(Rz90, np.array([0, 12, 0]))
        wTo1 = Pose3(Rz90, np.array([0, 14, 0]))
        wTo2 = Pose3(Rz90, np.array([0, 18, 0]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = gtsam.Pose3Pairs(list(zip(wToi_list, eToi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity3.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses_scaled_squares(self):
        """Test if Align Pose3Pairs method can account for gauge ambiguity.

        Make sure a big and small square can be aligned.
        The u's represent a big square (10x10), and v's represents a small square (4x4).

        Scenario:
           4 object poses
           with gauge ambiguity (2.5x scale)
        """
        # 0, 90, 180, and 270 degrees yaw
        R0 = Rot3.Rz(np.deg2rad(0))
        R90 = Rot3.Rz(np.deg2rad(90))
        R180 = Rot3.Rz(np.deg2rad(180))
        R270 = Rot3.Rz(np.deg2rad(270))

        aTi0 = Pose3(R0, np.array([2, 3, 0]))
        aTi1 = Pose3(R90, np.array([12, 3, 0]))
        aTi2 = Pose3(R180, np.array([12, 13, 0]))
        aTi3 = Pose3(R270, np.array([2, 13, 0]))

        aTi_list = [aTi0, aTi1, aTi2, aTi3]

        bTi0 = Pose3(R0, np.array([4, 3, 0]))
        bTi1 = Pose3(R90, np.array([8, 3, 0]))
        bTi2 = Pose3(R180, np.array([8, 7, 0]))
        bTi3 = Pose3(R270, np.array([4, 7, 0]))

        bTi_list = [bTi0, bTi1, bTi2, bTi3]

        ab_pairs = gtsam.Pose3Pairs(list(zip(aTi_list, bTi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        aSb = Similarity3.Align(ab_pairs)

        for aTi, bTi in zip(aTi_list, bTi_list):
            self.gtsamAssertEquals(aTi, aSb.transformFrom(bTi))

    def test_align_via_Sim3_to_poses_skydio32(self) -> None:
        """Ensure scale estimate of Sim(3) object is non-negative.

        Comes from real data (from Skydio-32 Crane Mast sequence with a SIFT front-end).
        """
        poses_gt = [
            Pose3(
                Rot3(
                    [
                        [0.696305769, -0.0106830792, -0.717665705],
                        [0.00546412488, 0.999939148, -0.00958346857],
                        [0.717724415, 0.00275160848, 0.696321772],
                    ]
                ),
                Point3(5.83077801, -0.94815149, 0.397751679),
            ),
            Pose3(
                Rot3(
                    [
                        [0.692272397, -0.00529704529, -0.721616549],
                        [0.00634689669, 0.999979075, -0.00125157022],
                        [0.721608079, -0.0037136016, 0.692291531],
                    ]
                ),
                Point3(5.03853323, -0.97547405, -0.348177392),
            ),
            Pose3(
                Rot3(
                    [
                        [0.945991981, -0.00633548292, -0.324128225],
                        [0.00450436485, 0.999969379, -0.00639931046],
                        [0.324158843, 0.00459370582, 0.945991552],
                    ]
                ),
                Point3(4.13186176, -0.956364218, -0.796029527),
            ),
            Pose3(
                Rot3(
                    [
                        [0.999553623, -0.00346470207, -0.0296740626],
                        [0.00346104216, 0.999993995, -0.00017469881],
                        [0.0296744897, 7.19175654e-05, 0.999559612],
                    ]
                ),
                Point3(3.1113898, -0.928583423, -0.90539337),
            ),
            Pose3(
                Rot3(
                    [
                        [0.967850252, -0.00144846042, 0.251522892],
                        [0.000254511591, 0.999988546, 0.00477934325],
                        [-0.251526934, -0.00456167299, 0.967839535],
                    ]
                ),
                Point3(2.10584013, -0.921303194, -0.809322971),
            ),
            Pose3(
                Rot3(
                    [
                        [0.969854065, 0.000629052774, 0.243685716],
                        [0.000387180179, 0.999991428, -0.00412234326],
                        [-0.243686221, 0.00409242166, 0.969845508],
                    ]
                ),
                Point3(1.0753788, -0.913035975, -0.616584091),
            ),
            Pose3(
                Rot3(
                    [
                        [0.998189342, 0.00110235337, 0.0601400045],
                        [-0.00110890447, 0.999999382, 7.55559042e-05],
                        [-0.060139884, -0.000142108649, 0.998189948],
                    ]
                ),
                Point3(0.029993558, -0.951495122, -0.425525143),
            ),
            Pose3(
                Rot3(
                    [
                        [0.999999996, -2.62868666e-05, -8.67178281e-05],
                        [2.62791334e-05, 0.999999996, -8.91767396e-05],
                        [8.67201719e-05, 8.91744604e-05, 0.999999992],
                    ]
                ),
                Point3(-0.973569417, -0.936340994, -0.253464928),
            ),
            Pose3(
                Rot3(
                    [
                        [0.99481227, -0.00153645011, 0.101716252],
                        [0.000916919443, 0.999980747, 0.00613725239],
                        [-0.101723724, -0.00601214847, 0.994794525],
                    ]
                ),
                Point3(-2.02071256, -0.955446292, -0.240707879),
            ),
            Pose3(
                Rot3(
                    [
                        [0.89795602, -0.00978591184, 0.43997636],
                        [0.00645921401, 0.999938116, 0.00905779513],
                        [-0.440037771, -0.00529159974, 0.89796366],
                    ]
                ),
                Point3(-2.94096695, -0.939974858, 0.0934225593),
            ),
            Pose3(
                Rot3(
                    [
                        [0.726299119, -0.00916784876, 0.687318077],
                        [0.00892018672, 0.999952563, 0.0039118575],
                        [-0.687321336, 0.00328981905, 0.726346444],
                    ]
                ),
                Point3(-3.72843416, -0.897889251, 0.685129502),
            ),
            Pose3(
                Rot3(
                    [
                        [0.506756029, -0.000331706105, 0.862089858],
                        [0.00613841257, 0.999975964, -0.00322354286],
                        [-0.862068067, 0.00692541035, 0.506745885],
                    ]
                ),
                Point3(-4.3909926, -0.890883291, 1.43029524),
            ),
            Pose3(
                Rot3(
                    [
                        [0.129316352, -0.00206958814, 0.991601896],
                        [0.00515932597, 0.999985691, 0.00141424797],
                        [-0.991590634, 0.00493310721, 0.129325179],
                    ]
                ),
                Point3(-4.58510846, -0.922534227, 2.36884523),
            ),
            Pose3(
                Rot3(
                    [
                        [0.599853194, -0.00890004681, -0.800060263],
                        [0.00313716318, 0.999956608, -0.00877161373],
                        [0.800103615, 0.00275175707, 0.599855085],
                    ]
                ),
                Point3(5.71559638, 0.486863076, 0.279141372),
            ),
            Pose3(
                Rot3(
                    [
                        [0.762552447, 0.000836438681, -0.646926069],
                        [0.00211337894, 0.999990607, 0.00378404105],
                        [0.646923157, -0.00425272942, 0.762543517],
                    ]
                ),
                Point3(5.00243443, 0.513321893, -0.466921769),
            ),
            Pose3(
                Rot3(
                    [
                        [0.930381645, -0.00340164355, -0.36657678],
                        [0.00425636616, 0.999989781, 0.00152338305],
                        [0.366567852, -0.00297761145, 0.930386617],
                    ]
                ),
                Point3(4.05404984, 0.493385291, -0.827904571),
            ),
            Pose3(
                Rot3(
                    [
                        [0.999996073, -0.00278379707, -0.000323508543],
                        [0.00278790921, 0.999905063, 0.0134941517],
                        [0.000285912831, -0.0134950006, 0.999908897],
                    ]
                ),
                Point3(3.04724478, 0.491451306, -0.989571061),
            ),
            Pose3(
                Rot3(
                    [
                        [0.968578343, -0.002544616, 0.248695527],
                        [0.000806130148, 0.999974526, 0.00709200332],
                        [-0.248707238, -0.0066686795, 0.968555721],
                    ]
                ),
                Point3(2.05737869, 0.46840177, -0.546344594),
            ),
            Pose3(
                Rot3(
                    [
                        [0.968827882, 0.000182770584, 0.247734722],
                        [-0.000558107079, 0.9999988, 0.00144484904],
                        [-0.24773416, -0.00153807255, 0.968826821],
                    ]
                ),
                Point3(1.14019947, 0.469674641, -0.0491053805),
            ),
            Pose3(
                Rot3(
                    [
                        [0.991647805, 0.00197867892, 0.128960146],
                        [-0.00247518407, 0.999990129, 0.00368991165],
                        [-0.128951572, -0.00397829284, 0.991642914],
                    ]
                ),
                Point3(0.150270471, 0.457867448, 0.103628642),
            ),
            Pose3(
                Rot3(
                    [
                        [0.992244594, 0.00477781876, -0.124208847],
                        [-0.0037682125, 0.999957938, 0.00836195891],
                        [0.124243574, -0.00782906317, 0.992220862],
                    ]
                ),
                Point3(-0.937954641, 0.440532658, 0.154265069),
            ),
            Pose3(
                Rot3(
                    [
                        [0.999591078, 0.00215462857, -0.0285137564],
                        [-0.00183807224, 0.999936443, 0.0111234301],
                        [0.028535911, -0.0110664711, 0.999531507],
                    ]
                ),
                Point3(-1.95622231, 0.448914367, -0.0859439782),
            ),
            Pose3(
                Rot3(
                    [
                        [0.931835342, 0.000956922238, 0.362880212],
                        [0.000941640753, 0.99998678, -0.00505501434],
                        [-0.362880252, 0.00505214382, 0.931822122],
                    ]
                ),
                Point3(-2.85557418, 0.434739285, 0.0793777177),
            ),
            Pose3(
                Rot3(
                    [
                        [0.781615218, -0.0109886966, 0.623664238],
                        [0.00516954657, 0.999924591, 0.011139446],
                        [-0.623739616, -0.00548270158, 0.781613084],
                    ]
                ),
                Point3(-3.67524552, 0.444074681, 0.583718622),
            ),
            Pose3(
                Rot3(
                    [
                        [0.521291761, 0.00264805046, 0.853374051],
                        [0.00659087718, 0.999952868, -0.00712898365],
                        [-0.853352707, 0.00934076542, 0.521249738],
                    ]
                ),
                Point3(-4.35541796, 0.413479707, 1.31179007),
            ),
            Pose3(
                Rot3(
                    [
                        [0.320164205, -0.00890839482, 0.947319884],
                        [0.00458409304, 0.999958649, 0.007854118],
                        [-0.947350678, 0.00182799903, 0.320191803],
                    ]
                ),
                Point3(-4.71617526, 0.476674479, 2.16502998),
            ),
            Pose3(
                Rot3(
                    [
                        [0.464861609, 0.0268597443, -0.884976415],
                        [-0.00947397841, 0.999633409, 0.0253631906],
                        [0.885333239, -0.00340614699, 0.464945663],
                    ]
                ),
                Point3(6.11772094, 1.63029238, 0.491786626),
            ),
            Pose3(
                Rot3(
                    [
                        [0.691647251, 0.0216006293, -0.721912024],
                        [-0.0093228132, 0.999736395, 0.020981541],
                        [0.722174939, -0.00778156302, 0.691666308],
                    ]
                ),
                Point3(5.46912979, 1.68759322, -0.288499782),
            ),
            Pose3(
                Rot3(
                    [
                        [0.921208931, 0.00622640471, -0.389018433],
                        [-0.00686296262, 0.999976419, -0.000246683913],
                        [0.389007724, 0.00289706631, 0.92122994],
                    ]
                ),
                Point3(4.70156942, 1.72186229, -0.806181015),
            ),
            Pose3(
                Rot3(
                    [
                        [0.822397705, 0.00276497594, 0.568906142],
                        [0.00804891535, 0.999831556, -0.016494662],
                        [-0.568855921, 0.0181442503, 0.822236923],
                    ]
                ),
                Point3(-3.51368714, 1.59619714, 0.437437437),
            ),
            Pose3(
                Rot3(
                    [
                        [0.726822937, -0.00545541524, 0.686803193],
                        [0.00913794245, 0.999956756, -0.00172754968],
                        [-0.686764068, 0.00753159111, 0.726841357],
                    ]
                ),
                Point3(-4.29737821, 1.61462527, 1.11537749),
            ),
            Pose3(
                Rot3(
                    [
                        [0.402595481, 0.00697612855, 0.915351441],
                        [0.0114113638, 0.999855006, -0.0126391687],
                        [-0.915306892, 0.0155338804, 0.4024575],
                    ]
                ),
                Point3(-4.6516433, 1.6323107, 1.96579585),
            ),
        ]
        # from estimated cameras
        unaligned_pose_dict = {
            2: Pose3(
                Rot3(
                    [
                        [-0.681949, -0.568276, 0.460444],
                        [0.572389, -0.0227514, 0.819667],
                        [-0.455321, 0.822524, 0.34079],
                    ]
                ),
                Point3(-1.52189, 0.78906, -1.60608),
            ),
            4: Pose3(
                Rot3(
                    [
                        [-0.817805393, -0.575044816, 0.022755196],
                        [0.0478829397, -0.0285875849, 0.998443776],
                        [-0.573499401, 0.81762229, 0.0509139174],
                    ]
                ),
                Point3(-1.22653168, 0.686485651, -1.39294168),
            ),
            3: Pose3(
                Rot3(
                    [
                        [-0.783051568, -0.571905041, 0.244448085],
                        [0.314861464, -0.0255673164, 0.948793218],
                        [-0.536369743, 0.819921299, 0.200091385],
                    ]
                ),
                Point3(-1.37620079, 0.721408674, -1.49945316),
            ),
            5: Pose3(
                Rot3(
                    [
                        [-0.818916586, -0.572896131, 0.0341415873],
                        [0.0550548476, -0.0192038786, 0.99829864],
                        [-0.571265778, 0.819402974, 0.0472670839],
                    ]
                ),
                Point3(-1.06370243, 0.663084159, -1.27672831),
            ),
            6: Pose3(
                Rot3(
                    [
                        [-0.798825521, -0.571995242, 0.186277293],
                        [0.243311017, -0.0240196245, 0.969650869],
                        [-0.550161372, 0.819905178, 0.158360233],
                    ]
                ),
                Point3(-0.896250742, 0.640768239, -1.16984756),
            ),
            7: Pose3(
                Rot3(
                    [
                        [-0.786416666, -0.570215296, 0.237493882],
                        [0.305475635, -0.0248440676, 0.951875732],
                        [-0.536873788, 0.821119534, 0.193724669],
                    ]
                ),
                Point3(-0.740385043, 0.613956842, -1.05908579),
            ),
            8: Pose3(
                Rot3(
                    [
                        [-0.806252832, -0.57019757, 0.157578877],
                        [0.211046715, -0.0283979846, 0.977063375],
                        [-0.55264424, 0.821016617, 0.143234279],
                    ]
                ),
                Point3(-0.58333517, 0.549832698, -0.9542864),
            ),
            9: Pose3(
                Rot3(
                    [
                        [-0.821191354, -0.557772774, -0.120558255],
                        [-0.125347331, -0.0297958331, 0.991665395],
                        [-0.556716092, 0.829458703, -0.0454472483],
                    ]
                ),
                Point3(-0.436483039, 0.55003923, -0.850733187),
            ),
            21: Pose3(
                Rot3(
                    [
                        [-0.778607603, -0.575075476, 0.251114312],
                        [0.334920968, -0.0424301164, 0.941290407],
                        [-0.53065822, 0.816999316, 0.225641247],
                    ]
                ),
                Point3(-0.736735967, 0.571415247, -0.738663611),
            ),
            17: Pose3(
                Rot3(
                    [
                        [-0.818569806, -0.573904529, 0.0240221722],
                        [0.0512889176, -0.0313725422, 0.998190969],
                        [-0.572112681, 0.818321059, 0.0551155579],
                    ]
                ),
                Point3(-1.36150982, 0.724829031, -1.16055631),
            ),
            18: Pose3(
                Rot3(
                    [
                        [-0.812668105, -0.582027424, 0.0285417146],
                        [0.0570298244, -0.0306936169, 0.997900547],
                        [-0.579929436, 0.812589675, 0.0581366453],
                    ]
                ),
                Point3(-1.20484771, 0.762370343, -1.05057127),
            ),
            20: Pose3(
                Rot3(
                    [
                        [-0.748446406, -0.580905382, 0.319963926],
                        [0.416860654, -0.0368374152, 0.908223651],
                        [-0.515805363, 0.813137099, 0.269727429],
                    ]
                ),
                Point3(569.449421, -907.892555, -794.585647),
            ),
            22: Pose3(
                Rot3(
                    [
                        [-0.826878177, -0.559495019, -0.0569017041],
                        [-0.0452256802, -0.0346974602, 0.99837404],
                        [-0.560559647, 0.828107125, 0.00338702978],
                    ]
                ),
                Point3(-0.591431172, 0.55422253, -0.654656597),
            ),
            29: Pose3(
                Rot3(
                    [
                        [-0.785759779, -0.574532433, -0.229115805],
                        [-0.246020939, -0.049553424, 0.967996981],
                        [-0.567499134, 0.81698038, -0.102409921],
                    ]
                ),
                Point3(69.4916073, 240.595227, -493.278045),
            ),
            23: Pose3(
                Rot3(
                    [
                        [-0.783524382, -0.548569702, -0.291823276],
                        [-0.316457553, -0.051878563, 0.94718701],
                        [-0.534737468, 0.834493797, -0.132950906],
                    ]
                ),
                Point3(-5.93496204, 41.9304933, -3.06881633),
            ),
            10: Pose3(
                Rot3(
                    [
                        [-0.766833992, -0.537641809, -0.350580824],
                        [-0.389506676, -0.0443270797, 0.919956336],
                        [-0.510147213, 0.84200736, -0.175423563],
                    ]
                ),
                Point3(234.185458, 326.007989, -691.769777),
            ),
            30: Pose3(
                Rot3(
                    [
                        [-0.754844165, -0.559278755, -0.342662459],
                        [-0.375790683, -0.0594160018, 0.92479787],
                        [-0.537579435, 0.826847636, -0.165321923],
                    ]
                ),
                Point3(-5.93398168, 41.9107972, -3.07385081),
            ),
        }

        unaligned_pose_list = []
        for i in range(32):
            wTi = unaligned_pose_dict.get(i, None)
            unaligned_pose_list.append(wTi)
        # GT poses are the reference/target
        rSe = align_poses_sim3_ignore_missing(aTi_list=poses_gt, bTi_list=unaligned_pose_list)
        assert rSe.scale() >= 0


def align_poses_sim3_ignore_missing(aTi_list: List[Optional[Pose3]], bTi_list: List[Optional[Pose3]]) -> Similarity3:
    """Align by similarity transformation, but allow missing estimated poses in the input.

    Note: this is a wrapper for align_poses_sim3() that allows for missing poses/dropped cameras.
    This is necessary, as align_poses_sim3() requires a valid pose for every input pair.

    We force SIM(3) alignment rather than SE(3) alignment.
    We assume the two trajectories are of the exact same length.

    Args:
        aTi_list: reference poses in frame "a" which are the targets for alignment
        bTi_list: input poses which need to be aligned to frame "a"

    Returns:
        aSb: Similarity(3) object that aligns the two pose graphs.
    """
    assert len(aTi_list) == len(bTi_list)

    # only choose target poses for which there is a corresponding estimated pose
    corresponding_aTi_list = []
    valid_camera_idxs = []
    valid_bTi_list = []
    for i, bTi in enumerate(bTi_list):
        if bTi is not None:
            valid_camera_idxs.append(i)
            valid_bTi_list.append(bTi)
            corresponding_aTi_list.append(aTi_list[i])

    aSb = align_poses_sim3(aTi_list=corresponding_aTi_list, bTi_list=valid_bTi_list)
    return aSb


def align_poses_sim3(aTi_list: List[Pose3], bTi_list: List[Pose3]) -> Similarity3:
    """Align two pose graphs via similarity transformation. Note: poses cannot be missing/invalid.

    We force SIM(3) alignment rather than SE(3) alignment.
    We assume the two trajectories are of the exact same length.

    Args:
        aTi_list: reference poses in frame "a" which are the targets for alignment
        bTi_list: input poses which need to be aligned to frame "a"

    Returns:
        aSb: Similarity(3) object that aligns the two pose graphs.
    """
    n_to_align = len(aTi_list)
    assert len(aTi_list) == len(bTi_list)
    assert n_to_align >= 2, "SIM(3) alignment uses at least 2 frames"

    ab_pairs = gtsam.Pose3Pairs(list(zip(aTi_list, bTi_list)))

    aSb = Similarity3.Align(ab_pairs)

    if np.isnan(aSb.scale()) or aSb.scale() == 0:
        # we have run into a case where points have no translation between them (i.e. panorama).
        # We will first align the rotations and then align the translation by using centroids.
        # TODO: handle it in GTSAM

        # align the rotations first, so that we can find the translation between the two panoramas
        aSb = Similarity3(aSb.rotation(), np.zeros((3,)), 1.0)
        aTi_list_rot_aligned = [aSb.transformFrom(bTi) for bTi in bTi_list]

        # fit a single translation motion to the centroid
        aTi_centroid = np.array([aTi.translation() for aTi in aTi_list]).mean(axis=0)
        aTi_rot_aligned_centroid = np.array([aTi.translation() for aTi in aTi_list_rot_aligned]).mean(axis=0)

        # construct the final SIM3 transform
        aSb = Similarity3(aSb.rotation(), aTi_centroid - aTi_rot_aligned_centroid, 1.0)

    return aSb


if __name__ == "__main__":
    unittest.main()
