"""GNSS front-end helpers for the RTK / PPP-RTK GTSAM examples.

Hides the GNSS plumbing (RINEX / QZSS-CLAS decoding, satellite states,
observation extraction) and the LAMBDA covariance bridge, so the example
notebook can focus on building the GTSAM factor graph -- the counterpart of
``gnss_utils.py`` for these examples.

The raw-measurement front-end is installed with::

    pip install git+https://github.com/inuex35/cssrlib-numba.git@gtsam-gnss-frontend
"""
import os
from copy import deepcopy
from binascii import unhexlify
from types import SimpleNamespace

import numpy as np
import gtsam

import cssrlib.rinex as rn
import cssrlib.gnss as gn
from cssrlib.gnss import (rSigRnx, uTYP, sat2prn, uGNSS, ecef2pos, ecef2enu,
                          Nav, time2gpst, time2doy, epoch2time)
from cssrlib.gnss import geodist as _geodist
from cssrlib.rtk import rtkpos


def _P3(v):
    return gtsam.Point3(float(v[0]), float(v[1]), float(v[2]))


# --------------------------------------------------------------------------- #
# RTK (double difference): rover + base, data bundled with the cssrlib package
# --------------------------------------------------------------------------- #
def load_rtk(n_epochs=60):
    """Load the bundled rover/base RINEX and return per-epoch DD measurements.

    Returns a namespace with: ``frames`` (list of cssrlib DD bundles),
    ``ref_of`` (reference satellite per constellation), ``rb`` (base ECEF),
    ``x0`` (rover seed), ``xyz_ref``/``pos_ref`` (surveyed marker), ``nf``,
    ``syss`` and ``engine`` (the rtkpos instance, used by
    :func:`resolve_integer_ambiguities`).
    """
    bdir = os.path.join(os.path.dirname(gn.__file__), "data") + os.sep
    syss = (uGNSS.GPS, uGNSS.GAL)
    xyz_ref = np.array([-3962108.673, 3381309.574, 3668678.638])

    sigs = [rSigRnx("GC1C"), rSigRnx("GC2W"), rSigRnx("GL1C"), rSigRnx("GL2W"),
            rSigRnx("GS1C"), rSigRnx("GS2W"),
            rSigRnx("EC1C"), rSigRnx("EC5Q"), rSigRnx("EL1C"), rSigRnx("EL5Q"),
            rSigRnx("ES1C"), rSigRnx("ES5Q")]
    sigsb = [rSigRnx("GC1C"), rSigRnx("GC2W"), rSigRnx("GL1C"), rSigRnx("GL2W"),
             rSigRnx("GS1C"), rSigRnx("GS2W"),
             rSigRnx("EC1X"), rSigRnx("EC5X"), rSigRnx("EL1X"), rSigRnx("EL5X"),
             rSigRnx("ES1X"), rSigRnx("ES5X")]

    dec = rn.rnxdec(); dec.setSignals(sigs)
    nav = Nav(); dec.decode_nav(bdir + "SEPT078M.21P", nav)
    decb = rn.rnxdec(); decb.setSignals(sigsb)
    decb.decode_obsh(bdir + "3034078M1.21O"); dec.decode_obsh(bdir + "SEPT078M1.21O")
    nav.rb = [-3959400.631, 3385704.533, 3667523.111]
    rtk = rtkpos(nav, dec.pos)
    nav.x[0:3] = np.array(dec.pos)            # seed so qcedit computes elevations

    frames = []
    sync = rn.sync_obs_hold(dec, decb, maxage=nav.maxtdiff)
    for ne, (obs, obsb, dt) in enumerate(sync):
        if ne >= n_epochs:
            break
        if obsb is None:
            continue
        dd = rtk.prepare_double_difference_measurements(obs, obsb, pos_pred=dec.pos)
        if dd is not None:
            frames.append((obs, obsb, dd))

    # reference satellite per constellation = max cumulative elevation
    el_cum = {}
    for (_, _, dd) in frames:
        for k, s in enumerate(dd.sat):
            if dd.el[k] > 0:
                el_cum[int(s)] = el_cum.get(int(s), 0.0) + dd.el[k]
    ref_of = {}
    for s, e in el_cum.items():
        sys = sat2prn(s)[0]
        if sys in syss and (sys not in ref_of or e > el_cum[ref_of[sys]]):
            ref_of[sys] = s

    return SimpleNamespace(
        frames=frames, ref_of=ref_of, rb=np.array(nav.rb), x0=np.array(dec.pos),
        xyz_ref=xyz_ref, pos_ref=ecef2pos(xyz_ref), nf=nav.nf, syss=syss,
        engine=rtk)


def dd_observations(frame, data):
    """Yield ready-to-use double differences for one epoch.

    Each item carries the integer-ambiguity bookkeeping plus argument tuples to
    splat straight into the GTSAM factors:
      ``DoubleDifferencePseudorangeFactor(pos, *pr_args, noise)``
      ``DoubleDifferenceCarrierPhaseFactor(pos, ambRef, ambTgt, *cp_args, lam, noise)``
    """
    obs, obsb, dd = frame
    by_sys = {}
    for k, s in enumerate(dd.sat):
        by_sys.setdefault(sat2prn(int(s))[0], []).append(k)
    for sys, ks in by_sys.items():
        ref = data.ref_of.get(sys)
        ridx = next((k for k in ks if int(dd.sat[k]) == ref), None)
        if ridx is None:
            continue
        for f in range(data.nf):
            lam = obs.sig[sys][uTYP.L][f].wavelength()
            pr_rr, pr_br = obs.P[dd.iu[ridx], f], obsb.P[dd.ir[ridx], f]
            cp_rr = obs.L[dd.iu[ridx], f] * lam
            cp_br = obsb.L[dd.ir[ridx], f] * lam
            if 0.0 in (pr_rr, pr_br, cp_rr, cp_br):
                continue
            rs_ref, rsb_ref = dd.rs[dd.iu[ridx], :3], dd.rsb[dd.ir[ridx], :3]
            sd_ref = ((cp_rr - cp_br) - (pr_rr - pr_br)) / lam
            for k in ks:
                if k == ridx:
                    continue
                tgt = int(dd.sat[k])
                pr_tr, pr_tb = obs.P[dd.iu[k], f], obsb.P[dd.ir[k], f]
                cp_tr = obs.L[dd.iu[k], f] * lam
                cp_tb = obsb.L[dd.ir[k], f] * lam
                if 0.0 in (pr_tr, pr_tb, cp_tr, cp_tb):
                    continue
                rs_j, rsb_j = dd.rs[dd.iu[k], :3], dd.rsb[dd.ir[k], :3]
                w = 1.0 / max(np.sin(min(dd.el[k], dd.el[ridx])), 0.1)
                geom = (_P3(rs_ref), _P3(rs_j), _P3(rsb_ref), _P3(rsb_j),
                        _P3(data.rb))
                yield SimpleNamespace(
                    ref=ref, tgt=tgt, f=f, lam=lam, w=w,
                    sd_ref=sd_ref, sd_tgt=((cp_tr - cp_tb) - (pr_tr - pr_tb)) / lam,
                    pr_args=(pr_rr, pr_br, pr_tr, pr_tb, *geom),
                    cp_args=(cp_rr, cp_br, cp_tr, cp_tb, *geom))


# --------------------------------------------------------------------------- #
# PPP-RTK (undifferenced): single receiver + QZSS CLAS (downloaded data)
# --------------------------------------------------------------------------- #
def load_ppp(datadir, n_epochs=120, min_epoch=20):
    """Decode QZSS CLAS and return per-epoch undifferenced PPP measurements.

    Returns a namespace with ``frames`` (cssrlib PPP bundles), ``x0`` (a seed
    ~7 m off truth), ``xyz_ref``/``pos_ref``, ``nf``, ``syss``, ``ztd_sig`` and
    ``engine`` (the ppprtkpos instance).
    """
    from cssrlib.cssrlib import cssr
    from cssrlib.peph import atxdec, searchpcv
    from cssrlib.ppprtk import ppprtkpos
    from cssrlib.rinex import rnxdec

    syss = (uGNSS.GPS, uGNSS.GAL, uGNSS.QZS)
    xyz_ref = np.array([-3962108.7007, 3381309.5532, 3668678.6648])
    ep = [2025, 8, 21, 7, 0, 0]
    time = epoch2time(ep)
    doy = int(time2doy(time))
    let = chr(ord("a") + ep[3])
    bdir = f"{datadir}/doy{ep[0]:04d}-{doy:03d}/"

    sigs = [rSigRnx("GC1C"), rSigRnx("GC2W"), rSigRnx("EC1C"), rSigRnx("EC5Q"),
            rSigRnx("JC1C"), rSigRnx("JC2L"),
            rSigRnx("GL1C"), rSigRnx("GL2W"), rSigRnx("EL1C"), rSigRnx("EL5Q"),
            rSigRnx("JL1C"), rSigRnx("JL2L"),
            rSigRnx("GS1C"), rSigRnx("GS2W"), rSigRnx("ES1C"), rSigRnx("ES5Q"),
            rSigRnx("JS1C"), rSigRnx("JS2L")]
    nav = Nav(); nav = rnxdec().decode_nav(bdir + f"{doy:03d}{let}_rnx.nav", nav)
    atx = atxdec(); atx.readpcv(f"{datadir}/antex/igs20.atx")
    rnx = rnxdec(); rnx.setSignals(sigs)
    cs = cssr(); cs.monlevel = 0; cs.week = time2gpst(time)[0]
    cs.read_griddef(f"{datadir}/clas_grid.def")
    assert rnx.decode_obsh(bdir + f"{doy:03d}{let}_rnx.obs") >= 0
    rnx.autoSubstituteSignals()
    ppp = ppprtkpos(nav, rnx.pos)
    nav.rcv_ant = searchpcv(atx.pcvr, rnx.ant, rnx.ts); nav.sat_ant = atx.pcvs
    cs.find_grid_index(ecef2pos(rnx.pos))

    v = np.genfromtxt(bdir + f"{doy:03d}{let}_qzsl6.txt",
                      dtype=[("wn", "int"), ("tow", "int"), ("prn", "int"),
                             ("type", "int"), ("len", "int"), ("nav", "S500")])
    frames = []
    obs = rnx.decode_obs()
    while time > obs.t and obs.t.time != 0:
        obs = rnx.decode_obs()
    for k in range(n_epochs):
        week, tow = time2gpst(obs.t)
        vi = v[(v["tow"] == tow) & (v["type"] == 0) & (v["prn"] == 199)]
        if len(vi) > 0:
            cs.decode_l6msg(unhexlify(vi["nav"][0]), 0)
            if cs.fcnt == 5:
                cs.decode_cssr(bytes(cs.buff), 0)
        if k == 0:
            nav.t = deepcopy(obs.t); t0 = deepcopy(obs.t)
            t0.time = t0.time // 30 * 30; cs.time = obs.t; nav.time_p = t0
        if cs.chk_stat():
            ppm = ppp.prepare_ppp_measurements(obs, cs=cs, pos_pred=rnx.pos)
            if ppm is not None and k >= min_epoch:
                frames.append(ppm)
        obs = rnx.decode_obs()
        if obs.t.time == 0:
            break

    ztd_sigs = [fr.ztd_sig for fr in frames if np.isfinite(fr.ztd_sig)]
    return SimpleNamespace(
        frames=frames, x0=xyz_ref + np.array([5.0, -4.0, 3.0]), xyz_ref=xyz_ref,
        pos_ref=ecef2pos(xyz_ref), nf=nav.nf, syss=syss,
        ztd_sig=float(np.median(ztd_sigs)) if ztd_sigs else 0.1, engine=ppp)


def undiff_observations(frame, data):
    """Yield ready-to-use undifferenced observations for one PPP epoch.

    Each item carries the satellite/clock/iono bookkeeping plus argument tuples:
      ``UndifferencedPseudorangeFactor(pos, clk, ztd, iono, *pr_args, noise)``
      ``UndifferencedCarrierPhaseFactor(pos, clk, ztd, iono, amb, *cp_args, noise)``
    Only satellites with both frequencies present (so the slant iono is
    observable) are returned.
    """
    fr = frame
    rr = fr.pos_pred
    nf = data.nf
    for i, s in enumerate(fr.sat):
        s = int(s); sys = sat2prn(s)[0]
        if sys not in data.syss or fr.el[i] <= 0:
            continue
        if not np.all(np.isfinite(fr.rs[i])) or np.linalg.norm(fr.rs[i]) < 1e6:
            continue
        if not (fr.y[i, 0] != 0 and fr.y[i, 1] != 0
                and fr.y[i, nf] != 0 and fr.y[i, nf + 1] != 0):
            continue
        geom, _ = _geodist(fr.rs[i], rr)
        s_el = 1.0 / max(np.sin(fr.el[i]), 0.1)
        sat_pos = _P3(fr.rs[i])
        for f in range(nf):
            lam, mu = fr.lam[i, f], fr.mu[i, f]
            if lam <= 0 or mu <= 0 or fr.y[i, f] == 0 or fr.y[i, nf + f] == 0:
                continue
            m_phase = fr.y[i, f] + geom
            m_code = fr.y[i, nf + f] + geom
            iono_sig = min(fr.iono_sig[i] if np.isfinite(fr.iono_sig[i])
                           else 0.05, 0.01)
            yield SimpleNamespace(
                sat=s, sys=sys, f=f, lam=lam, s_el=s_el, iono_sig=iono_sig,
                clock_init=float(m_code - geom),
                amb_init=float((m_phase - geom - (m_code - geom)) / lam),
                pr_args=(m_code, sat_pos, fr.mapfw[i], mu, 0.0),
                cp_args=(m_phase, sat_pos, fr.mapfw[i], mu, lam, 0.0))


# --------------------------------------------------------------------------- #
# Doppler (range rate): single receiver, same open-sky RINEX as the PPP example
# --------------------------------------------------------------------------- #
def load_doppler(observation_path, navigation_path=None, reference_ecef=None,
                 n_epochs=120, elmask_deg=15.0):
    """Load RINEX files and return per-epoch Doppler measurements.

    ``observation_path`` and ``navigation_path`` can point to any static-receiver
    RINEX observation and broadcast-navigation files. For compatibility with
    this example, omitting ``navigation_path`` treats ``observation_path`` as
    the cssrlib-data root and selects the open-sky day 233 files automatically.
    ``reference_ecef`` defaults to the RINEX header position for custom files.

    Returns a namespace with ``frames`` (list of per-epoch namespaces holding
    the observation time and the per-satellite records), ``xyz_ref``/``pos_ref``
    (the surveyed static marker, so the true velocity is zero) and ``syss``.
    """
    from cssrlib.ephemeris import satposs
    from cssrlib.gnss import geodist, satazel
    from cssrlib.rinex import rnxdec

    syss = (uGNSS.GPS, uGNSS.GAL, uGNSS.QZS)
    elmask = np.deg2rad(elmask_deg)
    if navigation_path is None:
        bdir = f"{observation_path}/doy2025-233/"
        observation_path = bdir + "233h_rnx.obs"
        navigation_path = bdir + "233h_rnx.nav"
        if reference_ecef is None:
            reference_ecef = [-3962108.7007, 3381309.5532, 3668678.6648]

    sigs = [rSigRnx(f"{c}{t}1{a}") for c, a in (("G", "C"), ("E", "C"), ("J", "C"))
            for t in "CLDS"]
    nav = rnxdec().decode_nav(str(navigation_path), Nav())
    rnx = rnxdec()
    rnx.setSignals(sigs)
    assert rnx.decode_obsh(str(observation_path)) >= 0
    rnx.autoSubstituteSignals()
    xyz_ref = np.asarray(rnx.pos if reference_ecef is None else reference_ecef,
                         dtype=float)
    pos_ref = ecef2pos(xyz_ref)

    frames = []
    for _ in range(n_epochs):
        obs = rnx.decode_obs()
        if obs.t.time == 0:
            break
        rs, vs, dts, svh, _ = satposs(obs, nav)
        sats = {}
        for i, s in enumerate(obs.sat):
            s = int(s)
            sys = sat2prn(s)[0]
            if sys not in syss or svh[i] != 0:
                continue
            if obs.D[i, 0] == 0.0 or obs.P[i, 0] == 0.0:
                continue
            if not np.all(np.isfinite(rs[i])) or np.linalg.norm(rs[i]) < 1e6:
                continue
            _, e = geodist(rs[i], xyz_ref)
            _, el = satazel(pos_ref, e)
            if el < elmask:
                continue
            sats[s] = SimpleNamespace(
                sat=s, sys=sys, doppler=obs.D[i, 0], el=el, los=e,
                lam=obs.sig[sys][uTYP.L][0].wavelength(),
                sat_pos=rs[i].copy(), sat_vel=vs[i].copy(), sat_clk=dts[i])
        frames.append(SimpleNamespace(t=obs.t, sats=sats))

    return SimpleNamespace(frames=frames, xyz_ref=xyz_ref, pos_ref=pos_ref,
                           syss=syss)


def doppler_observations(previous, current, data):
    """Yield ready-to-use Doppler observations for one pair of epochs.

    Only satellites seen at both epochs are returned, because the receiver
    clock drift is modelled as the difference of the two clock-bias states and
    the satellite clock drift is differenced from the same pair.  The argument
    tuples splat straight into the two factors::

        DopplerFactor(vel, biasPrev, biasCurr, *o.meas_args, *o.epoch_args, noise)
        DopplerFactorArm(pose, vel, biasPrev, biasCurr, *o.meas_args,
                         leverArm, angularVelocity, *o.epoch_args, noise)
    """
    from cssrlib.gnss import timediff

    dt = timediff(current.t, previous.t)
    for s, o in current.sats.items():
        if s not in previous.sats:
            continue
        sat_drift = (o.sat_clk - previous.sats[s].sat_clk) / dt
        yield SimpleNamespace(
            sat=s, sys=o.sys, el=o.el, los=o.los, lam=o.lam, dt=dt,
            w=1.0 / max(np.sin(o.el), 0.1),
            meas_args=(o.doppler, o.lam, _P3(o.sat_pos), _P3(o.sat_vel),
                       _P3(data.xyz_ref)),
            epoch_args=(dt, sat_drift))


def save_doppler_data(data, path):
    """Save factor-ready Doppler arrays for use without cssrlib."""
    from cssrlib.gnss import timediff

    records, offsets = [], [0]
    epoch_dt = [0.0]
    for previous, current in zip(data.frames[:-1], data.frames[1:]):
        records.extend(doppler_observations(previous, current, data))
        offsets.append(len(records))
        epoch_dt.append(timediff(current.t, previous.t))

    ecef_R_enu = np.column_stack(
        [ecef2enu(data.pos_ref, axis) for axis in np.eye(3)])
    np.savez_compressed(
        path,
        receiver_ecef=data.xyz_ref,
        ecef_R_enu=ecef_R_enu,
        epoch_dt=np.asarray(epoch_dt),
        epoch_offsets=np.asarray(offsets, dtype=np.int32),
        satellite=np.asarray([o.sat for o in records], dtype=np.int16),
        doppler=np.asarray([o.meas_args[0] for o in records]),
        elevation=np.asarray([o.el for o in records]),
        wavelength=np.asarray([o.lam for o in records]),
        satellite_position=np.asarray([o.meas_args[2] for o in records]),
        satellite_velocity=np.asarray([o.meas_args[3] for o in records]),
        satellite_clock_drift=np.asarray([o.epoch_args[1] for o in records]),
    )


# --------------------------------------------------------------------------- #
# Integer ambiguity resolution (LAMBDA) on a GTSAM float estimate
# --------------------------------------------------------------------------- #
def resolve_integer_ambiguities(engine, isam, res, x_key, amb_key, sats, el,
                                seen_am, nf, syss,
                                conv_sigma=None, elmaskar_deg=15.0):
    """Fix the integer ambiguities with cssrlib's LAMBDA on the ISAM2 float.

    GTSAM only estimates the *float* (continuous, real-valued) ambiguities; the
    integer fixing is a separate step performed here, outside the factor graph.
    This routine writes those float ambiguities and their joint covariance into
    the cssrlib ``nav`` state, then calls ``resamb_lambda`` (the LAMBDA integer
    search). The full position+ambiguity joint marginal is ill-conditioned, so
    the covariance is assembled from the ambiguity-only joint (stable) plus
    pairwise (position, ambiguity) cross terms, with a non-finite guard.
    ``conv_sigma`` gates the resolution until the position 1-sigma is small
    enough (PPP); leave ``None`` for RTK.

    Returns ``(nb, fixed_xyz)`` -- number of fixed SD ambiguities and the fixed
    ECEF position, or ``(0, None)`` if AR was skipped / not accepted.
    """
    nav = engine.nav
    nav.x[nav.na:] = 0.0
    nav.P[:, :] = 0.0
    nav.vsat[:, :] = 0
    nav.x[0:3] = np.array(res.atPoint3(x_key))

    amb = [(int(s), f) for s in sats for f in range(nf)
           if sat2prn(int(s))[0] in syss and amb_key(int(s), f) in seen_am
           and res.exists(amb_key(int(s), f))]
    if len(amb) < 4:
        return 0, None

    p_pos = isam.marginalCovariance(x_key)
    if conv_sigma is not None and np.sqrt(np.trace(p_pos)) > conv_sigma:
        return 0, None

    el_now = {int(s): el[i] for i, s in enumerate(sats)}
    for (s_, f) in amb:
        j = engine.IB(s_, f, nav.na)
        nav.x[j] = res.atDouble(amb_key(s_, f))
        nav.vsat[s_ - 1, f] = 1
        if s_ in el_now:
            nav.el[s_ - 1] = el_now[s_]

    nav.P[0:3, 0:3] = p_pos
    kv = gtsam.KeyVector([amb_key(s_, f) for (s_, f) in amb])
    jm = isam.jointMarginalCovariance(kv)
    for (s_, f) in amb:
        j = engine.IB(s_, f, nav.na)
        nav.P[j, j] = jm.at(amb_key(s_, f), amb_key(s_, f))[0, 0]
        pxn = isam.jointMarginalCovariance(
            gtsam.KeyVector([x_key, amb_key(s_, f)])).at(x_key, amb_key(s_, f))[:, 0]
        nav.P[0:3, j] = pxn
        nav.P[j, 0:3] = pxn
    for a in range(len(amb)):
        s1, f1 = amb[a]; j1 = engine.IB(s1, f1, nav.na)
        for b in range(a + 1, len(amb)):
            s2, f2 = amb[b]; j2 = engine.IB(s2, f2, nav.na)
            c = jm.at(amb_key(s1, f1), amb_key(s2, f2))[0, 0]
            nav.P[j1, j2] = c
            nav.P[j2, j1] = c

    bad = ~np.isfinite(nav.P)
    if bad.any():
        nav.P[bad] = 0.0
        d = np.where(np.diag(bad))[0]
        nav.P[d, d] = 1e10

    nav.elmaskar = np.deg2rad(elmaskar_deg)
    sat_ar = np.array(sorted({s_ for (s_, f) in amb}))
    nb, _ = engine.resamb_lambda(sat_ar, nav.parmode, nav.par_P0)
    return nb, (np.array(nav.xa[0:3]) if nb > 0 else None)


def enu_error(pos_ref, xyz_ref, xyz):
    """2D and 3D error [m] of an ECEF estimate against the surveyed marker."""
    enu = ecef2enu(pos_ref, np.asarray(xyz) - xyz_ref)
    return float(np.hypot(enu[0], enu[1])), float(np.linalg.norm(np.asarray(xyz) - xyz_ref))
