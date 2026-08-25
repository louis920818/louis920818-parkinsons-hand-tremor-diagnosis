#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run on the device VM: read PADS raw timeseries, build a feature table using the feature functions from pads_sensor_only.py, and output a small CSV.
Usage: python3 extract_on_device.py <ROOT (containing movement/timeseries and patients)> <output CSV>"""
import os, sys, glob, json, collections
import numpy as np, pandas as pd
from numpy.fft import rfft, rfftfreq

ROOT = sys.argv[1]; OUT = sys.argv[2]
TS = os.path.join(ROOT, "movement", "timeseries")
FS = 100; BAND = (2, 12); TREM = (3, 7); WR = ["LeftWrist", "RightWrist"]
KIN    = ["PointFinger", "TouchNose"]
EXTRA2 = ["RelaxedTask", "LiftHold", "CrossArms", "TouchIndex", "Entrainment"]
NSEG = 4; SEG_OVERLAP = 0.5; TRIM_SEC = 0.5

def load(path):
    try: d = pd.read_csv(path, header=None).values
    except Exception: return None
    k = int(TRIM_SEC * FS)
    return d[k:] if d.shape[0] > k + 64 else d

def bp(P, fr, lo, hi):
    m = (fr >= lo) & (fr <= hi); return P[m].sum()

def rec_feats(path):
    d = load(path)
    if d is None or d.shape[0] < 256: return None
    acc = d[:, 1:4]; gyr = d[:, 4:7]; lin = acc - acc.mean(0)
    N = lin.shape[0]; w = np.hanning(N); fr = rfftfreq(N, 1 / FS)
    P = np.zeros_like(fr)
    for a in range(3):
        F = rfft(lin[:, a] * w); P += F.real ** 2 + F.imag ** 2
    band = (fr >= BAND[0]) & (fr <= BAND[1]); trem = (fr >= TREM[0]) & (fr <= TREM[1])
    Pb = P[band]; fb = fr[band]
    if Pb.sum() <= 0: return None
    peak = float(fb[np.argmax(Pb)])
    bratio = float(P[trem].sum() / (Pb.sum() + 1e-12))
    bpow = float(np.log10(P[trem].sum() + 1e-12))
    rms = float(np.sqrt((lin ** 2).sum(1).mean()))
    grms = float(np.sqrt((gyr ** 2).sum(1).mean()))
    pp = Pb / (Pb.sum() + 1e-12)
    spec_ent = float(-(pp * np.log(pp + 1e-12)).sum() / np.log(len(pp)))
    peak_sharp = float(np.log10(Pb.max() / (np.median(Pb) + 1e-12) + 1))
    lohi = float(bp(P, fr, 3, 5) / (P[trem].sum() + 1e-12))
    Pg = np.zeros_like(fr)
    for a in range(3):
        Fg = rfft((gyr[:, a] - gyr[:, a].mean()) * w); Pg += Fg.real ** 2 + Fg.imag ** 2
    g_bratio = float(Pg[trem].sum() / (Pg[band].sum() + 1e-12))
    peaks = []; win = N // NSEG
    step = max(1, int(round(win * (1 - SEG_OVERLAP))))
    for s in range(0, N - win + 1, step):
        x = lin[s:s + win]
        if x.shape[0] < 64: continue
        ww = np.hanning(x.shape[0]); ff = rfftfreq(x.shape[0], 1 / FS); qp = np.zeros_like(ff)
        for a in range(3):
            Fs = rfft(x[:, a] * ww); qp += Fs.real ** 2 + Fs.imag ** 2
        bb = (ff >= BAND[0]) & (ff <= BAND[1])
        if qp[bb].sum() > 0: peaks.append(ff[bb][np.argmax(qp[bb])])
    freq_stab = float(np.std(peaks)) if len(peaks) >= 2 else np.nan
    m = np.sqrt((lin ** 2).sum(1)); m = m - m.mean()
    fftv = np.fft.rfft(m, n=2 * N); ac = np.fft.irfft(fftv * np.conj(fftv))[:N]; ac = ac / (ac[0] + 1e-12)
    lo_lag, hi_lag = max(1, int(FS / 7)), int(FS / 3)
    ac_reg = float(ac[lo_lag:hi_lag + 1].max()) if (hi_lag > lo_lag and hi_lag < len(ac)) else np.nan
    jerk = float(np.sqrt((np.diff(lin, axis=0) ** 2).sum(1).mean()) * FS / (rms + 1e-9))
    return dict(peak=peak, bratio=bratio, bpow=bpow, rms=rms, grms=grms,
               spec_ent=spec_ent, peak_sharp=peak_sharp, lohi=lohi,
               g_bratio=g_bratio, freq_stab=freq_stab, ac_reg=ac_reg, jerk=jerk)

BASE_KEYS = ["peak", "bratio", "bpow", "rms", "grms"]
RICH_KEYS = BASE_KEYS + ["spec_ent", "peak_sharp", "lohi", "g_bratio", "freq_stab", "ac_reg", "jerk"]
REP_KEYS  = ["rep_rate", "rep_reg", "rep_dec", "kin_bratio"]

def rep_feats(path):
    d = load(path)
    if d is None or d.shape[0] < 256: return None
    lin = d[:, 1:4] - d[:, 1:4].mean(0); N = lin.shape[0]
    w = np.hanning(N); fr = rfftfreq(N, 1 / FS)
    mag = np.sqrt((lin ** 2).sum(1)); env = mag - mag.mean()
    Fe = rfft(env * w); Pe = Fe.real ** 2 + Fe.imag ** 2
    mv = (fr >= 0.3) & (fr <= 3.0)
    if Pe[mv].sum() <= 0: return None
    rep_rate = float(fr[mv][np.argmax(Pe[mv])])
    rep_reg  = float(np.log10(Pe[mv].max() / (np.median(Pe[mv]) + 1e-12) + 1))
    seg = N // 6; amps = [np.ptp(mag[s * seg:(s + 1) * seg]) for s in range(6) if mag[s * seg:(s + 1) * seg].size]
    rep_dec = float(np.polyfit(np.arange(len(amps)), amps, 1)[0] / (np.mean(amps) + 1e-9)) if len(amps) >= 3 else np.nan
    band = (fr >= BAND[0]) & (fr <= BAND[1]); trem = (fr >= TREM[0]) & (fr <= TREM[1])
    Pl = np.zeros_like(fr)
    for ax in range(3):
        Fa = rfft((lin[:, ax] - lin[:, ax].mean()) * w); Pl += Fa.real ** 2 + Fa.imag ** 2
    kin_bratio = float(Pl[trem].sum() / (Pl[band].sum() + 1e-12))
    return dict(rep_rate=rep_rate, rep_reg=rep_reg, rep_dec=rep_dec, kin_bratio=kin_bratio)

def subject_row(sid):
    strong = {}; row = {}
    for task in ["Relaxed", "StretchHold", "HoldWeight", "DrinkGlas"]:
        fw = {wr: rec_feats(os.path.join(TS, f"{sid}_{task}_{wr}.txt")) for wr in WR}
        avail = [f for f in fw.values() if f]
        if not avail: strong[task] = None; continue
        st = max(avail, key=lambda f: f["bpow"]); strong[task] = st
        if task in ("Relaxed", "StretchHold"):
            for k in RICH_KEYS: row[f"{task}_{k}"] = st[k]
            if fw["LeftWrist"] and fw["RightWrist"]:
                row[f"{task}_asym_bpow"] = abs(fw["LeftWrist"]["bpow"] - fw["RightWrist"]["bpow"])
        else:
            for k in ["bpow", "bratio", "spec_ent"]: row[f"{task}_{k}"] = st[k]
    if strong.get("Relaxed") is None or strong.get("StretchHold") is None:
        return None
    rest, post = strong["Relaxed"], strong["StretchHold"]
    hw, act = strong.get("HoldWeight"), strong.get("DrinkGlas")
    row["rp_bpow"] = rest["bpow"] - post["bpow"]
    row["rp_bratio"] = rest["bratio"] - post["bratio"]
    row["rp_peak"] = rest["peak"] - post["peak"]
    row["rest_dom"] = rest["bpow"] - max(x["bpow"] for x in [post, hw, act] if x)
    if act: row["ra_bpow"] = rest["bpow"] - act["bpow"]
    kin_best = {}
    for task in KIN:
        cand = [rep_feats(os.path.join(TS, f"{sid}_{task}_{wr}.txt")) for wr in WR]
        cand = [c for c in cand if c]
        if not cand: continue
        kf = max(cand, key=lambda c: c["rep_reg"]); kin_best[task] = kf
        for k in REP_KEYS: row[f"{task}_{k}"] = kf[k]
    if kin_best:
        row["rest_vs_kin"] = rest["bratio"] - max(k["kin_bratio"] for k in kin_best.values())
    for task in EXTRA2:
        fw = {wr: rec_feats(os.path.join(TS, f"{sid}_{task}_{wr}.txt")) for wr in WR}
        avail = [f for f in fw.values() if f]
        if not avail: continue
        st = max(avail, key=lambda f: f["bpow"])
        for k in ("bpow", "bratio", "spec_ent", "peak"): row[f"{task}_{k}"] = st[k]
    return row

labmap = {}
for f in glob.glob(os.path.join(ROOT, "patients", "patient_*.json")):
    d = json.load(open(f)); labmap[d["id"]] = d.get("condition", "?")
subs = sorted(labmap)
start = int(sys.argv[3]) if len(sys.argv) > 3 else 0
end   = int(sys.argv[4]) if len(sys.argv) > 4 else len(subs)
rows, ys, ids = [], [], []
for i, sid in enumerate(subs[start:end]):
    r = subject_row(sid)
    if r is None: continue
    rows.append(r); ys.append(labmap[sid]); ids.append(sid)
    if (i + 1) % 50 == 0: print("done", start + i + 1, flush=True)
df = pd.DataFrame(rows, index=ids); df.insert(0, "label", ys)
df.to_csv(OUT, encoding="utf-8")
print(f"SAVED {OUT} rows={len(df)} cols={df.shape[1]-1} dist={dict(collections.Counter(ys))}", flush=True)
