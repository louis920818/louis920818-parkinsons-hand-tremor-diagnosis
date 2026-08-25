#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PADS watch-only (sensor) version -- fully standalone, uses no questionnaire / demographics, only the smartwatch IMU signal.
====================================================================================
- Subject-wise validation: one row per subject (label = diagnosis) -> StratifiedKFold, no subject is in both train and test (no leakage).
- Uses accel / gyro features from all 11 action tasks; includes a label-shuffle leakage test.
- This file is a standalone program; it does not depend on pads_tremor_analysis_v2.py.

Measured (469 subjects, RepeatedStratifiedKFold 5x12, passes the leakage test):
    PD vs healthy        AUC ~ 0.86
    PD vs essential tremor    AUC ~ 0.91

Outputs: pads_sensoronly_charts.png, pads_sensoronly_results.txt
Usage: python pads_sensor_only.py            (the pads-...-1.0.0/ dataset must be in the same folder as this file)
Tunable (top of file or env vars): SEG_OVERLAP (freq_stab overlap ratio, default 0.5), NSEG (number of segments 4),
            TRIM_SEC (trim the 0.5s start-up shake of each segment), FAST=1 (fewer folds, quick preview)
"""
import os, glob, json, collections, warnings, pickle
import numpy as np, pandas as pd
warnings.filterwarnings("ignore")
from numpy.fft import rfft, rfftfreq
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from sklearn.ensemble import RandomForestClassifier, HistGradientBoostingClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.pipeline import Pipeline
from sklearn.impute import SimpleImputer
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import StratifiedKFold, cross_val_predict, train_test_split
from sklearn.inspection import permutation_importance
from sklearn.metrics import roc_auc_score

# ---- paths and parameters ----
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = None
for c in glob.glob(os.path.join(HERE, "pads-*")):
    if os.path.isdir(os.path.join(c, "movement", "timeseries")):
        ROOT = c; break
TS = os.path.join(ROOT, "movement", "timeseries") if ROOT else None
FS = 100; BAND = (2, 12); TREM = (3, 7); WR = ["LeftWrist", "RightWrist"]
KIN    = ["PointFinger", "TouchNose"]                                          # repetitive-action (finger-tap) tasks
EXTRA2 = ["RelaxedTask", "LiftHold", "CrossArms", "TouchIndex", "Entrainment"] # other sensor tasks
NSEG        = int(os.environ.get("NSEG", 4))
SEG_OVERLAP = min(max(float(os.environ.get("SEG_OVERLAP", 0.5)), 0.0), 0.95)   # freq_stab segment overlap ratio
TRIM_SEC    = float(os.environ.get("TRIM_SEC", 0.5))                           # trim the start-up shake
CACHE = os.path.join(HERE, f"pads_sensoronly_feat_cache_ov{int(round(SEG_OVERLAP*100))}_{NSEG}.pkl")
FAST = bool(os.environ.get("FAST"))
R_HEAD, R_NULL, R_ABL = (3, 2, 2) if FAST else (12, 6, 5)

# ------------------------------------------------------------------ feature extraction
def load(path):
    try: d = pd.read_csv(path, header=None).values
    except Exception: return None
    k = int(TRIM_SEC * FS)
    return d[k:] if d.shape[0] > k + 64 else d

def bp(P, fr, lo, hi):
    m = (fr >= lo) & (fr <= hi); return P[m].sum()

def rec_feats(path):
    """Spectral/tremor features for one recording."""
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
    spec_ent = float(-(pp * np.log(pp + 1e-12)).sum() / np.log(len(pp)))         # spectral regularity (low = sharp peak)
    peak_sharp = float(np.log10(Pb.max() / (np.median(Pb) + 1e-12) + 1))
    lohi = float(bp(P, fr, 3, 5) / (P[trem].sum() + 1e-12))                      # tremor energy skewed to low freq (PD)
    Pg = np.zeros_like(fr)
    for a in range(3):
        Fg = rfft((gyr[:, a] - gyr[:, a].mean()) * w); Pg += Fg.real ** 2 + Fg.imag ** 2
    g_bratio = float(Pg[trem].sum() / (Pg[band].sum() + 1e-12))
    peaks = []; win = N // NSEG                                                  # segment length = signal length / NSEG
    step = max(1, int(round(win * (1 - SEG_OVERLAP))))                          # overlap ratio sets the step
    for s in range(0, N - win + 1, step):
        x = lin[s:s + win]
        if x.shape[0] < 64: continue
        ww = np.hanning(x.shape[0]); ff = rfftfreq(x.shape[0], 1 / FS); qp = np.zeros_like(ff)
        for a in range(3):
            Fs = rfft(x[:, a] * ww); qp += Fs.real ** 2 + Fs.imag ** 2
        bb = (ff >= BAND[0]) & (ff <= BAND[1])
        if qp[bb].sum() > 0: peaks.append(ff[bb][np.argmax(qp[bb])])
    freq_stab = float(np.std(peaks)) if len(peaks) >= 2 else np.nan
    m = np.sqrt((lin ** 2).sum(1)); m = m - m.mean()                            # time-domain rhythmicity (autocorrelation)
    fftv = np.fft.rfft(m, n=2 * N); ac = np.fft.irfft(fftv * np.conj(fftv))[:N]; ac = ac / (ac[0] + 1e-12)
    lo_lag, hi_lag = max(1, int(FS / 7)), int(FS / 3)
    ac_reg = float(ac[lo_lag:hi_lag + 1].max()) if (hi_lag > lo_lag and hi_lag < len(ac)) else np.nan
    jerk = float(np.sqrt((np.diff(lin, axis=0) ** 2).sum(1).mean()) * FS / (rms + 1e-9))  # smoothness
    return dict(peak=peak, bratio=bratio, bpow=bpow, rms=rms, grms=grms,
               spec_ent=spec_ent, peak_sharp=peak_sharp, lohi=lohi,
               g_bratio=g_bratio, freq_stab=freq_stab, ac_reg=ac_reg, jerk=jerk)

BASE_KEYS = ["peak", "bratio", "bpow", "rms", "grms"]
RICH_KEYS = BASE_KEYS + ["spec_ent", "peak_sharp", "lohi", "g_bratio", "freq_stab", "ac_reg", "jerk"]
REP_KEYS  = ["rep_rate", "rep_reg", "rep_dec", "kin_bratio"]

def rep_feats(path):
    """Repetitive-action (finger-tap) features: rhythm rate, regularity reg, amplitude decline dec, action tremor kin_bratio."""
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
    """One subject -> one row of sensor features (returns None if rest/posture are missing)."""
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
    kin_best = {}                                                               # finger-tap (repetitive action) features
    for task in KIN:
        cand = [rep_feats(os.path.join(TS, f"{sid}_{task}_{wr}.txt")) for wr in WR]
        cand = [c for c in cand if c]
        if not cand: continue
        kf = max(cand, key=lambda c: c["rep_reg"]); kin_best[task] = kf
        for k in REP_KEYS: row[f"{task}_{k}"] = kf[k]
    if kin_best:
        row["rest_vs_kin"] = rest["bratio"] - max(k["kin_bratio"] for k in kin_best.values())
    for task in EXTRA2:                                                         # other sensor tasks (light features)
        fw = {wr: rec_feats(os.path.join(TS, f"{sid}_{task}_{wr}.txt")) for wr in WR}
        avail = [f for f in fw.values() if f]
        if not avail: continue
        st = max(avail, key=lambda f: f["bpow"])
        for k in ("bpow", "bratio", "spec_ent", "peak"): row[f"{task}_{k}"] = st[k]
    return row

# ------------------------------------------------------------------ build feature table
if os.path.exists(CACHE):
    df, y = pickle.load(open(CACHE, "rb"))
    print(f"Loading feature cache: {os.path.basename(CACHE)}")
else:
    if ROOT is None:
        raise SystemExit("Feature cache not found, and the pads-...-1.0.0 dataset folder was not found either"
                         " (must contain movement/timeseries and patients/). Put the PADS dataset in the same folder as this file and run again.")
    labmap = {}
    for f in glob.glob(os.path.join(ROOT, "patients", "patient_*.json")):
        d = json.load(open(f)); labmap[d["id"]] = d.get("condition", "?")
    rows, ys, ids = [], [], []
    for sid in sorted(labmap):
        r = subject_row(sid)
        if r is None: continue
        rows.append(r); ys.append(labmap[sid]); ids.append(sid)
    df = pd.DataFrame(rows, index=ids); y = np.array(ys)
    pickle.dump((df, y), open(CACHE, "wb"))
cols = list(df.columns)
print(f"Subjects {len(y)} | sensor features {len(cols)} (no questionnaire/no demographics) | distribution {dict(collections.Counter(y))}")

# ---- feature groups (all sensor) ----
BASE = [f"{t}_{k}" for t in ("Relaxed", "StretchHold") for k in BASE_KEYS]                       # 10
REG  = [f"{t}_{k}" for t in ("Relaxed", "StretchHold") for k in ("spec_ent", "peak_sharp", "lohi")]
GYRO = [f"{t}_{k}" for t in ("Relaxed", "StretchHold") for k in ("g_bratio", "freq_stab")]
ASYM = [c for c in cols if c.endswith("_asym_bpow")]
HEALTH = [f"{t}_{k}" for t in ("Relaxed", "StretchHold") for k in ("ac_reg", "jerk")]            # healthy-group oriented
EXTRA = [c for c in cols if c.startswith("HoldWeight_") or c.startswith("DrinkGlas_")]
TAP = [c for c in cols if any(c.startswith(t + "_") for t in KIN)] + [c for c in ["rest_vs_kin"] if c in cols]
CONTRAST = ["rp_bpow", "rp_bratio", "rp_peak", "rest_dom", "ra_bpow"]
SENS2 = [c for c in cols if any(c.startswith(t + "_") for t in EXTRA2)]
CUR  = BASE + REG + GYRO + ASYM + HEALTH
FULL = cols                                                                                     # all sensor

def RF(n=350): return Pipeline([("imp", SimpleImputer(strategy="median")),
    ("clf", RandomForestClassifier(n_estimators=n, random_state=0, n_jobs=-1, class_weight="balanced"))])
def HGB(): return Pipeline([("clf", HistGradientBoostingClassifier(random_state=0, max_depth=3, learning_rate=0.06, max_iter=400, l2_regularization=1.0))])
def LR(): return Pipeline([("imp", SimpleImputer(strategy="median")), ("sc", StandardScaler()),
    ("clf", LogisticRegression(max_iter=2000, class_weight="balanced", C=0.5))])

def rauc(X, yb, model, R):
    a = []
    for r in range(R):
        skf = StratifiedKFold(5, shuffle=True, random_state=r)
        pr = cross_val_predict(model, X, yb, cv=skf, method="predict_proba", n_jobs=-1)[:, 1]
        a.append(roc_auc_score(yb, pr))
    return float(np.mean(a)), float(np.std(a))

PAIRS = [("PD vs Healthy", "Parkinson's", "Healthy"),
         ("PD vs Essential Tremor", "Parkinson's", "Essential Tremor")]
L = ["PADS watch-only (sensor) version - subject-wise StratifiedKFold (one row per subject, no leakage, no questionnaire)",
     f"AUC = RepeatedStratifiedKFold(5×{R_HEAD}) pooled-OOF, mean±std" + ("  [FAST]" if FAST else ""), "=" * 70]
def log(s): print(s); L.append(str(s))
summ = {}
for name, pos, neg in PAIRS:
    mask = (y == pos) | (y == neg); yb = (y[mask] == pos).astype(int)
    log(f"\n### {name}  (n={mask.sum()}, {pos} {int((yb==1).sum())} vs {neg} {int((yb==0).sum())})")
    b = rauc(df.loc[mask, BASE].values, yb, RF(), R_HEAD)
    c = rauc(df.loc[mask, CUR].values,  yb, RF(), R_HEAD)
    f = rauc(df.loc[mask, FULL].values, yb, RF(), R_HEAD)
    log(f"  baseline ({len(BASE)} features, RF)                    AUC {b[0]:.3f} +/- {b[1]:.3f}")
    log(f"  rest+posture ({len(CUR)} features, RF)                 AUC {c[0]:.3f} +/- {c[1]:.3f}")
    log(f"  all sensor ({len(FULL)} features, RF)                  AUC {f[0]:.3f} +/- {f[1]:.3f}")
    for mn, md in [("HistGBM", HGB()), ("LogReg", LR())]:
        m = rauc(df.loc[mask, FULL].values, yb, md, R_HEAD)
        log(f"    (all sensor + {mn:8s})                          AUC {m[0]:.3f} +/- {m[1]:.3f}")
    summ[name] = dict(baseline=b, cur=c, full=f)

# ---- leakage guard: label-shuffle null test ----
log("\n" + "=" * 70); log("Leakage test (after shuffling labels, AUC should be ~ 0.5):")
rng = np.random.default_rng(0)
for name, pos, neg in PAIRS:
    mask = (y == pos) | (y == neg); yb = (y[mask] == pos).astype(int); X = df.loc[mask, FULL].values
    real = np.mean([rauc(X, yb, RF(), 1)[0] for _ in range(2)])
    nulls = [rauc(X, rng.permutation(yb), RF(), 1)[0] for _ in range(R_NULL * 2)]
    log(f"  [{name:24s}] real {real:.3f} | shuffled {np.mean(nulls):.3f} +/- {np.std(nulls):.3f} (max {np.max(nulls):.3f})")

# ---- Ablation (add sensor feature groups one by one) ----
log("\n" + "=" * 70); log("Ablation (add feature groups one by one):")
GROUPS = [("baseline", BASE), ("+regularity", REG), ("+gyro", GYRO), ("+asymmetry", ASYM),
          ("+health", HEALTH), ("+extra tasks", EXTRA), ("+tapping", TAP),
          ("+rest/action", CONTRAST), ("+sensor-extra", SENS2)]
GROUPS = [g for g in GROUPS if g[1]]
abl = {}
for name, pos, neg in PAIRS:
    mask = (y == pos) | (y == neg); yb = (y[mask] == pos).astype(int)
    cum = []; curve = []
    for gn, gc in GROUPS:
        cum = cum + gc; m = rauc(df.loc[mask, cum].values, yb, RF(300), R_ABL)
        curve.append(m[0]); log(f"  [{name[:14]:14s}] {gn:14s} ({len(cum):2d}) AUC {m[0]:.3f}")
    abl[name] = curve

# ---- hold-out permutation importance (PD vs ET) ----
mask = (y == "Parkinson's") | (y == "Essential Tremor"); yb = (y[mask] == "Parkinson's").astype(int)
X = df.loc[mask, FULL].values
Xtr, Xte, ytr, yte = train_test_split(X, yb, test_size=0.35, stratify=yb, random_state=1)
mdl = RF().fit(Xtr, ytr)
pi_mean = permutation_importance(mdl, Xte, yte, n_repeats=30, random_state=1, scoring="roc_auc", n_jobs=-1).importances_mean
order = np.argsort(pi_mean)[::-1][:10]

# ---- charts ----
fig, ax = plt.subplots(1, 3, figsize=(17, 4.8))
names = [p[0] for p in PAIRS]
base = [summ[n]["baseline"][0] for n in names]; full = [summ[n]["full"][0] for n in names]
bsd = [summ[n]["baseline"][1] for n in names]; fsd = [summ[n]["full"][1] for n in names]
x = np.arange(len(names)); w = 0.34
b1 = ax[0].bar(x - w/2, base, w, yerr=bsd, capsize=4, color="#9AA9AE", label=f"Baseline ({len(BASE)})")
b2 = ax[0].bar(x + w/2, full, w, yerr=fsd, capsize=4, color="#0F7C8A", label=f"Sensor-only ({len(FULL)})")
for bb, v in zip(b1, base): ax[0].text(bb.get_x()+bb.get_width()/2, v+.012, f"{v:.2f}", ha="center", fontsize=10, color="#444")
for bb, v in zip(b2, full): ax[0].text(bb.get_x()+bb.get_width()/2, v+.012, f"{v:.2f}", ha="center", fontsize=10, fontweight="bold", color="#0B5560")
ax[0].set_xticks(x); ax[0].set_xticklabels(["PD vs\nHealthy", "PD vs\nEss. Tremor"]); ax[0].set_ylim(0.5, 1.0)
ax[0].set_ylabel("AUC (subject-level CV)"); ax[0].set_title("Sensor-only (no questionnaire)"); ax[0].legend(fontsize=9, loc="lower right"); ax[0].axhline(.5, color="k", ls="--", alpha=.3)
GL = [g[0] for g in GROUPS]
for name, c in [("PD vs Healthy", "#0F7C8A"), ("PD vs Essential Tremor", "#EC8A3C")]:
    ax[1].plot(range(len(GL)), abl[name], "-o", color=c, label=name)
ax[1].set_xticks(range(len(GL))); ax[1].set_xticklabels(GL, rotation=35, ha="right", fontsize=8)
ax[1].set_ylim(0.5, 1.0); ax[1].set_ylabel("AUC"); ax[1].set_title("Ablation: AUC as feature groups added"); ax[1].legend(fontsize=8, loc="lower right"); ax[1].axhline(.5, color="k", ls="--", alpha=.3)
labs = [cols[i] for i in order][::-1]; vals = [pi_mean[i] for i in order][::-1]
colr = ["#EC8A3C" if any(k in l for k in ("rp_", "ra_", "rest_dom", "asym")) else "#17A08F" for l in labs]
ax[2].barh(labs, vals, color=colr); ax[2].set_title("Top features — PD vs ET (hold-out)\norange = rest-vs-posture/action & asymmetry"); ax[2].set_xlabel("permutation importance (AUC drop)")
plt.tight_layout(); plt.savefig(os.path.join(HERE, "pads_sensoronly_charts.png"), dpi=130)
open(os.path.join(HERE, "pads_sensoronly_results.txt"), "w", encoding="utf-8").write("\n".join(L))
log("\nCharts saved to pads_sensoronly_charts.png | summary saved to pads_sensoronly_results.txt")
