import numpy as np, pandas as pd
from sklearn.model_selection import StratifiedGroupKFold, StratifiedKFold
from sklearn.metrics import roc_auc_score
from sklearn.impute import SimpleImputer
import xgboost as xgb
df=pd.read_csv("pads_overlap_windows.csv")
FEATS=["peak","bratio","bpow","rms","grms","spec_ent","centroid","peak_prom"]
P=dict(max_depth=3,eta=0.05,min_child_weight=5,subsample=0.9,colsample_bytree=0.9,objective='binary:logistic',tree_method='hist',eval_metric='auc',base_score=0.5); NR=250
def sub(pn): d=df[df.label.isin(pn)].copy(); d["y"]=(d.label==pn[0]).astype(int); return d
def A_base(d,seeds=(1,2,3,4,5)):
    g=d.groupby(["sid","wrist"]); X=g[FEATS].mean().values; y=g["y"].first().values; grp=g["y"].first().index.get_level_values(0).values; R=[]
    for s in seeds:
        skf=StratifiedGroupKFold(5,shuffle=True,random_state=s); oof=np.full(len(y),np.nan)
        for tr,te in skf.split(X,y,grp):
            im=SimpleImputer(strategy='median').fit(X[tr]); m=xgb.train(P,xgb.DMatrix(im.transform(X[tr]),label=y[tr]),NR); oof[te]=m.predict(xgb.DMatrix(im.transform(X[te])))
        R.append(roc_auc_score(y,oof))
    return np.mean(R),np.std(R)
def subj_auc_from_oof(d,oof): agg=d.assign(p=oof).groupby("sid").agg(p=("p","mean"),y=("y","first")); return roc_auc_score(agg.y,agg.p)
def B_correct(d,seeds=(1,2,3,4,5)):
    X=d[FEATS].values; y=d["y"].values; grp=d["sid"].values; R=[]
    for s in seeds:
        skf=StratifiedGroupKFold(5,shuffle=True,random_state=s); oof=np.full(len(y),np.nan)
        for tr,te in skf.split(X,y,grp):
            im=SimpleImputer(strategy='median').fit(X[tr]); m=xgb.train(P,xgb.DMatrix(im.transform(X[tr]),label=y[tr]),NR); oof[te]=m.predict(xgb.DMatrix(im.transform(X[te])))
        R.append(subj_auc_from_oof(d,oof))
    return np.mean(R),np.std(R)
def C_leaky_subj(d,seeds=(1,2,3,4,5)):   # windows split randomly (leakage) -> aggregated per subject (same baseline as B)
    X=d[FEATS].values; y=d["y"].values; R=[]
    for s in seeds:
        skf=StratifiedKFold(5,shuffle=True,random_state=s); oof=np.full(len(y),np.nan)
        for tr,te in skf.split(X,y):
            im=SimpleImputer(strategy='median').fit(X[tr]); m=xgb.train(P,xgb.DMatrix(im.transform(X[tr]),label=y[tr]),NR); oof[te]=m.predict(xgb.DMatrix(im.transform(X[te])))
        R.append(subj_auc_from_oof(d,oof))
    return np.mean(R),np.std(R)
for name,pn in [("PD vs healthy",[1,0]),("PD vs other disorders (incl. essential tremor)",[1,2])]:
    d=sub(pn); a=A_base(d); b=B_correct(d); c=C_leaky_subj(d)
    print(f"\n=== {name} (LiftHold, 8 features/window, subject-level AUC) ===")
    print(f"  A no overlapping windows (baseline)          {a[0]:.3f}+/-{a[1]:.3f}")
    print(f"  B overlapping 'correct' (subject-wise group) {b[0]:.3f}+/-{b[1]:.3f}   OK reportable")
    print(f"  C overlapping 'inflated' (random split, leak){c[0]:.3f}+/-{c[1]:.3f}   invalid (leakage)")
