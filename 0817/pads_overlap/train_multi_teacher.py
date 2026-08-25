import numpy as np, pandas as pd
from sklearn.model_selection import StratifiedGroupKFold, StratifiedKFold
from sklearn.metrics import roc_auc_score
from sklearn.impute import SimpleImputer
import xgboost as xgb
df=pd.read_csv("pads_win_multitask.csv")
FEATS=["peak","bratio","bpow","rms","grms","spec_ent","centroid","peak_prom","zcr","iqr","kurt","skew","g_bratio"]
print("windows:",len(df),"tasks:",sorted(df.task.unique()),"avg windows per subject:",round(len(df)/df.sid.nunique(),0))
P=dict(max_depth=5,eta=0.08,min_child_weight=3,subsample=0.9,colsample_bytree=0.9,objective='binary:logistic',tree_method='hist',eval_metric='auc',base_score=0.5); NR=300
def sub(pn): d=df[df.label.isin(pn)].copy(); d["y"]=(d.label==pn[0]).astype(int); return d
def teacher(d,seeds=(1,2,3)):     # windows shuffled, random split, window-level AUC
    X=d[FEATS].values;y=d["y"].values;R=[]
    for s in seeds:
        skf=StratifiedKFold(5,shuffle=True,random_state=s);oof=np.full(len(y),np.nan)
        for tr,te in skf.split(X,y):
            im=SimpleImputer(strategy='median').fit(X[tr]);m=xgb.train(P,xgb.DMatrix(im.transform(X[tr]),label=y[tr]),NR);oof[te]=m.predict(xgb.DMatrix(im.transform(X[te])))
        R.append(roc_auc_score(y,oof))
    return np.mean(R),np.std(R)
def subjwise(d,seeds=(1,2,3)):
    X=d[FEATS].values;y=d["y"].values;grp=d["sid"].values;R=[]
    for s in seeds:
        skf=StratifiedGroupKFold(5,shuffle=True,random_state=s);oof=np.full(len(y),np.nan)
        for tr,te in skf.split(X,y,grp):
            im=SimpleImputer(strategy='median').fit(X[tr]);m=xgb.train(P,xgb.DMatrix(im.transform(X[tr]),label=y[tr]),NR);oof[te]=m.predict(xgb.DMatrix(im.transform(X[te])))
        agg=d.assign(p=oof).groupby("sid").agg(p=("p","mean"),y=("y","first"));R.append(roc_auc_score(agg.y,agg.p))
    return np.mean(R),np.std(R)
for name,pn in [("PD vs healthy",[1,0]),("PD vs other disorders",[1,2])]:
    d=sub(pn);t=teacher(d);s=subjwise(d)
    print(f"\n=== {name} (6 tasks, 13 features, 80% overlap, {len(d)} windows) ===")
    print(f"  advisor method: windows shuffled, random split (window-level AUC) {t[0]:.3f}+/-{t[1]:.3f}")
    print(f"  backup: subject-wise + aggregation (honest)          {s[0]:.3f}+/-{s[1]:.3f}")
