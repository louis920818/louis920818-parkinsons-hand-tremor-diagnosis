import numpy as np, pandas as pd
from sklearn.model_selection import StratifiedGroupKFold
from sklearn.metrics import roc_auc_score
from sklearn.impute import SimpleImputer
import xgboost as xgb
df=pd.read_csv("pads_win_multitask.csv")
FEATS=["peak","bratio","bpow","rms","grms","spec_ent","centroid","peak_prom","zcr","iqr","kurt","skew","g_bratio"]
P=dict(max_depth=5,eta=0.08,min_child_weight=3,subsample=0.9,colsample_bytree=0.9,objective='binary:logistic',tree_method='hist',eval_metric='auc',base_score=0.5);NR=300
d=df[df.label.isin([1,0])].copy(); d["y"]=(d.label==1).astype(int)
X=d[FEATS].values;grp=d["sid"].values
# shuffle labels by subject (one random label per person)
rng=np.random.RandomState(0); uid=np.unique(grp); perm={u:rng.randint(0,2) for u in uid}
ysh=np.array([perm[u] for u in grp])
skf=StratifiedGroupKFold(5,shuffle=True,random_state=1);oof=np.full(len(ysh),np.nan)
for tr,te in skf.split(X,ysh,grp):
    im=SimpleImputer(strategy='median').fit(X[tr]);m=xgb.train(P,xgb.DMatrix(im.transform(X[tr]),label=ysh[tr]),NR);oof[te]=m.predict(xgb.DMatrix(im.transform(X[te])))
agg=d.assign(p=oof,ys=ysh).groupby("sid").agg(p=("p","mean"),ys=("ys","first"))
print(f"subject-wise leakage test (shuffled labels) AUC = {roc_auc_score(agg.ys,agg.p):.3f}  (should be ~0.5)")
