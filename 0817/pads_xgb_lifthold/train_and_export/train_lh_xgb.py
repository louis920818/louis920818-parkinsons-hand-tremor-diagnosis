import numpy as np, pandas as pd, json
from sklearn.model_selection import StratifiedGroupKFold
from sklearn.metrics import roc_auc_score
import xgboost as xgb
df=pd.read_csv("lh_all.csv"); FEATS=[f'f{i}' for i in range(15)]
X=df[FEATS].values.astype(float); y=df.label.values; g=df.sid.values
# existing deployed LR (pads_v3) baseline
IMP=np.array([-4.62141360,6.53754891,6.57489417,3.47378789,5.33342413,1.92700076,10.70669121,0.42265262,1.61511598,6.97304457,0.52685386,0.79626068,0.37192050,0.46940372,6.72866256])
MEAN=np.array([-4.54808854,6.51319652,6.63282883,4.17313814,5.44142152,1.91264826,11.10853522,0.43243705,2.80963513,9.54913797,0.56913327,0.78009637,0.38149490,0.48299823,6.76812285])
SCALE=np.array([0.51171628,0.98672092,1.02833769,2.89668348,1.02491733,0.89523369,3.02504858,0.15230501,2.64201516,8.82250134,0.31036363,0.08120282,0.15282394,0.15276103,0.95826636])
COEF=np.array([0.85802874,0.60724724,-0.29184700,0.45949733,0.44483262,-0.27173627,-0.07002648,-0.14449949,0.19245119,0.45778122,-0.28629953,0.15603928,0.30312676,0.52407445,0.0])
INTER=0.88270409
def lr_prob(Xin):
    Xi=np.where(np.isnan(Xin),IMP,Xin); z=INTER+((Xi-MEAN)/SCALE)@COEF; return 1/(1+np.exp(-z))
# tunable hyperparameters (small model, MCU-friendly; exact for bit-consistent export)
MAX_DEPTH=3; ETA=0.15; NROUND=180; MIN_CHILD=3; SUBSAMPLE=0.9; COLSAMPLE=0.9
PARAMS=dict(max_depth=MAX_DEPTH,eta=ETA,subsample=SUBSAMPLE,colsample_bytree=COLSAMPLE,
            min_child_weight=MIN_CHILD,objective='binary:logistic',tree_method='exact',
            eval_metric='auc',base_score=0.5)
def cv(pos,neg,seeds=(1,2,3,4,5)):
    m=np.isin(y,[pos,neg]); Xm,ym,gm=X[m],(y[m]==pos).astype(int),g[m]
    xa=[]; la=[]
    for s in seeds:
        skf=StratifiedGroupKFold(5,shuffle=True,random_state=s)
        oof=np.full(len(ym),np.nan); lr=np.full(len(ym),np.nan)
        for tr,te in skf.split(Xm,ym,gm):
            med=np.nanmedian(Xm[tr],0)
            Xtr=np.where(np.isnan(Xm[tr]),med,Xm[tr]); Xte=np.where(np.isnan(Xm[te]),med,Xm[te])
            d=xgb.train(PARAMS,xgb.DMatrix(Xtr,label=ym[tr]),NROUND)
            oof[te]=d.predict(xgb.DMatrix(Xte)); lr[te]=lr_prob(Xm[te])
        xa.append(roc_auc_score(ym,oof)); la.append(roc_auc_score(ym,lr))
    rng=np.random.RandomState(0); uid=np.unique(gm); perm={u:rng.randint(0,2) for u in uid}
    ysh=np.array([perm[u] for u in gm]); skf=StratifiedGroupKFold(5,shuffle=True,random_state=1)
    oof=np.full(len(ym),np.nan)
    for tr,te in skf.split(Xm,ysh,gm):
        med=np.nanmedian(Xm[tr],0); Xtr=np.where(np.isnan(Xm[tr]),med,Xm[tr]); Xte=np.where(np.isnan(Xm[te]),med,Xm[te])
        d=xgb.train(PARAMS,xgb.DMatrix(Xtr,label=ysh[tr]),NROUND); oof[te]=d.predict(xgb.DMatrix(Xte))
    return np.mean(xa),np.std(xa),np.mean(la),roc_auc_score(ysh,oof)
rep={}
for tag,pos,neg in [("PD_vs_HC",1,0),("PD_vs_DD",1,2)]:
    a,s,l,sh=cv(pos,neg); rep[tag]=dict(xgb_auc=a,xgb_std=s,lr_auc=l,shuffle_auc=sh)
    print(f"[{tag}] XGB AUC={a:.3f}+/-{s:.3f}   existing LR AUC={l:.3f}   leakage(shuffle)={sh:.3f}")
json.dump(rep,open("lh_xgb_report.json","w"),indent=2)
