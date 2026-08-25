import numpy as np, pandas as pd, json, warnings
warnings.filterwarnings("ignore")
from sklearn.ensemble import RandomForestClassifier
from sklearn.impute import SimpleImputer
from sklearn.pipeline import Pipeline
from sklearn.model_selection import StratifiedKFold, cross_val_predict
from sklearn.metrics import roc_auc_score
from xgboost import XGBClassifier

df=pd.read_csv("/home/claude/sensor_feats.csv",index_col=0)
y=df["label"].values; X=df.drop(columns=["label"]); cols=list(X.columns); Xv=X.values

def RF():
    return Pipeline([("imp",SimpleImputer(strategy="median")),
        ("clf",RandomForestClassifier(n_estimators=350,random_state=0,n_jobs=-1,class_weight="balanced"))])
def XGB(spw=1.0):
    return Pipeline([("imp",SimpleImputer(strategy="median")),
        ("clf",XGBClassifier(n_estimators=300,max_depth=3,learning_rate=0.05,subsample=0.85,
            colsample_bytree=0.8,reg_lambda=1.0,min_child_weight=2.0,gamma=0.0,
            scale_pos_weight=spw,eval_metric="logloss",tree_method="hist",random_state=0,n_jobs=-1))])

def rauc(Xm,yb,model_fn,R=8):
    a=[]
    for r in range(R):
        skf=StratifiedKFold(5,shuffle=True,random_state=r)
        pr=cross_val_predict(model_fn(),Xm,yb,cv=skf,method="predict_proba",n_jobs=-1)[:,1]
        a.append(roc_auc_score(yb,pr))
    return float(np.mean(a)),float(np.std(a))

PAIRS=[("PD vs Healthy","Parkinson's","Healthy"),
       ("PD vs Essential Tremor","Parkinson's","Essential Tremor")]
rng=np.random.default_rng(0); rep={"pairs":{}}
for name,pos,neg in PAIRS:
    mask=(y==pos)|(y==neg); yb=(y[mask]==pos).astype(int); Xm=Xv[mask]
    spw=float((yb==0).sum()/max(1,(yb==1).sum()))
    rf=rauc(Xm,yb,RF); xg=rauc(Xm,yb,lambda: XGB(spw))
    # leakage null (shuffled labels) for XGB
    nulls=[rauc(Xm,rng.permutation(yb),lambda: XGB(spw),R=2)[0] for _ in range(4)]
    rep["pairs"][name]=dict(n=int(mask.sum()),pos=int((yb==1).sum()),neg=int((yb==0).sum()),
        RF=rf,XGB=xg,null_mean=float(np.mean(nulls)),null_max=float(np.max(nulls)))
    print(f"{name} (n={mask.sum()}, {pos} {int((yb==1).sum())} vs {neg} {int((yb==0).sum())})")
    print(f"   RF   AUC {rf[0]:.3f} ± {rf[1]:.3f}")
    print(f"   XGB  AUC {xg[0]:.3f} ± {xg[1]:.3f}   | shuffle-null {np.mean(nulls):.3f} (max {np.max(nulls):.3f})",flush=True)

# ---- train final deployable XGB on ALL data for each pair; save booster + feature list ----
import pickle
finals={}
for name,pos,neg in PAIRS:
    mask=(y==pos)|(y==neg); yb=(y[mask]==pos).astype(int)
    imp=SimpleImputer(strategy="median").fit(Xv[mask])
    spw=float((yb==0).sum()/max(1,(yb==1).sum()))
    clf=XGBClassifier(n_estimators=300,max_depth=3,learning_rate=0.05,subsample=0.85,colsample_bytree=0.8,
        reg_lambda=1.0,min_child_weight=2.0,scale_pos_weight=spw,eval_metric="logloss",tree_method="hist",
        random_state=0,n_jobs=-1).fit(imp.transform(Xv[mask]),yb)
    finals[name]={"imputer_medians":imp.statistics_.tolist(),"booster_json":clf.get_booster().save_raw(raw_format="json").decode() if False else None}
    clf.get_booster().save_model(f"/home/claude/xgb_{'HC' if 'Healthy' in name else 'ET'}.json")
rep["features"]=cols; rep["feature_count"]=len(cols)
json.dump(rep,open("/home/claude/xgb_report.json","w"),ensure_ascii=False,indent=2)
np.save("/home/claude/xgb_impute_medians.npy",
        {n:SimpleImputer(strategy='median').fit(Xv[(y==p)|(y==q)]).statistics_ for n,p,q in PAIRS},allow_pickle=True)
print("DONE",flush=True)
