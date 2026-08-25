import numpy as np, pandas as pd, json, os
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import roc_auc_score
from sklearn.impute import SimpleImputer
import xgboost as xgb
df=pd.read_csv("sensor_feats.csv",index_col=0); cond=df.label.values; FE=df.drop(columns=['label'])
c=list(FE.columns)
rest=[x for x in c if x.startswith('Relaxed_') and 'asym' not in x]      # rest 12
post=[x for x in c if x.startswith('StretchHold_') and 'asym' not in x]  # posture 12
tap =[x for x in c if x.startswith('PointFinger_')]                       # tap 4
FEATS=rest+post+tap                                                       # 28
print("Feature order (28):"); [print(f"  {i:2d} {n}") for i,n in enumerate(FEATS)]
# tunable hyperparameters (small model, MCU-friendly; exact for bit-consistent export)
MAX_DEPTH=3; ETA=0.05; NROUND=250; MIN_CHILD=3
P=dict(max_depth=MAX_DEPTH,eta=ETA,min_child_weight=MIN_CHILD,subsample=0.9,colsample_bytree=0.9,
       objective='binary:logistic',tree_method='exact',eval_metric='auc',base_score=0.5)
def flatten(bst):
    FEo=[];TH=[];LE=[];RI=[];RO=[]
    for ds in bst.get_dump(dump_format="json"):
        tree=json.loads(ds); order=[]
        def walk(n):
            order.append(n)
            if "children" in n: walk(n["children"][0]); walk(n["children"][1])
        walk(tree); idx={}; base=len(FEo)
        for k,n in enumerate(order): idx[n["nodeid"]]=base+k
        RO.append(idx[tree["nodeid"]])
        for n in order:
            if "leaf" in n: FEo.append(-1);TH.append(float(n["leaf"]));LE.append(-1);RI.append(-1)
            else:
                FEo.append(int(n["split"][1:])); TH.append(float(n["split_condition"])); LE.append(idx[n["yes"]]); RI.append(idx[n["no"]])
    return FEo,TH,LE,RI,RO
def pym(F,T,L,R,RO,x):
    s=0.0
    for r in RO:
        i=r
        while F[i]!=-1: i=L[i] if x[F[i]]<T[i] else R[i]
        s+=T[i]
    return s
def cvauc(pos,neg,seeds=(1,2,3,4,5)):
    m=np.isin(cond,[pos,neg]); X=FE[FEATS].values[m]; y=(cond[m]==pos).astype(int); A=[]
    for s in seeds:
        skf=StratifiedKFold(5,shuffle=True,random_state=s); oof=np.full(len(y),np.nan)
        for tr,te in skf.split(X,y):
            im=SimpleImputer(strategy='median').fit(X[tr]); d=xgb.train(P,xgb.DMatrix(im.transform(X[tr]),label=y[tr]),NROUND); oof[te]=d.predict(xgb.DMatrix(im.transform(X[te])))
        A.append(roc_auc_score(y,oof))
    sh=[]
    for sd in range(6):
        rng=np.random.RandomState(sd); ysh=rng.permutation(y); skf=StratifiedKFold(5,shuffle=True,random_state=7); oof=np.full(len(y),np.nan)
        for tr,te in skf.split(X,ysh):
            im=SimpleImputer(strategy='median').fit(X[tr]); d=xgb.train(P,xgb.DMatrix(im.transform(X[tr]),label=ysh[tr]),NROUND); oof[te]=d.predict(xgb.DMatrix(im.transform(X[te])))
        sh.append(roc_auc_score(ysh,oof))
    return np.mean(A),np.std(A),np.mean(sh)
def export(pos,neg,tag):
    m=np.isin(cond,[pos,neg]); X=FE[FEATS].values[m]; y=(cond[m]==pos).astype(int)
    med=np.nanmedian(X,0); Xi=np.where(np.isnan(X),med,X)
    bst=xgb.train(P,xgb.DMatrix(Xi,label=y),NROUND); Fl=flatten(bst)
    dm=xgb.DMatrix(Xi); bm=bst.predict(dm,output_margin=True); mys=np.array([pym(*Fl,Xi[k]) for k in range(len(Xi))])
    inter=float(np.median(bm-mys)); perr=float(np.max(np.abs(bst.predict(dm)-1/(1+np.exp(-(mys+inter))))))
    print(f"[{tag}] trees={len(Fl[4])} nodes={len(Fl[0])} prob_err={perr:.2e}")
    return dict(F=Fl[0],T=Fl[1],L=Fl[2],R=Fl[3],RO=Fl[4],inter=inter,med=med.tolist())
a,s,sh=cvauc("Parkinson's","Healthy"); print(f"[validation] PD vs healthy AUC={a:.3f}+/-{s:.3f} leakage(6shuf)={sh:.3f}")
a2,s2,sh2=cvauc("Parkinson's","Essential Tremor"); print(f"[validation] PD vs essential tremor AUC={a2:.3f}+/-{s2:.3f} leakage={sh2:.3f}")
E={'HC':export("Parkinson's","Healthy","PD_vs_HC"),'ET':export("Parkinson's","Essential Tremor","PD_vs_ET")}
def carr(name,vals,typ,per=20):
    s=f"static const {typ} {name}[{len(vals)}] = {{\n"
    for i in range(0,len(vals),per):
        ch=vals[i:i+per]; s+="  "+(",".join(f"{v:.7g}f" for v in ch) if typ=="float" else ",".join(str(int(v)) for v in ch))+",\n"
    return s+"};\n"
H=['// Three-task (rest Relaxed + posture StretchHold + tap PointFinger) XGBoost -- replaces the guided CMPI hand-weighting.',
   '// Research screening, not a diagnosis. Features use pads_rec_feats (rest/posture) and pads_rep_feats (tap) from pads_sensor_features.h, fs=100Hz.',
   '// The raw acc/gyr of each task must be resampled to 100Hz before computing features. Returns PD probability (0~1).','#pragma once','#include <math.h>','',
   '#define PADS_G3_NFEAT 28','']
for i,n in enumerate(FEATS): H.append(f'//  {i:2d} {n}')
H.append('')
for tag in ('HC','ET'):
    e=E[tag]; p=f"PADS_G3_{tag}"
    H.append(f'#define {p}_NTREE {len(e["RO"])}'); H.append(f'static const float {p}_INTERCEPT = {e["inter"]:.7g}f;')
    for nm,v,t in [("FEAT",e["F"],"short"),("THR",e["T"],"float"),("LEFT",e["L"],"short"),("RIGHT",e["R"],"short"),("ROOT",e["RO"],"short"),("MEDIAN",e["med"],"float")]:
        H.append(carr(f"{p}_{nm}",v,t))
    H.append('')
H.append(r'''static inline float pads_g3_predict(const short* FEAT,const float* THR,const short* LEFT,
    const short* RIGHT,const short* ROOT,int ntree,float inter,const float* median,const float* feat){
  float x[PADS_G3_NFEAT];
  for(int i=0;i<PADS_G3_NFEAT;i++){ float v=feat[i]; x[i]=(isnan(v)?median[i]:v); }
  float s=inter;
  for(int t=0;t<ntree;t++){ int i=ROOT[t];
    while(FEAT[i]!=-1) i=(x[FEAT[i]]<THR[i])?LEFT[i]:RIGHT[i];
    s+=THR[i]; }
  return 1.0f/(1.0f+expf(-s));
}
// feat[28] order = rest rich 12 (peak,bratio,bpow,rms,grms,spec_ent,peak_sharp,lohi,g_bratio,freq_stab,ac_reg,jerk)
//             + posture rich 12 (same) + tap 4 (rep_rate,rep_reg,rep_dec,kin_bratio)
static inline float pads_g3_PD_vs_HC(const float* feat){
  return pads_g3_predict(PADS_G3_HC_FEAT,PADS_G3_HC_THR,PADS_G3_HC_LEFT,PADS_G3_HC_RIGHT,PADS_G3_HC_ROOT,PADS_G3_HC_NTREE,PADS_G3_HC_INTERCEPT,PADS_G3_HC_MEDIAN,feat);
}
static inline float pads_g3_PD_vs_ET(const float* feat){
  return pads_g3_predict(PADS_G3_ET_FEAT,PADS_G3_ET_THR,PADS_G3_ET_LEFT,PADS_G3_ET_RIGHT,PADS_G3_ET_ROOT,PADS_G3_ET_NTREE,PADS_G3_ET_INTERCEPT,PADS_G3_ET_MEDIAN,feat);
}
static const char* const PADS_G3_VERSION = "pads-guided3-xgb-20260818";''')
open("pads_xgb_guided3_model.h","w").write("\n".join(H))
print("WROTE pads_xgb_guided3_model.h bytes=",os.path.getsize("pads_xgb_guided3_model.h"))
