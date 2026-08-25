import json, numpy as np, pandas as pd
import xgboost as xgb

df=pd.read_csv("/home/claude/sensor_feats.csv",index_col=0)
y=df["label"].values; cols=list(df.drop(columns=["label"]).columns); Xall=df.drop(columns=["label"]).values
NF=len(cols)
MODELS=[("HC","/home/claude/xgb_HC.json","Parkinson's","Healthy"),
        ("ET","/home/claude/xgb_ET.json","Parkinson's","Essential Tremor")]

def flatten(bst):
    dumps=bst.get_dump(dump_format="json")
    FEAT=[];THR=[];LEFT=[];RIGHT=[];ROOTS=[]
    for ds in dumps:
        tree=json.loads(ds); idx={}
        # first pass assign global indices via DFS preorder
        order=[]
        def walk(n):
            order.append(n)
            if "children" in n:
                walk(n["children"][0]); walk(n["children"][1])
        walk(tree)
        base=len(FEAT)
        for k,n in enumerate(order): idx[n["nodeid"]]=base+k
        ROOTS.append(idx[tree["nodeid"]])
        for n in order:
            if "leaf" in n:
                FEAT.append(-1); THR.append(float(n["leaf"])); LEFT.append(-1); RIGHT.append(-1)
            else:
                f=int(n["split"][1:]); FEAT.append(f); THR.append(float(n["split_condition"]))
                LEFT.append(idx[n["yes"]]); RIGHT.append(idx[n["no"]])
    return FEAT,THR,LEFT,RIGHT,ROOTS

def py_margin(F,T,L,R,ROOTS,x):
    s=0.0
    for r in ROOTS:
        i=r
        while F[i]!=-1:
            i = L[i] if x[F[i]] < T[i] else R[i]
        s+=T[i]
    return s

emitted={}
for tag,path,pos,neg in MODELS:
    bst=xgb.Booster(); bst.load_model(path)
    F,T,L,R,ROOTS=flatten(bst)
    # intercept: booster margin - sum_leaves, on real (imputed) samples
    mask=(y==pos)|(y==neg)
    from sklearn.impute import SimpleImputer
    med=SimpleImputer(strategy="median").fit(Xall[mask]).statistics_
    Xi=np.where(np.isnan(Xall[mask]),med,Xall[mask])
    dm=xgb.DMatrix(Xi); bm=bst.predict(dm,output_margin=True)
    mys=np.array([py_margin(F,T,L,R,ROOTS,Xi[k]) for k in range(len(Xi))])
    inter=float(np.median(bm-mys)); err=float(np.max(np.abs((mys+inter)-bm)))
    # also verify prob matches
    prob_bst=bst.predict(dm); prob_my=1/(1+np.exp(-(mys+inter)))
    perr=float(np.max(np.abs(prob_bst-prob_my)))
    print(f"[{tag}] trees={len(ROOTS)} nodes={len(F)} intercept={inter:.5f} margin_err={err:.2e} prob_err={perr:.2e}")
    emitted[tag]=dict(F=F,T=T,L=L,R=R,ROOTS=ROOTS,inter=inter,med=med.tolist())

# ---- emit C header ----
def carr(name,vals,typ="int",per=20):
    s=f"static const {typ} {name}[{len(vals)}] = {{\n"
    for i in range(0,len(vals),per):
        chunk=vals[i:i+per]
        if typ=="float": s+="  "+",".join(f"{v:.6g}f" for v in chunk)+",\n"
        else: s+="  "+",".join(str(int(v)) for v in chunk)+",\n"
    return s+"};\n"

H=['// Auto-generated from XGBoost (PADS sensor-only). PD screening — research, not diagnosis.',
   '// Two models: HC = PD vs healthy; ET = PD vs essential tremor. predict returns PD probability (0~1).',
   '#pragma once','#include <math.h>','',f'#define PADS_XGB_NFEAT {NF}','']
H.append('// Feature order (index -> name):')
for i,c in enumerate(cols): H.append(f'//  {i:2d} {c}')
H.append('')
for tag in ("HC","ET"):
    e=emitted[tag]; p=f"PADS_{tag}"
    H.append(f'// ===== model {tag} =====')
    H.append(f'#define {p}_NTREE {len(e["ROOTS"])}')
    H.append(f'#define {p}_NNODE {len(e["F"])}')
    H.append(f'static const float {p}_INTERCEPT = {e["inter"]:.6g}f;')
    H.append(carr(f"{p}_FEAT",e["F"],"short"))
    H.append(carr(f"{p}_THR",e["T"],"float"))
    H.append(carr(f"{p}_LEFT",e["L"],"short"))
    H.append(carr(f"{p}_RIGHT",e["R"],"short"))
    H.append(carr(f"{p}_ROOT",e["ROOTS"],"short"))
    H.append(carr(f"{p}_MEDIAN",e["med"],"float"))
    H.append('')
# generic predictor macro-ish function
H.append(r'''static inline float pads_xgb_predict(const short* FEAT,const float* THR,const short* LEFT,
    const short* RIGHT,const short* ROOT,int ntree,float inter,const float* median,const float* feat){
  float x[PADS_XGB_NFEAT];
  for(int i=0;i<PADS_XGB_NFEAT;i++){ float v=feat[i]; x[i]=(isnan(v)?median[i]:v); }
  float s=inter;
  for(int t=0;t<ntree;t++){ int i=ROOT[t];
    while(FEAT[i]!=-1) i = (x[FEAT[i]]<THR[i])?LEFT[i]:RIGHT[i];
    s+=THR[i]; }
  return 1.0f/(1.0f+expf(-s));
}
static inline float pads_predict_PD_vs_HC(const float* feat){
  return pads_xgb_predict(PADS_HC_FEAT,PADS_HC_THR,PADS_HC_LEFT,PADS_HC_RIGHT,PADS_HC_ROOT,PADS_HC_NTREE,PADS_HC_INTERCEPT,PADS_HC_MEDIAN,feat);
}
static inline float pads_predict_PD_vs_ET(const float* feat){
  return pads_xgb_predict(PADS_ET_FEAT,PADS_ET_THR,PADS_ET_LEFT,PADS_ET_RIGHT,PADS_ET_ROOT,PADS_ET_NTREE,PADS_ET_INTERCEPT,PADS_ET_MEDIAN,feat);
}''')
open("/home/claude/pads_xgb_model.h","w").write("\n".join(H))
import os; print("WROTE pads_xgb_model.h  bytes=",os.path.getsize("/home/claude/pads_xgb_model.h"))
