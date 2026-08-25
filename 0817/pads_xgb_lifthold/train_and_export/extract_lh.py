import numpy as np, pandas as pd, os, sys, time
import lh_feats as L
DS=L.DS; TS=L.TS
fl=pd.read_csv(L.FL); fl=fl[fl.resource_type=='patient'][['id','condition','label']]
lab={str(r.id).zfill(3):int(r.label) for r in fl.itertuples()}
ids=sorted(lab.keys())
lo=int(sys.argv[1]); hi=int(sys.argv[2]); out=sys.argv[3]
rows=[]; t0=time.time()
for sid in ids[lo:hi]:
    for wr in ['LeftWrist','RightWrist']:
        p=os.path.join(TS,f"{sid}_LiftHold_{wr}.txt")
        if not os.path.exists(p): continue
        f=L.extract(p,200,True)              # 200Hz resample + deg/s (align with device)
        if f is None: continue
        rows.append([sid,lab[sid],wr]+list(f))
cols=['sid','label','wrist']+[f'f{i}' for i in range(15)]
pd.DataFrame(rows,columns=cols).to_csv(out,index=False)
print("wrote",out,"n=",len(rows),"secs=%.1f"%(time.time()-t0))
