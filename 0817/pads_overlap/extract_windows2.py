import numpy as np, pandas as pd, glob, os
from numpy.fft import rfft, rfftfreq

# ---- tunable parameters (first column, clearly labeled) ----
WIN_SEC = float(os.environ.get("WIN_SEC", 2.56))   # window length (seconds)
OVERLAP = float(os.environ.get("OVERLAP", 0.80))   # overlap ratio 0~0.95 (advisor: partial overlap is fine -> raised here)
FS=100; TRIM_SEC=0.5; BAND=(2.0,12.0); TREM=(3.0,7.0)
DS=glob.glob(os.path.expanduser("~/mnt/*/pads-*"))[0]
TS=os.path.join(DS,"movement","timeseries"); FL=os.path.join(DS,"preprocessed","file_list.csv")
fl=pd.read_csv(FL); fl=fl[fl.resource_type=='patient'][['id','label']]
lab={str(r.id).zfill(3):int(r.label) for r in fl.itertuples()}
COLS=["peak","bratio","bpow","rms","grms","spec_ent","centroid","peak_prom","zcr","iqr","kurt","skew","g_bratio"]

def _std(x):
    m=x.mean(); v=((x-m)**2).mean(); return m,np.sqrt(v)+1e-12
def kurtosis(x):
    m,sd=_std(x); return float((((x-m)/sd)**4).mean()-3.0)
def skew(x):
    m,sd=_std(x); return float((((x-m)/sd)**3).mean())

def win_feats(acc,gyr):
    N=acc.shape[0]; lin=acc-acc.mean(0); w=np.hanning(N); fr=rfftfreq(N,1/FS)
    P=np.zeros(N//2+1); 
    for a in range(3): F=rfft(lin[:,a]*w); P+=F.real**2+F.imag**2
    Pg=np.zeros(N//2+1)
    for a in range(3): Fg=rfft((gyr[:,a]-gyr[:,a].mean())*w); Pg+=Fg.real**2+Fg.imag**2
    bm=(fr>=BAND[0])&(fr<=BAND[1]); tm=(fr>=TREM[0])&(fr<=TREM[1])
    bandP=P[bm].sum(); tremP=P[tm].sum()
    if bandP<=0: return None
    peak=fr[bm][np.argmax(P[bm])]
    pr=P[bm]/bandP; ent=-(pr[pr>0]*np.log(pr[pr>0])).sum()/np.log(bm.sum())
    cent=(fr[bm]*P[bm]).sum()/bandP
    mag=np.sqrt((lin**2).sum(1)); rms=float(np.sqrt((lin**2).sum(1).mean()))
    grms=float(np.sqrt((gyr**2).sum(1).mean()))
    md=np.median(P[bm]); prom=float(np.log10(P[bm].max()/(md+1e-12)+1))
    zc=float(((mag-mag.mean())[:-1]*(mag-mag.mean())[1:]<0).mean())
    iqr=float(np.percentile(mag,75)-np.percentile(mag,25))
    ku=float(kurtosis(mag)); sk=float(skew(mag))
    gb=float(Pg[tm].sum()/(Pg[bm].sum()+1e-12))
    return [float(peak),float(tremP/bandP),float(np.log10(tremP+1e-12)),rms,grms,float(ent),float(cent),prom,zc,iqr,ku,sk,gb]
def load(path):
    try: d=pd.read_csv(path,header=None).values
    except: return None
    t=d[:,0]; k=int(TRIM_SEC*100)
    if d.shape[0]<=k+64: return None
    d=d[k:]; t=d[:,0]; acc=d[:,1:4]; gyr=d[:,4:7]*(180.0/np.pi)
    nt=np.arange(t[0],t[-1],1.0/FS)
    acc=np.stack([np.interp(nt,t,acc[:,a]) for a in range(3)],1); gyr=np.stack([np.interp(nt,t,gyr[:,a]) for a in range(3)],1)
    return acc,gyr
win=int(WIN_SEC*FS); hop=max(1,int(win*(1-OVERLAP))); rows=[]
for sid in sorted(lab.keys()):
    for wr in ["LeftWrist","RightWrist"]:
        p=os.path.join(TS,f"{sid}_LiftHold_{wr}.txt")
        if not os.path.exists(p): continue
        r=load(p)
        if r is None: continue
        acc,gyr=r; N=acc.shape[0]; wi=0
        for s in range(0,N-win+1,hop):
            f=win_feats(acc[s:s+win],gyr[s:s+win])
            if f is None: continue
            rows.append([sid,lab[sid],wr,wi]+f); wi+=1
df=pd.DataFrame(rows,columns=["sid","label","wrist","win"]+COLS)
out="pads_overlap_windows2.csv"; df.to_csv(out,index=False)
print(f"WIN={win}pt HOP={hop}pt OVERLAP={OVERLAP} rows={len(df)} feats={len(COLS)} avg_win/rec={len(df)/df.groupby(['sid','wrist']).ngroups:.1f}")
