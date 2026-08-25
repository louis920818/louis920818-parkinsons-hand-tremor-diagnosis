import numpy as np, pandas as pd, glob, os, json
DS = glob.glob(os.path.expanduser("~/mnt/*/pads-*"))[0]
TS = os.path.join(DS, "movement", "timeseries")
FL = os.path.join(DS, "preprocessed", "file_list.csv")
FFT=512; HOP=256; MAXW=15; BLO,BHI,TLO,THI=2.0,12.0,3.0,7.0; MINPROM=3.0
hann=np.hanning(FFT); hannPow=float((hann**2).sum())
def psd(x, fs):
    xw=(x-x.mean())*hann; X=np.fft.rfft(xw); scale=fs*hannPow
    P=(X.real**2+X.imag**2)/(scale if scale>0 else 1.0); P[1:]*=2.0
    return P[:FFT//2]
def integ(P,lo,hi,binHz):
    low=max(1,int(np.ceil(lo/binHz))); high=min(FFT//2-1,int(np.floor(hi/binHz)))
    return float(P[low:high+1].sum()*binHz)
def win_feat(x, fs):   # x length FFT (raw channel values)
    binHz=fs/FFT; P=psd(x,fs)
    low=max(1,int(np.ceil(BLO/binHz))); high=min(FFT//2-2,int(np.floor(BHI/binHz)))
    seg=P[low:high+1]; idx=np.arange(low,high+1)
    peak=low+int(np.argmax(seg)); powerSum=float(seg.sum())
    dom=peak*binHz
    p2_12=integ(P,BLO,BHI,binHz); p3_7=integ(P,TLO,THI,binHz)
    ratio=p3_7/p2_12 if p2_12>0 else 0.0
    centroid=float((P[low:high+1]*idx*binHz).sum()/powerSum) if powerSum>0 else 0.0
    if powerSum>0:
        pr=seg/powerSum; nz=pr>0; ent=float(-(pr[nz]*np.log(pr[nz])).sum())
        cnt=high-low+1; sent=ent/np.log(cnt) if cnt>1 else 0.0
    else: sent=0.0
    bg=float(np.median(seg)); prom=10.0*np.log10((P[peak]+1e-18)/(bg+1e-18))
    arms=float(np.sqrt((x*x).mean()))
    return dict(dom=dom,p2_12=p2_12,ratio=ratio,centroid=centroid,sent=sent,prom=prom,arms=arms)
def windows(vals, fs):
    N=len(vals); wc=min(MAXW, 1+(N-FFT)//HOP) if N>=FFT else 0
    return [win_feat(vals[w*HOP:w*HOP+FFT], fs) for w in range(wc)]
def med_field(ws, key):
    if not ws: return np.nan
    if key=='p2_12log': v=[np.log10(w['p2_12']+1e-18) for w in ws]
    else: v=[w[key] for w in ws]
    return float(np.median(v))
def mad(v):
    m=np.median(v); return float(np.median(np.abs(v-m)))
def iqr(v):
    return float(np.quantile(v,0.75)-np.quantile(v,0.25))
def freq_stab(ws):
    f=[w['dom'] for w in ws if w['prom']>=MINPROM]
    if len(f)<2: return np.nan
    return float(np.std(f))            # population std (ddof=0)
def amp_var(ws):
    if not ws: return 0.0
    a=np.array([w['arms'] for w in ws]); m=a.mean()
    if m<=0: return 0.0
    return float(np.sqrt(((a-m)**2).mean())/m)
def extract(path, target_fs, resample):
    try: d=pd.read_csv(path,header=None).values
    except: return None
    k=int(0.5*100)                      # trim 0.5s at native 100Hz
    if d.shape[0]<=k+64: return None
    d=d[k:]
    t=d[:,0]; acc=d[:,1:4]; gyr=d[:,4:7]*(180.0/np.pi)  # GYRO_DEG: PADS rad/s -> device deg/s
    if resample:
        nt=np.arange(t[0],t[-1],1.0/target_fs)
        acc=np.stack([np.interp(nt,t,acc[:,a]) for a in range(3)],1)
        gyr=np.stack([np.interp(nt,t,gyr[:,a]) for a in range(3)],1)
    fs=target_fs
    ax,ay,az=acc[:,0],acc[:,1],acc[:,2]
    gx,gy,gz=gyr[:,0],gyr[:,1],gyr[:,2]
    gmag=np.sqrt(gx*gx+gy*gy+gz*gz)
    wAX,wAY,wAZ=windows(ax,fs),windows(ay,fs),windows(az,fs)
    wGX,wGY=windows(gx,fs),windows(gy,fs)
    f=np.full(15,np.nan)
    f[0]=med_field(wAX,'p2_12log'); f[1]=med_field(wAX,'centroid'); f[2]=med_field(wAY,'centroid')
    f[3]=mad(gy);                    f[4]=med_field(wAZ,'centroid'); f[5]=freq_stab(wGX)
    f[6]=med_field(wAZ,'prom');      f[7]=med_field(wAX,'ratio');    f[8]=mad(gz)
    f[9]=iqr(gmag);                  f[10]=amp_var(wAX);             f[11]=med_field(wAZ,'sent')
    f[12]=med_field(wAY,'ratio');    f[13]=med_field(wGY,'ratio');   f[14]=med_field(wGX,'centroid')
    return f
if __name__=='__main__':
    fl=pd.read_csv(FL); fl=fl[fl.resource_type=='patient'][['id','condition','label']]
    lab={str(r.id).zfill(3):(r.condition,int(r.label)) for r in fl.itertuples()}
    SCALER_MEAN=np.array([-4.54808854,6.51319652,6.63282883,4.17313814,5.44142152,1.91264826,11.10853522,0.43243705,2.80963513,9.54913797,0.56913327,0.78009637,0.38149490,0.48299823,6.76812285])
    import sys; NS=int(sys.argv[1]) if len(sys.argv)>1 else 40
    mode=sys.argv[2] if len(sys.argv)>2 else '100n'
    ids=sorted(lab.keys())[:NS]
    rows=[]
    for sid in ids:
        for wr in ['LeftWrist','RightWrist']:
            p=os.path.join(TS,f"{sid}_LiftHold_{wr}.txt")
            if not os.path.exists(p): continue
            if mode=='100n': f=extract(p,100,False)
            elif mode=='200r': f=extract(p,200,True)
            elif mode=='100r': f=extract(p,100,True)
            if f is not None: rows.append(f)
    R=np.array(rows); mean=np.nanmean(R,0)
    print("mode",mode,"n=",len(rows))
    for i in range(15):
        print(f"  f{i:2d} mine={mean[i]:9.3f}  scaler={SCALER_MEAN[i]:9.3f}  ratio={mean[i]/SCALER_MEAN[i]:.2f}")
