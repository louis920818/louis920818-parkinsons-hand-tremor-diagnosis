// PADS sensor-only feature extraction — C port of pads_sensor_only.py (rec_feats / rep_feats).
// For research screening, not a diagnosis. Input is one recording's acc/gyr (after trimming). fs=100Hz.
// Uses a direct DFT (arbitrary N, aligned to numpy rfft), good for one-shot inference; larger N is slower (recommend <= 1024).
#pragma once
#include <math.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define PADS_FS       100.0f
#define PADS_NFEAT    66
#define PADS_BAND_LO  2.0f
#define PADS_BAND_HI  12.0f
#define PADS_TREM_LO  3.0f
#define PADS_TREM_HI  7.0f
#define PADS_NSEG     4        // freq_stab number of segments
#define PADS_SEG_OV   0.5f     // freq_stab segment overlap ratio

typedef struct { float peak,bratio,bpow,rms,grms,spec_ent,peak_sharp,lohi,g_bratio,freq_stab,ac_reg,jerk; int valid; } PadsRec;
typedef struct { float rep_rate,rep_reg,rep_dec,kin_bratio; int valid; } PadsRep;

// ---- helpers ----
static inline float pads_median(float* a, int n){                 // in-place sort, take the median
  for(int i=1;i<n;i++){ float k=a[i]; int j=i-1; while(j>=0&&a[j]>k){a[j+1]=a[j];j--;} a[j+1]=k; }
  return (n&1)? a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
}
// Three-axis (mean-removed + Hann) power spectrum; P length = N/2+1; returns frequency resolution fs/N. x3 is [N][3].
static void pads_pspec3(const float* x3, int N, float* P){
  int M=N/2; for(int k=0;k<=M;k++) P[k]=0.0f;
  // precompute Hann and remove the mean
  static float buf[2048]; // single-axis scratch (N<=2048)
  for(int a=0;a<3;a++){
    float mean=0; for(int n=0;n<N;n++) mean+=x3[n*3+a]; mean/=N;
    for(int n=0;n<N;n++){ float w=0.5f*(1.0f-cosf(2.0f*(float)M_PI*n/(N-1))); buf[n]=(x3[n*3+a]-mean)*w; }
    for(int k=0;k<=M;k++){ float re=0,im=0, wk=2.0f*(float)M_PI*k/N;
      for(int n=0;n<N;n++){ float ang=wk*n; re+=buf[n]*cosf(ang); im-=buf[n]*sinf(ang); }
      P[k]+=re*re+im*im; }
  }
}
static inline int pads_binlo(float f,int N){ int b=(int)ceilf(f*N/PADS_FS); return b<0?0:b; }
static inline int pads_binhi(float f,int N,int M){ int b=(int)floorf(f*N/PADS_FS); return b>M?M:b; }

// ---- rec_feats: spectral/tremor features for one recording (rest/posture, etc.) ----
static PadsRec pads_rec_feats(const float* acc,const float* gyr,int N){
  PadsRec r; memset(&r,0,sizeof(r)); r.valid=0;
  if(N<256) return r;
  int M=N/2; static float P[1025],Pg[1025];
  // linear-acceleration (mean-removed) power spectrum
  pads_pspec3(acc,N,P);
  // frequency bands
  int bl=pads_binlo(PADS_BAND_LO,N), bh=pads_binhi(PADS_BAND_HI,N,M);
  int tl=pads_binlo(PADS_TREM_LO,N), th=pads_binhi(PADS_TREM_HI,N,M);
  float bandsum=0,tremsum=0,peakP=-1; int peakb=bl;
  for(int k=bl;k<=bh;k++){ bandsum+=P[k]; if(P[k]>peakP){peakP=P[k];peakb=k;} }
  for(int k=tl;k<=th;k++) tremsum+=P[k];
  if(bandsum<=0) return r;
  r.peak = peakb*PADS_FS/N;
  r.bratio = tremsum/(bandsum+1e-12f);
  r.bpow = log10f(tremsum+1e-12f);
  // rms (mean-removed linear accel), grms (raw gyro)
  double s2=0; for(int a=0;a<3;a++){ double m=0; for(int n=0;n<N;n++) m+=acc[n*3+a]; m/=N;
    for(int n=0;n<N;n++){ double v=acc[n*3+a]-m; s2+=v*v; } }
  r.rms=(float)sqrt(s2/N);
  double g2=0; for(int n=0;n<N;n++) for(int a=0;a<3;a++){ double v=gyr[n*3+a]; g2+=v*v; }
  r.grms=(float)sqrt(g2/N);
  // spec_ent (regularity of the power distribution within the band)
  double ent=0; int cnt=bh-bl+1;
  for(int k=bl;k<=bh;k++){ double p=P[k]/(bandsum+1e-12f); if(p>0) ent-=p*log(p+1e-12); }
  r.spec_ent=(float)(ent/log((double)cnt));
  // peak_sharp
  { static float tmp[1025]; int c=0; for(int k=bl;k<=bh;k++) tmp[c++]=P[k]; float md=pads_median(tmp,c);
    r.peak_sharp=log10f(peakP/(md+1e-12f)+1.0f); }
  // lohi: tremor energy skewed to low freq = P(3~5)/P(3~7)
  { int l5=pads_binhi(5.0f,N,M); double p35=0; for(int k=tl;k<=l5;k++) p35+=P[k];
    r.lohi=(float)(p35/(tremsum+1e-12)); }
  // gyro band ratio
  pads_pspec3(gyr,N,Pg); double gb=0,gt=0; for(int k=bl;k<=bh;k++) gb+=Pg[k]; for(int k=tl;k<=th;k++) gt+=Pg[k];
  r.g_bratio=(float)(gt/(gb+1e-12));
  // freq_stab: std dev of per-segment peak frequency
  { int win=N/PADS_NSEG; int step=(int)roundf(win*(1-PADS_SEG_OV)); if(step<1)step=1;
    static float peaks[64]; int np=0; static float seg[2048*3];
    for(int s=0;s+win<=N && np<64; s+=step){
      for(int n=0;n<win;n++) for(int a=0;a<3;a++) seg[n*3+a]=acc[(s+n)*3+a];
      static float Ps[1025]; pads_pspec3(seg,win,Ps); int mm=win/2;
      int wl=pads_binlo(PADS_BAND_LO,win), wh=pads_binhi(PADS_BAND_HI,win,mm);
      float pv=-1; int pb=wl; float ss=0; for(int k=wl;k<=wh;k++){ ss+=Ps[k]; if(Ps[k]>pv){pv=Ps[k];pb=k;} }
      if(ss>0) peaks[np++]=pb*PADS_FS/win;
    }
    if(np>=2){ double mn=0; for(int i=0;i<np;i++) mn+=peaks[i]; mn/=np; double v=0;
      for(int i=0;i<np;i++){ double d=peaks[i]-mn; v+=d*d; } r.freq_stab=(float)sqrt(v/np); }
    else r.freq_stab=NAN;
  }
  // ac_reg: autocorrelation of the combined amplitude (max over lag FS/7~FS/3)
  { static float mg[2048]; double mm=0; for(int n=0;n<N;n++){ double s=0; for(int a=0;a<3;a++){ float v=acc[n*3+a]; s+=v*v; } mg[n]=(float)sqrt(s); mm+=mg[n]; }
    mm/=N; for(int n=0;n<N;n++) mg[n]-=mm;
    int lo=(int)(PADS_FS/7); if(lo<1)lo=1; int hi=(int)(PADS_FS/3);
    double ac0=0; for(int n=0;n<N;n++) ac0+=mg[n]*mg[n];
    float best=-1e9f; for(int L=lo;L<=hi&&L<N;L++){ double s=0; for(int n=0;n<N-L;n++) s+=mg[n]*mg[n+L]; float a=(float)(s/(ac0+1e-12)); if(a>best)best=a; }
    r.ac_reg = (hi>lo)? best : NAN;
  }
  // jerk: smoothness
  { double j=0; for(int n=1;n<N;n++){ double s=0; for(int a=0;a<3;a++){ double d=(acc[n*3+a]-acc[(n-1)*3+a]); s+=d*d; } j+=s; }
    r.jerk=(float)(sqrt(j/(N-1))*PADS_FS/(r.rms+1e-9f)); }
  r.valid=1; return r;
}

// ---- rep_feats: repetitive-action (finger-tap) features ----
static PadsRep pads_rep_feats(const float* acc,int N){
  PadsRep r; memset(&r,0,sizeof(r)); r.valid=0; if(N<256) return r;
  int M=N/2; static float mg[2048];
  double mean3[3]={0,0,0}; for(int n=0;n<N;n++) for(int a=0;a<3;a++) mean3[a]+=acc[n*3+a];
  for(int a=0;a<3;a++) mean3[a]/=N;
  double mm=0; for(int n=0;n<N;n++){ double s=0; for(int a=0;a<3;a++){ double v=acc[n*3+a]-mean3[a]; s+=v*v; } mg[n]=(float)sqrt(s); mm+=mg[n]; } mm/=N;
  static float env[2048]; for(int n=0;n<N;n++) env[n]=mg[n]-(float)mm;
  // envelope power spectrum (single-axis DFT + Hann)
  static float Pe[1025];
  { for(int k=0;k<=M;k++){ float re=0,im=0,wk=2.0f*(float)M_PI*k/N;
      for(int n=0;n<N;n++){ float w=0.5f*(1.0f-cosf(2.0f*(float)M_PI*n/(N-1))); float x=env[n]*w; float ang=wk*n; re+=x*cosf(ang); im-=x*sinf(ang);} Pe[k]=re*re+im*im; } }
  int ml=pads_binlo(0.3f,N), mh=pads_binhi(3.0f,N,M);
  float pv=-1; int pb=ml; double ss=0; for(int k=ml;k<=mh;k++){ ss+=Pe[k]; if(Pe[k]>pv){pv=Pe[k];pb=k;} }
  if(ss<=0) return r;
  r.rep_rate=pb*PADS_FS/N;
  { static float tmp[1025]; int c=0; for(int k=ml;k<=mh;k++) tmp[c++]=Pe[k]; float md=pads_median(tmp,c);
    r.rep_reg=log10f(pv/(md+1e-12f)+1.0f); }
  // rep_dec: 6-segment amplitude (ptp) linear decline / mean
  { int seg=N/6; float amps[6]; int na=0;
    for(int s=0;s<6;s++){ int a0=s*seg,a1=(s+1)*seg; if(a1>N)a1=N; if(a1<=a0)continue;
      float mn=mg[a0],mx=mg[a0]; for(int n=a0;n<a1;n++){ if(mg[n]<mn)mn=mg[n]; if(mg[n]>mx)mx=mg[n]; } amps[na++]=mx-mn; }
    if(na>=3){ double sx=0,sy=0,sxx=0,sxy=0; for(int i=0;i<na;i++){ sx+=i; sy+=amps[i]; sxx+=(double)i*i; sxy+=(double)i*amps[i]; }
      double slope=(na*sxy-sx*sy)/(na*sxx-sx*sx+1e-12); double am=sy/na; r.rep_dec=(float)(slope/(am+1e-9)); }
    else r.rep_dec=NAN; }
  // kin_bratio: action tremor 3-7/2-12
  { static float Pl[1025]; pads_pspec3(acc,N,Pl); int bl=pads_binlo(PADS_BAND_LO,N),bh=pads_binhi(PADS_BAND_HI,N,M),tl=pads_binlo(PADS_TREM_LO,N),th=pads_binhi(PADS_TREM_HI,N,M);
    double b=0,t=0; for(int k=bl;k<=bh;k++) b+=Pl[k]; for(int k=tl;k<=th;k++) t+=Pl[k]; r.kin_bratio=(float)(t/(b+1e-12)); }
  r.valid=1; return r;
}
