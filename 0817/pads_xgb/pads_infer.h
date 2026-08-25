// PADS sensor-only inference integration -- assembles per-task capture results into the 66-dim feature vector and calls XGBoost (C version).
// For research screening, not a diagnosis. A single device (one watch) cannot capture the left-right asymmetry features -> set to NAN and imputed by the model with the training median.
#pragma once
#include "pads_sensor_features.h"
#include "pads_xgb_model.h"

// Results of one full guided capture (for un-captured tasks just leave .valid as 0; their features are auto-filled NAN->median).
typedef struct {
  PadsRec relaxed, stretchhold, holdweight, drinkglas;               // rest/posture/hold-weight/drink
  PadsRec relaxedtask, lifthold, crossarms, touchindex, entrainment; // other sensor tasks
  PadsRep pointfinger, touchnose;                                    // finger-tap type (repetitive action)
} PadsCapture;

static void pads_build_features(const PadsCapture* c, float* f){
  for(int i=0;i<PADS_NFEAT;i++) f[i]=NAN;
  const PadsRec *R=&c->relaxed, *S=&c->stretchhold, *H=&c->holdweight, *D=&c->drinkglas;
  // Relaxed RICH 0..11 + asym 12 (single watch->NAN)
  if(R->valid){ f[0]=R->peak;f[1]=R->bratio;f[2]=R->bpow;f[3]=R->rms;f[4]=R->grms;f[5]=R->spec_ent;
    f[6]=R->peak_sharp;f[7]=R->lohi;f[8]=R->g_bratio;f[9]=R->freq_stab;f[10]=R->ac_reg;f[11]=R->jerk; }
  // StretchHold RICH 13..24 + asym 25
  if(S->valid){ f[13]=S->peak;f[14]=S->bratio;f[15]=S->bpow;f[16]=S->rms;f[17]=S->grms;f[18]=S->spec_ent;
    f[19]=S->peak_sharp;f[20]=S->lohi;f[21]=S->g_bratio;f[22]=S->freq_stab;f[23]=S->ac_reg;f[24]=S->jerk; }
  // HoldWeight 26..28, DrinkGlas 29..31 (bpow,bratio,spec_ent)
  if(H->valid){ f[26]=H->bpow;f[27]=H->bratio;f[28]=H->spec_ent; }
  if(D->valid){ f[29]=D->bpow;f[30]=D->bratio;f[31]=D->spec_ent; }
  // contrast features (need Relaxed + StretchHold)
  if(R->valid && S->valid){ f[32]=R->bpow-S->bpow; f[33]=R->bratio-S->bratio; f[34]=R->peak-S->peak;
    float mx=S->bpow; if(H->valid&&H->bpow>mx)mx=H->bpow; if(D->valid&&D->bpow>mx)mx=D->bpow; f[35]=R->bpow-mx; }
  if(R->valid && D->valid) f[36]=R->bpow-D->bpow;
  // finger-tap type PointFinger 37..40, TouchNose 41..44
  if(c->pointfinger.valid){ f[37]=c->pointfinger.rep_rate;f[38]=c->pointfinger.rep_reg;f[39]=c->pointfinger.rep_dec;f[40]=c->pointfinger.kin_bratio; }
  if(c->touchnose.valid){ f[41]=c->touchnose.rep_rate;f[42]=c->touchnose.rep_reg;f[43]=c->touchnose.rep_dec;f[44]=c->touchnose.kin_bratio; }
  if(R->valid){ float mk=-1e30f,has=0; if(c->pointfinger.valid){mk=c->pointfinger.kin_bratio;has=1;} if(c->touchnose.valid&&c->touchnose.kin_bratio>mk){mk=c->touchnose.kin_bratio;has=1;}
    if(has) f[45]=R->bratio-mk; }
  // EXTRA2 tasks (bpow,bratio,spec_ent,peak): RelaxedTask 46..49, LiftHold 50..53, CrossArms 54..57, TouchIndex 58..61, Entrainment 62..65
  const PadsRec* ex[5]={&c->relaxedtask,&c->lifthold,&c->crossarms,&c->touchindex,&c->entrainment};
  for(int i=0;i<5;i++){ const PadsRec* e=ex[i]; int b=46+i*4; if(e->valid){ f[b]=e->bpow;f[b+1]=e->bratio;f[b+2]=e->spec_ent;f[b+3]=e->peak; } }
}

// Convenience: returns PD probability (0~1). gate = PD vs healthy; diff = PD vs essential tremor.
static inline float pads_screen_PD_vs_HC(const PadsCapture* c){ float f[PADS_NFEAT]; pads_build_features(c,f); return pads_predict_PD_vs_HC(f); }
static inline float pads_screen_PD_vs_ET(const PadsCapture* c){ float f[PADS_NFEAT]; pads_build_features(c,f); return pads_predict_PD_vs_ET(f); }
