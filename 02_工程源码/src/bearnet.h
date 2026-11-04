#ifndef BEARNET_H_
#define BEARNET_H_

#define BN_S 256
#define BN_IN_W BN_S
#define BN_IN_H BN_S
#define BN_ANCHORS ((BN_S/8)*(BN_S/8) + (BN_S/16)*(BN_S/16) + (BN_S/32)*(BN_S/32))
#define BN_CLASSES 1

/* out: (4+BN_CLASSES) * BN_ANCHORS floats, layout [row*2100 + anchor],
   row0=x row1=y row2=w row3=h row4=class prob (all in 320x320 letterbox space) */
int  bn_init(void);
void bn_forward(const float *input_chw, float *out);

#endif
