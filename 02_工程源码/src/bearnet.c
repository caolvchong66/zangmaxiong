#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "bearnet.h"
#include "windex.h"

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

#if defined(__arm__) || defined(__ARM_NEON)
#define BN_MODEL_DATA 1
#endif
extern const unsigned char model_weights[];

static const unsigned char *W = 0;
static int bn_inited = 0;

/* ---------------- workspace ---------------- */
#define S BN_S

static float in_buf[3*S*S];
static float t0[16*160*160];
static float t1[32*80*80];
static float t2[32*80*80];
static float t3[64*40*40];
static float t4[128*20*20];
static float t5[256*10*10];
static float b6[128*20*20];
static float hA[384*40*40];
static float hB[192*40*40];
static float hC[192*20*20];
static float hD[384*10*10];
static float c2f_cv1[64*80*80];
static float c2f_mid[2*32*80*80];
static float c2f_cat[48*80*80];
static float det_box[64*40*40];
static float det_cls[64*40*40];
static float spool[512*10*10];
static float up_buf[128*40*40];
static int8_t qbuf[460000];
static int8_t cbuf[2000000];

int bn_dbg = 0;
static void dbg_dump_raw(const void *p, int n, const char *path)
{
    FILE *f;
    if (!bn_dbg) return;
    f = fopen(path, "wb");
    if (!f) return;
    fwrite(p, 1, n, f);
    fclose(f);
}

static void dbg_dump(const char *p, int n, const char *path)
{
    FILE *f;
    if (!bn_dbg) return;
    f = fopen(path, "wb");
    if (!f) return;
    fwrite(p, 4, n, f);
    fclose(f);
}

static const wconv_t *findw(const char *name)
{
    int i;
    for (i = 0; i < WCONV_N; i++)
        if (strcmp(WCONV[i].name, name) == 0) return &WCONV[i];
    return 0;
}

static float amax_of(const float *x, int n)
{
    float m = 0.f; int i;
    for (i = 0; i < n; i++) { float a = fabsf(x[i]); if (a > m) m = a; }
    return m;
}

static void quantize(const float *x, int n, int8_t *q, float scale)
{
    float inv = 1.f / scale; int i;
    for (i = 0; i < n; i++) {
        float v = x[i] * inv;
        int iv = (int)(v + (v >= 0 ? 0.5f : -0.5f));
        if (iv > 127) iv = 127; if (iv < -127) iv = -127;
        q[i] = (int8_t)iv;
    }
}

static void im2col(const int8_t *q, int cin, int ih, int iw, int k, int s, int p,
                   int oh, int ow, int8_t *col)
{
    int y, x, c, ky, kx;
    int K = cin * k * k;
    for (y = 0; y < oh; y++) {
        for (x = 0; x < ow; x++) {
            int8_t *dst = col + (size_t)(y * ow + x) * K;
            int iy0 = y * s - p, ix0 = x * s - p;
            int idx = 0;
            for (c = 0; c < cin; c++) {
                const int8_t *src = q + (size_t)c * ih * iw;
                for (ky = 0; ky < k; ky++) {
                    int iy = iy0 + ky;
                    for (kx = 0; kx < k; kx++) {
                        int ix = ix0 + kx;
                        dst[idx++] = (iy >= 0 && iy < ih && ix >= 0 && ix < iw) ? src[iy * iw + ix] : 0;
                    }
                }
            }
        }
    }
}

static float sigmoidf_(float x) { return 1.f / (1.f + expf(-x)); }

static void silu(float *x, int n)
{
    int i;
    for (i = 0; i < n; i++) x[i] = x[i] * sigmoidf_(x[i]);
}

static void conv(const char *name, const float *in, int cin, int ih, int iw,
                 float *out, int act, int *pcout, int *poh, int *pow_)
{
    const wconv_t *wc = findw(name);
    int cout, k, s, p, oh, ow, K, pcount, oc0, pp, i;
    float sa;
    const int8_t *qw;
    const float *wsc, *bias;

    if (!wc) { printf("missing conv %s\n", name); exit(1); }
    cout = wc->cout; k = wc->k; s = wc->s; p = wc->p;
    oh = (ih + 2*p - k) / s + 1;
    ow = (iw + 2*p - k) / s + 1;
    K = cin * k * k;
    pcount = oh * ow;
    qw = (const int8_t *)(W + wc->off);
    wsc = (const float *)(W + wc->off + (size_t)cout * K);
    bias = (const float *)(W + wc->off + (size_t)cout * K + (size_t)cout * 4);

    sa = amax_of(in, cin * ih * iw);
    if (sa < 1e-8f) sa = 1e-8f;
    sa = sa / 127.f;

    quantize(in, cin * ih * iw, qbuf, sa);
    im2col(qbuf, cin, ih, iw, k, s, p, oh, ow, cbuf);
    if (name[6] == '0' && name[7] == 0) {
        dbg_dump_raw(qbuf, cin * ih * iw, "d_q.bin");
        dbg_dump_raw(cbuf, (int)((size_t)K * pcount), "d_col.bin");
        dbg_dump(in, cin * ih * iw, "d_in.bin");
    }

    for (oc0 = 0; oc0 < cout; oc0 += 8) {
        int nb = cout - oc0;
        const int8_t *wbase;
        float *obase;
        if (nb > 8) nb = 8;
        wbase = qw + (size_t)oc0 * K;
        obase = out + (size_t)oc0 * pcount;
        if (nb == 8) {
            for (pp = 0; pp < pcount; pp++) {
                const int8_t *cr = cbuf + (size_t)pp * K;
                int32_t acc[8];
                int b;
                i = 0;
                for (b = 0; b < 8; b++) acc[b] = 0;
#ifdef __ARM_NEON
                {
                    int32x4_t v0 = vdupq_n_s32(0), v1 = vdupq_n_s32(0);
                    int32x4_t v2 = vdupq_n_s32(0), v3 = vdupq_n_s32(0);
                    int32x4_t v4 = vdupq_n_s32(0), v5 = vdupq_n_s32(0);
                    int32x4_t v6 = vdupq_n_s32(0), v7 = vdupq_n_s32(0);
                    for (; i + 16 <= K; i += 16) {
                        int8x16_t a8 = vld1q_s8(cr + i);
                        int16x8_t a01 = vmovl_s8(vget_low_s8(a8));
                        int16x8_t a23 = vmovl_s8(vget_high_s8(a8));
                        int16x4_t al0 = vget_low_s16(a01), ah0 = vget_high_s16(a01);
                        int16x4_t al1 = vget_low_s16(a23), ah1 = vget_high_s16(a23);
                        int8x16_t w8; int16x8_t w01, w23;
#define WSTEP(N, V) do { \
    w8 = vld1q_s8(wbase + (size_t)(N) * K + i); \
    w01 = vmovl_s8(vget_low_s8(w8)); \
    w23 = vmovl_s8(vget_high_s8(w8)); \
    V = vmlal_s16(V, al0, vget_low_s16(w01)); \
    V = vmlal_s16(V, ah0, vget_high_s16(w01)); \
    V = vmlal_s16(V, al1, vget_low_s16(w23)); \
    V = vmlal_s16(V, ah1, vget_high_s16(w23)); \
} while (0)
                        WSTEP(0, v0); WSTEP(1, v1); WSTEP(2, v2); WSTEP(3, v3);
                        WSTEP(4, v4); WSTEP(5, v5); WSTEP(6, v6); WSTEP(7, v7);
                    }
#undef WSTEP
                    acc[0] = vgetq_lane_s32(v0,0)+vgetq_lane_s32(v0,1)+vgetq_lane_s32(v0,2)+vgetq_lane_s32(v0,3);
                    acc[1] = vgetq_lane_s32(v1,0)+vgetq_lane_s32(v1,1)+vgetq_lane_s32(v1,2)+vgetq_lane_s32(v1,3);
                    acc[2] = vgetq_lane_s32(v2,0)+vgetq_lane_s32(v2,1)+vgetq_lane_s32(v2,2)+vgetq_lane_s32(v2,3);
                    acc[3] = vgetq_lane_s32(v3,0)+vgetq_lane_s32(v3,1)+vgetq_lane_s32(v3,2)+vgetq_lane_s32(v3,3);
                    acc[4] = vgetq_lane_s32(v4,0)+vgetq_lane_s32(v4,1)+vgetq_lane_s32(v4,2)+vgetq_lane_s32(v4,3);
                    acc[5] = vgetq_lane_s32(v5,0)+vgetq_lane_s32(v5,1)+vgetq_lane_s32(v5,2)+vgetq_lane_s32(v5,3);
                    acc[6] = vgetq_lane_s32(v6,0)+vgetq_lane_s32(v6,1)+vgetq_lane_s32(v6,2)+vgetq_lane_s32(v6,3);
                    acc[7] = vgetq_lane_s32(v7,0)+vgetq_lane_s32(v7,1)+vgetq_lane_s32(v7,2)+vgetq_lane_s32(v7,3);
                }
#endif
                for (; i < K; i++)
                    for (b = 0; b < 8; b++) acc[b] += (int)cr[i] * (int)wbase[(size_t)b * K + i];
                for (b = 0; b < 8; b++)
                    obase[(size_t)b * pcount + pp] = (float)acc[b] * sa * wsc[oc0 + b] + bias[oc0 + b];
            }
        } else {
            int b;
            for (b = 0; b < nb; b++) {
                const int8_t *wr = wbase + (size_t)b * K;
                float scale = sa * wsc[oc0 + b];
                float bv = bias[oc0 + b];
                float *orow = obase + (size_t)b * pcount;
                for (pp = 0; pp < pcount; pp++) {
                    const int8_t *cr = cbuf + (size_t)pp * K;
                    int isum = 0;
                    for (i = 0; i < K; i++) isum += (int)cr[i] * (int)wr[i];
                    orow[pp] = (float)isum * scale + bv;
                }
            }
        }
    }
    if (act) silu(out, cout * pcount);
    *pcout = cout; *poh = oh; *pow_ = ow;
}

static void concat2(const float *a, int ca, const float *b, int cb, int hw, float *out)
{
    memcpy(out, a, (size_t)ca * hw * 4);
    memcpy(out + (size_t)ca * hw, b, (size_t)cb * hw * 4);
}

static void maxpool5(const float *in, float *out, int ch, int h, int w)
{
    int c, y, x, ky, kx;
    for (c = 0; c < ch; c++) {
        const float *s = in + (size_t)c * h * w;
        float *d = out + (size_t)c * h * w;
        for (y = 0; y < h; y++) for (x = 0; x < w; x++) {
            float m = -1e30f;
            for (ky = -2; ky <= 2; ky++) for (kx = -2; kx <= 2; kx++) {
                int iy = y + ky, ix = x + kx;
                if (iy < 0) iy = 0; if (iy >= h) iy = h - 1;
                if (ix < 0) ix = 0; if (ix >= w) ix = w - 1;
                if (s[iy*w+ix] > m) m = s[iy*w+ix];
            }
            d[y*w+x] = m;
        }
    }
}

static void upsample2(const float *in, float *out, int ch, int h, int w)
{
    int c, y, x;
    for (c = 0; c < ch; c++) {
        const float *s = in + (size_t)c*h*w;
        float *d = out + (size_t)c*(h*2)*(w*2);
        for (y = 0; y < h*2; y++) for (x = 0; x < w*2; x++)
            d[y*(w*2)+x] = s[(y/2)*w + x/2];
    }
}

/* C2f: cv1 -> split in 2 -> n bottlenecks -> concat -> cv2 */
static void c2f(const char *p1, const char *p2, const char *n1a, const char *n1b,
                const char *n2a, const char *n2b, int n, int has_shortcut,
                const float *in, int cin, int h, int w, float *out)
{
    int c1 = 0, c2 = 0, oh = 0, ow = 0, i, half, hw = h*w;
    float *y1, *last, *dst;

    conv(p1, in, cin, h, w, c2f_cv1, 1, &c1, &oh, &ow);
    half = c1 / 2;
    y1 = c2f_cv1 + (size_t)half * hw;
    memcpy(c2f_cat, c2f_cv1, (size_t)half * hw * 4);
    memcpy(c2f_cat + (size_t)half*hw, y1, (size_t)half * hw * 4);
    dst = c2f_cat + (size_t)2 * half * hw;
    last = y1;
    for (i = 0; i < n; i++) {
        int oc;
        float *m1 = c2f_mid;
        float *m2 = c2f_mid + (size_t)half*hw;
        conv(i == 0 ? n1a : n2a, last, half, h, w, m1, 1, &oc, &oh, &ow);
        conv(i == 0 ? n1b : n2b, m1, half, h, w, m2, 1, &oc, &oh, &ow);
        if (has_shortcut) {
            int j;
            for (j = 0; j < half*hw; j++) m2[j] += last[j];
        }
        memcpy(dst, m2, (size_t)half*hw*4);
        last = dst;
        dst += (size_t)half*hw;
    }
    conv(p2, c2f_cat, (2 + n) * half, h, w, out, 1, &c2, &oh, &ow);
}

static void detect_head(int ch, int h, int w, const char *b0, const char *b1, const char *b2,
                        const char *c0, const char *c1, const char *c2,
                        const float *in, int stride, float *out, int anchor_base)
{
    int oc, oh, ow;
    float *cls;
    conv(b0, in, ch, h, w, det_box, 1, &oc, &oh, &ow);
    conv(b1, det_box, oc, h, w, det_box, 1, &oc, &oh, &ow);
    conv(b2, det_box, oc, h, w, det_box, 0, &oc, &oh, &ow);
    conv(c0, in, ch, h, w, det_cls, 1, &oc, &oh, &ow);
    conv(c1, det_cls, oc, h, w, det_cls, 1, &oc, &oh, &ow);
    conv(c2, det_cls, oc, h, w, det_cls, 0, &oc, &oh, &ow);
    cls = det_cls;

    {
        int y, x, side, i;
        for (y = 0; y < h; y++) for (x = 0; x < w; x++) {
            int a = anchor_base + y * w + x;
            float x1 = 0, y1 = 0, x2 = 0, y2 = 0;
            float ax = (x + 0.5f) * stride, ay = (y + 0.5f) * stride;
            for (side = 0; side < 4; side++) {
                float p[16], mx = -1e30f, sum = 0.f, d = 0.f;
                for (i = 0; i < 16; i++) {
                    float v = det_box[(size_t)(side*16+i)*h*w + y*w + x];
                    p[i] = v; if (v > mx) mx = v;
                }
                for (i = 0; i < 16; i++) { p[i] = expf(p[i] - mx); sum += p[i]; }
                for (i = 0; i < 16; i++) d += (p[i] / sum) * (float)i;
                d *= stride;
                if (side == 0) x1 = ax - d;
                else if (side == 1) y1 = ay - d;
                else if (side == 2) x2 = ax + d;
                else y2 = ay + d;
            }
            out[(size_t)0*BN_ANCHORS + a] = (x1 + x2) * 0.5f;
            out[(size_t)1*BN_ANCHORS + a] = (y1 + y2) * 0.5f;
            out[(size_t)2*BN_ANCHORS + a] = x2 - x1;
            out[(size_t)3*BN_ANCHORS + a] = y2 - y1;
            out[(size_t)4*BN_ANCHORS + a] = sigmoidf_(cls[y*w+x]);
        }
    }
}

int bn_init(void)
{
    if (bn_inited) return 0;
#ifdef BN_MODEL_DATA
    W = model_weights;
#else
    {
        FILE *f = fopen("weights.bin", "rb");
        static unsigned char *buf;
        long n;
        if (!f) { printf("no weights.bin\n"); return -1; }
        fseek(f, 0, SEEK_END); n = ftell(f); fseek(f, 0, SEEK_SET);
        buf = (unsigned char *)malloc(n);
        if (fread(buf, 1, n, f) != (size_t)n) { fclose(f); return -1; }
        fclose(f);
        W = buf;
    }
#endif
    bn_inited = 1;
    return 0;
}

void bn_forward(const float *input_chw, float *out)
{
    int c = 0, h = 0, w = 0;
    const int S1 = S / 2, S2 = S / 4, S3 = S / 8, S4 = S / 16, S5 = S / 32;
    const int A1 = (S / 8) * (S / 8);
    const int A2 = A1 + (S / 16) * (S / 16);

    memcpy(in_buf, input_chw, sizeof(float) * 3 * S * S);

    conv("model.0", in_buf, 3, S, S, t0, 1, &c, &h, &w);
    conv("model.1", t0, 16, S1, S1, t1, 1, &c, &h, &w);
    c2f("model.2.cv1", "model.2.cv2", "model.2.m.0.cv1", "model.2.m.0.cv2",
        "model.2.m.0.cv1", "model.2.m.0.cv2", 1, 1, t1, 32, S2, S2, t2);
    conv("model.3", t2, 32, S2, S2, t3, 1, &c, &h, &w);
    c2f("model.4.cv1", "model.4.cv2", "model.4.m.0.cv1", "model.4.m.0.cv2",
        "model.4.m.1.cv1", "model.4.m.1.cv2", 2, 1, t3, 64, S3, S3, t3);
    conv("model.5", t3, 64, S3, S3, t4, 1, &c, &h, &w);
    c2f("model.6.cv1", "model.6.cv2", "model.6.m.0.cv1", "model.6.m.0.cv2",
        "model.6.m.1.cv1", "model.6.m.1.cv2", 2, 1, t4, 128, S4, S4, b6);
    conv("model.7", b6, 128, S4, S4, t5, 1, &c, &h, &w);
    c2f("model.8.cv1", "model.8.cv2", "model.8.m.0.cv1", "model.8.m.0.cv2",
        "model.8.m.0.cv1", "model.8.m.0.cv2", 1, 1, t5, 256, S5, S5, t5);
    {
        int oc, oh, ow;
        conv("model.9.cv1", t5, 256, S5, S5, t5, 1, &oc, &oh, &ow);
        memcpy(spool, t5, sizeof(float) * 128 * S5 * S5);
        maxpool5(t5, spool + 128 * S5 * S5, 128, S5, S5);
        maxpool5(spool + 128 * S5 * S5, spool + 2 * 128 * S5 * S5, 128, S5, S5);
        maxpool5(spool + 2 * 128 * S5 * S5, spool + 3 * 128 * S5 * S5, 128, S5, S5);
        conv("model.9.cv2", spool, 512, S5, S5, t5, 1, &oc, &oh, &ow);
    }
    upsample2(t5, up_buf, 256, S5, S5);
    concat2(up_buf, 256, b6, 128, S4 * S4, hA);
    c2f("model.12.cv1", "model.12.cv2", "model.12.m.0.cv1", "model.12.m.0.cv2",
        "model.12.m.0.cv1", "model.12.m.0.cv2", 1, 0, hA, 384, S4, S4, hA);

    upsample2(hA, up_buf, 128, S4, S4);
    concat2(up_buf, 128, t3, 64, S3 * S3, hB);
    c2f("model.15.cv1", "model.15.cv2", "model.15.m.0.cv1", "model.15.m.0.cv2",
        "model.15.m.0.cv1", "model.15.m.0.cv2", 1, 0, hB, 192, S3, S3, t3);
    conv("model.16", t3, 64, S3, S3, hC, 1, &c, &h, &w);
    concat2(hC, 64, hA, 128, S4 * S4, hC);
    c2f("model.18.cv1", "model.18.cv2", "model.18.m.0.cv1", "model.18.m.0.cv2",
        "model.18.m.0.cv1", "model.18.m.0.cv2", 1, 0, hC, 192, S4, S4, t4);
    conv("model.19", t4, 128, S4, S4, hD, 1, &c, &h, &w);
    concat2(hD, 128, t5, 256, S5 * S5, hD);
    c2f("model.21.cv1", "model.21.cv2", "model.21.m.0.cv1", "model.21.m.0.cv2",
        "model.21.m.0.cv1", "model.21.m.0.cv2", 1, 0, hD, 384, S5, S5, t5);

    detect_head(64, S3, S3, "model.22.cv2.0.cv2.0.0", "model.22.cv2.0.cv2.0.1", "model.22.cv2.0.cv2.0.2",
                "model.22.cv3.0.cv3.0.0", "model.22.cv3.0.cv3.0.1", "model.22.cv3.0.cv3.0.2",
                t3, 8, out, 0);
    detect_head(128, S4, S4, "model.22.cv2.1.cv2.1.0", "model.22.cv2.1.cv2.1.1", "model.22.cv2.1.cv2.1.2",
                "model.22.cv3.1.cv3.1.0", "model.22.cv3.1.cv3.1.1", "model.22.cv3.1.cv3.1.2",
                t4, 16, out, A1);
    detect_head(256, S5, S5, "model.22.cv2.2.cv2.2.0", "model.22.cv2.2.cv2.2.1", "model.22.cv2.2.cv2.2.2",
                "model.22.cv3.2.cv3.2.0", "model.22.cv3.2.cv3.2.1", "model.22.cv3.2.cv3.2.2",
                t5, 32, out, A2);
}
