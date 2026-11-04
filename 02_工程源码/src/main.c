#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_types.h"
#include "xgpiops.h"
#include "xgpio.h"
#include "xaxivdma.h"
#include "xvtc.h"
#include "xtime_l.h"
#include "xscutimer.h"
#include "xscugic.h"
#include "sleep.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "jlog.h"
#include "display_ctrl/display_ctrl.h"
#include "vdma_api/vdma_api.h"
#include "clk_wiz/clk_wiz.h"
#include "emio_sccb_cfg/emio_sccb_cfg.h"
#include "ov5640/ov5640_init.h"
#include "bearnet.h"

#define CAM0_VDMA_ID   XPAR_AXIVDMA_0_DEVICE_ID
#define CAM1_VDMA_ID   XPAR_AXIVDMA_1_DEVICE_ID
#define DISP_VDMA_ID   XPAR_AXIVDMA_2_DEVICE_ID
#define DISP_VTC_ID    XPAR_VTC_0_DEVICE_ID
#define CLK_WIZ_ID     XPAR_CLK_WIZ_0_DEVICE_ID
#define AXI_GPIO_0_ID  XPAR_AXI_GPIO_0_DEVICE_ID
#define BYTES_PIXEL    3

#define CAM0_BUF   0x08000000u
#define CAM1_OFF   (800 * 3 / 2)   /* second camera starts mid-line (vendor layout) */
#define ANNO_BUF   0x0D000000u
#define TEST_MAGIC 0x42545354u
#define TEST_HDR   0x0E000000u
#define TEST_IMG   0x0E001000u

#define CONF_THRES  0.28f
#define MIN_BOX_PX  20.f
#define MAX_ASPECT  2.8f
#define IOU_THRES   0.45f
#define ALARM_FRAMES 3

XAxiVdma cam0_vdma, cam1_vdma, disp_vdma;
DisplayCtrl dispCtrl;
XGpio axi_gpio_inst;
VideoMode vd_mode;
unsigned int lcd_id;

static float nn_in[3*BN_IN_W*BN_IN_H];
static float nn_tmp[3*BN_IN_W*BN_IN_H];
static float nn_out[5*BN_ANCHORS];
static int   g_scale_num, g_dx, g_dy, g_nw, g_nh;

/* ---------------- buzzer ------------------------------------------
   PL pin E18 == the "PL_TX" post of jumper J21 on the ATK-DF7020 board,
   driven by channel 2 of the AXI GPIO.
   The Prechin (普中) sensor-module buzzer is a PASSIVE buzzer: it needs a
   square wave (~2 kHz) to sound and is silent on a DC level, so there is
   no active-high / active-low ambiguity.

   The channel-2 registers are written directly instead of through
   XGpio_SetDataDirection()/XGpio_DiscreteWrite(): those assert on
   InstancePtr->IsDual, which is baked into the BSP (xparameters.h) and is
   0 when the BSP predates the dual-channel GPIO.  Direct register access
   works with an old or a regenerated BSP alike.                       */
#define BUZZ_CH        2
#define BUZZ_HALF_US   250          /* 250 us half period -> 2 kHz      */

static inline void buzzer_set(u32 v)
{
    XGpio_WriteReg(XPAR_AXI_GPIO_0_BASEADDR,
                   XGPIO_CHAN_OFFSET * (BUZZ_CH - 1) + XGPIO_DATA_OFFSET, v & 1u);
}

static void buzzer_init(void)
{
    XGpio_WriteReg(XPAR_AXI_GPIO_0_BASEADDR,
                   XGPIO_CHAN_OFFSET * (BUZZ_CH - 1) + XGPIO_TRI_OFFSET, 0x00); /* output */
    buzzer_set(0);
}

static void buzzer_tone_ms(int ms)
{
    int i, n = ms * 2;              /* two toggles per millisecond      */
    for (i = 0; i < n; i++) {
        buzzer_set((u32)(i & 1));
        usleep(BUZZ_HALF_US);
    }
    buzzer_set(0);
}

/* three rising beeps = one-shot alarm (boot self test / bench check) */
static void buzzer_alarm(void)
{
    buzzer_tone_ms(160);
    usleep(120000);
    buzzer_tone_ms(160);
    usleep(120000);
    buzzer_tone_ms(220);
}

/* ---- continuous alarm while the bear is in view -------------------
   The 4 kHz display timer toggles the buzzer pin on every tick, so the
   tone is produced entirely in the interrupt and the main loop never
   blocks on it.  A detection only has to push the "alarm until" tick
   stamp forward; the tone then keeps sounding across inference gaps and
   stops a few seconds after the last detection.                     */
#define ALARM_HOLD_MS   6000
#define TICK_HZ         4000

static volatile u32 g_ticks = 0;          /* 4 kHz tick counter        */
static volatile u32 g_alarm_until = 0;    /* tick stamp, 0 = no alarm  */
static volatile int g_alarm_on = 0;

static void alarm_start(int ms)
{
    g_alarm_until = g_ticks + (u32)ms * (TICK_HZ / 1000);
    g_alarm_on = 1;
}

static void alarm_stop(void)
{
    g_alarm_on = 0;
    buzzer_set(0);
}

static void draw_rect(u32 base, int stride, int x0, int y0, int x1, int y1, int w, int h, int th);
static void put_px(u32 base, int stride, int x, int y, int w, int h, unsigned char r, unsigned char g, unsigned char b);
static void draw_corners(u32 base, int stride, int x0, int y0, int x1, int y1, int w, int h, int len, int th);
static void flush_corners(u32 base, int stride, int x0, int y0, int x1, int y1, int w, int h, int len, int th);

/* ---- live preview: refresh the display from a timer interrupt ---- */
static volatile int g_w = 480, g_h = 272;
static volatile int g_box_n = 0;
static volatile float g_box[8][4];
static XScuTimer g_timer;
static XScuGic  g_intc;

static volatile u32 g_last_fc = 0xFFFFFFFFu;
static volatile u32 g_fc_seen = 0;


/* ---------------- lightweight correlation tracker ----------------
   The detector runs every ~2 s; between detections this tracker follows the
   target on every camera frame (30 fps) so the box moves smoothly.        */
#define TRK_N     16          /* template samples per axis */
#define TRK_SAMP  2           /* half-res px between samples */
#define TRK_R     6           /* search radius, half-res px */

static volatile int g_trk_on = 0;
static unsigned char g_tmpl[TRK_N * TRK_N];
static int g_tmpl_mean = 0;
static volatile int g_trk_cx = 0, g_trk_cy = 0;
static volatile int g_trk_fail = 0;
static volatile float g_trk_w = 80.f, g_trk_h = 80.f;

static inline unsigned char trk_gray(int hx, int hy)
{
    int fx = hx * 2, fy = hy * 2;
    const unsigned char *p;
    if (fx < 0 || fy < 0 || fx >= g_w - 1 || fy >= g_h - 1) return 0;
    p = (const unsigned char *)(CAM0_BUF + (u32)fy * (u32)(g_w * 3) + (u32)fx * 3);
    return (unsigned char)((p[2] * 77 + p[1] * 150 + p[0] * 29) >> 8);
}

static void trk_grab(float cx, float cy, float bw, float bh)
{
    int i, j, sum = 0;
    g_trk_cx = (int)(cx * 0.5f);
    g_trk_cy = (int)(cy * 0.5f);
    g_trk_w = bw; g_trk_h = bh;
    for (j = 0; j < TRK_N; j++)
        for (i = 0; i < TRK_N; i++) {
            unsigned char v = trk_gray(g_trk_cx + (i - TRK_N / 2) * TRK_SAMP,
                                       g_trk_cy + (j - TRK_N / 2) * TRK_SAMP);
            g_tmpl[j * TRK_N + i] = v;
            sum += v;
        }
    g_tmpl_mean = sum / (TRK_N * TRK_N);
    g_trk_fail = 0;
    g_trk_on = 1;
}

/* mean-removed SAD at a given offset; lower is better */
static int trk_cost(int dx, int dy, int *pm)
{
    int i, j, sum = 0, mean, cost = 0, n = TRK_N * TRK_N;
    for (j = 0; j < TRK_N; j++)
        for (i = 0; i < TRK_N; i++)
            sum += trk_gray(g_trk_cx + (i - TRK_N / 2) * TRK_SAMP + dx,
                            g_trk_cy + (j - TRK_N / 2) * TRK_SAMP + dy);
    mean = sum / n;
    for (j = 0; j < TRK_N; j++)
        for (i = 0; i < TRK_N; i++) {
            int a = (int)trk_gray(g_trk_cx + (i - TRK_N / 2) * TRK_SAMP + dx,
                                  g_trk_cy + (j - TRK_N / 2) * TRK_SAMP + dy) - mean;
            int b = (int)g_tmpl[j * TRK_N + i] - g_tmpl_mean;
            int d = a - b;
            if (d < 0) d = -d;
            cost += d;
        }
    *pm = mean;
    return cost;
}

static void trk_step(void)
{
    int dx, dy, best = 0x7FFFFFFF, bdx = 0, bdy = 0, m = 0;
    /* coarse pass */
    for (dy = -TRK_R; dy <= TRK_R; dy += 2)
        for (dx = -TRK_R; dx <= TRK_R; dx += 2) {
            int c = trk_cost(dx, dy, &m);
            if (c < best) { best = c; bdx = dx; bdy = dy; }
        }
    /* fine pass around the coarse winner */
    {
        int cx0 = bdx, cy0 = bdy, fbest = best, fdx = bdx, fdy = bdy;
        for (dy = cy0 - 1; dy <= cy0 + 1; dy++)
            for (dx = cx0 - 1; dx <= cx0 + 1; dx++) {
                int c = trk_cost(dx, dy, &m);
                if (c < fbest) { fbest = c; fdx = dx; fdy = dy; }
            }
        best = fbest; bdx = fdx; bdy = fdy;
    }
    if (best < TRK_N * TRK_N * 26) {          /* confident enough */
        g_trk_cx += bdx; g_trk_cy += bdy;
        g_trk_fail = 0;
    } else if (++g_trk_fail > 40) {
        g_trk_on = 0;
    }
}

/* redraw boxes into the live camera buffer once per captured frame.
   Runs at 4 kHz: every tick toggles the buzzer (-> 2 kHz tone) while an
   alarm is active, the display/tracker work only runs every 8th tick
   (500 Hz), exactly as before.                                        */
static void display_isr(void *arg)
{
    u32 sr, fc;
    int i;
    (void)arg;
    XScuTimer_ClearInterruptStatus(&g_timer);
    g_ticks++;

    if (g_alarm_on) {
        if ((s32)(g_ticks - g_alarm_until) < 0) {
            static u32 level = 0;
            level ^= 1u;
            buzzer_set(level);                  /* 4 kHz toggle -> 2 kHz */
        } else {
            alarm_stop();                       /* hold expired, silence */
        }
    }

    if (g_ticks & 7u) return;                   /* rest runs at 500 Hz */

    {
        static int div = 0;
        if (++div >= 16) { div = 0; if (g_trk_on) trk_step(); }
    }
    sr = *(volatile u32 *)(XPAR_AXIVDMA_0_BASEADDR + 0x34);   /* S2MM status */
    fc = (sr >> 16) & 0xFF;
    g_fc_seen = fc;
    if (g_trk_on && g_box_n > 0) {
        g_box[0][0] = (float)g_trk_cx * 2.f - g_trk_w * 0.5f;
        g_box[0][1] = (float)g_trk_cy * 2.f - g_trk_h * 0.5f;
        g_box[0][2] = (float)g_trk_cx * 2.f + g_trk_w * 0.5f;
        g_box[0][3] = (float)g_trk_cy * 2.f + g_trk_h * 0.5f;
    }
    if (g_box_n <= 0) return;


    for (i = 0; i < g_box_n && i < 8; i++) {
        int x0 = (int)g_box[i][0], y0 = (int)g_box[i][1];
        int x1 = (int)g_box[i][2], y1 = (int)g_box[i][3];
        u32 a, b;
        (void)a; (void)b;
        draw_corners(CAM0_BUF, g_w * 3, x0, y0, x1, y1, g_w, g_h, 26, 4);
        flush_corners(CAM0_BUF, g_w * 3, x0, y0, x1, y1, g_w, g_h, 26, 4);
    }
}

static int display_timer_init(void)
{
    XScuTimer_Config *tcfg = XScuTimer_LookupConfig(XPAR_XSCUTIMER_0_DEVICE_ID);
    XScuGic_Config  *gcfg = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
    if (!tcfg || !gcfg) return -1;

    XScuGic_CfgInitialize(&g_intc, gcfg, gcfg->CpuBaseAddress);
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
                                 (Xil_ExceptionHandler)XScuGic_InterruptHandler, &g_intc);
    XScuGic_Connect(&g_intc, 29, (Xil_ExceptionHandler)display_isr, &g_timer);
    XScuGic_Enable(&g_intc, 29);

    XScuTimer_CfgInitialize(&g_timer, tcfg, tcfg->BaseAddr);
    XScuTimer_LoadTimer(&g_timer, 333333333u / TICK_HZ);  /* 4 kHz tick: buzzer + 500 Hz redraw */
    XScuTimer_EnableAutoReload(&g_timer);
    XScuTimer_EnableInterrupt(&g_timer);
    XScuTimer_Start(&g_timer);
    Xil_ExceptionEnable();
    return 0;
}

static void put_px(u32 base, int stride, int x, int y, int w, int h, unsigned char r, unsigned char g, unsigned char b)
{
    unsigned char *p;
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    p = (unsigned char *)(base + (u32)(y * stride + x * 3));
    p[0] = b; p[1] = g; p[2] = r;   /* panel byte order */
}

static void draw_rect(u32 base, int stride, int x0, int y0, int x1, int y1, int w, int h, int th)
{
    int i;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= w) x1 = w - 1;
    if (y1 >= h) y1 = h - 1;
    for (i = 0; i < th; i++) {
        int x, y;
        for (x = x0; x <= x1; x++) {
            put_px(base, stride, x, y0 + i, w, h, 255, 0, 0);
            put_px(base, stride, x, y1 - i, w, h, 255, 0, 0);
        }
        for (y = y0; y <= y1; y++) {
            put_px(base, stride, x0 + i, y, w, h, 255, 0, 0);
            put_px(base, stride, x1 - i, y, w, h, 255, 0, 0);
        }
    }
}

static void draw_corners(u32 base, int stride, int x0, int y0, int x1, int y1, int w, int h, int len, int th)
{
    int i, j;
    for (i = 0; i < th; i++) {
        for (j = 0; j < len; j++) {
            put_px(base, stride, x0 + j, y0 + i, w, h, 255, 0, 0);
            put_px(base, stride, x0 + i, y0 + j, w, h, 255, 0, 0);
            put_px(base, stride, x1 - j, y0 + i, w, h, 255, 0, 0);
            put_px(base, stride, x1 - i, y0 + j, w, h, 255, 0, 0);
            put_px(base, stride, x0 + j, y1 - i, w, h, 255, 0, 0);
            put_px(base, stride, x0 + i, y1 - j, w, h, 255, 0, 0);
            put_px(base, stride, x1 - j, y1 - i, w, h, 255, 0, 0);
            put_px(base, stride, x1 - i, y1 - j, w, h, 255, 0, 0);
        }
    }
}

static void flush_corners(u32 base, int stride, int x0, int y0, int x1, int y1, int w, int h, int len, int th)
{
    int n = len + th + 2;
    u32 a;
    if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
    if (x1 >= w) x1 = w - 1; if (y1 >= h) y1 = h - 1;
    a = base + (u32)y0 * stride + (u32)x0 * 3;
    Xil_DCacheFlushRange(a, (u32)n * 3 + (u32)(n * stride));
    a = base + (u32)y0 * stride + (u32)(x1 - n) * 3;
    if (x1 - n > 0) Xil_DCacheFlushRange(a, (u32)n * 3 + (u32)(n * stride));
    a = base + (u32)(y1 - n) * stride + (u32)x0 * 3;
    if (y1 - n > 0) Xil_DCacheFlushRange(a, (u32)n * 3 + (u32)(n * stride));
    a = base + (u32)(y1 - n) * stride + (u32)(x1 - n) * 3;
    if (y1 - n > 0 && x1 - n > 0) Xil_DCacheFlushRange(a, (u32)n * 3 + (u32)(n * stride));
}

/* letterbox rgb(w x h) -> 320x320 CHW float in [0,1] */
static void preprocess_rgb(const unsigned char *rgb, int stride, int srcw, int srch, float *dst)
{
    int S = BN_IN_W, y, x, c;
    float scale = (float)S / (float)(srcw > srch ? srcw : srch);
    int nw = (int)(srcw * scale), nh = (int)(srch * scale);
    int dx = (S - nw) / 2, dy = (S - nh) / 2;
    g_scale_num = nw; g_nw = nw; g_nh = nh; g_dx = dx; g_dy = dy;
    for (c = 0; c < 3; c++) {
        float *plane = dst + (size_t)c * S * S;
        for (y = 0; y < S; y++) {
            for (x = 0; x < S; x++) {
                int sx = (x - dx), sy = (y - dy);
                if (sx >= 0 && sx < nw && sy >= 0 && sy < nh) {
                    int ox = sx * srcw / nw, oy = sy * srch / nh;
                    plane[y*S + x] = rgb[(size_t)oy * stride + (size_t)ox * 3 + (2 - c)] * (1.f/255.f);
                } else {
                    plane[y*S + x] = 114.f/255.f;
                }
            }
        }
    }
}

static int nms(float *out, float thr, int *idx, float *score, int maxn)
{
    int i, j, n = 0, m = 0;
    static int order[BN_ANCHORS];
    static float scores[BN_ANCHORS];
    for (i = 0; i < BN_ANCHORS; i++) {
        float bw, bh;
        if (out[4*BN_ANCHORS + i] <= thr) continue;
        bw = out[2*BN_ANCHORS + i]; bh = out[3*BN_ANCHORS + i];
        if (bw < MIN_BOX_PX || bh < MIN_BOX_PX) continue;                 /* too small  */
        if (bw > bh * MAX_ASPECT || bh > bw * MAX_ASPECT) continue;       /* odd shape  */
        order[m] = i; scores[m] = out[4*BN_ANCHORS + i]; m++;
    }
    for (i = 0; i < m && n < maxn; i++) {
        int best = i;
        for (j = i+1; j < m; j++) if (scores[j] > scores[best]) best = j;
        if (best != i) {
            int t = order[i]; order[i] = order[best]; order[best] = t;
            { float s = scores[i]; scores[i] = scores[best]; scores[best] = s; }
        }
        {
            float x0 = out[order[i]], y0 = out[BN_ANCHORS+order[i]];
            float w0 = out[2*BN_ANCHORS+order[i]], h0 = out[3*BN_ANCHORS+order[i]];
            float ax0 = x0 - w0/2, ay0 = y0 - h0/2, ax1 = x0 + w0/2, ay1 = y0 + h0/2;
            int keep = 1;
            for (j = 0; j < n; j++) {
                int k = idx[j];
                float bx0 = out[k] - out[2*BN_ANCHORS+k]/2, by0 = out[BN_ANCHORS+k] - out[3*BN_ANCHORS+k]/2;
                float bx1 = out[k] + out[2*BN_ANCHORS+k]/2, by1 = out[BN_ANCHORS+k] + out[3*BN_ANCHORS+k]/2;
                float ix0 = ax0 > bx0 ? ax0 : bx0, iy0 = ay0 > by0 ? ay0 : by0;
                float ix1 = ax1 < bx1 ? ax1 : bx1, iy1 = ay1 < by1 ? ay1 : by1;
                float iw = ix1-ix0, ih = iy1-iy0;
                float inter = (iw > 0 && ih > 0) ? iw*ih : 0;
                float ua = (ax1-ax0)*(ay1-ay0) + (bx1-bx0)*(by1-by0) - inter;
                if (inter > 0 && ua > 0 && inter/ua > IOU_THRES) { keep = 0; break; }
            }
            if (keep) { idx[n] = order[i]; score[n] = scores[i]; n++; }
        }
    }
    return n;
}

static void run_test_mode(void)
{
    int idx[16], i, nfound;
    float sc[16];
    u32 t0, t1;
    Xil_DCacheInvalidateRange(TEST_IMG, 320*320*3);
    jlogf("TEST MODE\r\n");
    preprocess_rgb((const unsigned char *)TEST_IMG, BN_IN_W*3, BN_IN_W, BN_IN_H, nn_in);
    XTime_GetTime(&t0);
    bn_forward(nn_in, nn_out);
    XTime_GetTime(&t1);
    nfound = nms(nn_out, 0.25f, idx, sc, 16);
    jlogf("detections=%d\r\n", nfound);
    /* bench check of the buzzer without a camera scene: if the injected
       test image is a bear, the alarm must sound                     */
    XGpio_Initialize(&axi_gpio_inst, AXI_GPIO_0_ID);
    buzzer_init();
    if (nfound > 0) {
        jlogf("BUZZ: test-mode alarm\r\n");
        buzzer_alarm();
    } else {
        jlogf("BUZZ: test-mode no bear, silent\r\n");
    }
    for (i = 0; i < nfound && i < 8; i++) {
        int k = idx[i];
        jlogf("  #%d conf=%.3f x=%.1f y=%.1f w=%.1f h=%.1f\r\n", i, sc[i],
              nn_out[k], nn_out[BN_ANCHORS+k], nn_out[2*BN_ANCHORS+k], nn_out[3*BN_ANCHORS+k]);
    }
    memcpy((void *)ANNO_BUF, (const void *)TEST_IMG, 320*320*3);
    for (i = 0; i < nfound; i++) {
        int k = idx[i];
        int x0 = (int)(nn_out[k] - nn_out[2*BN_ANCHORS+k]/2), y0 = (int)(nn_out[BN_ANCHORS+k] - nn_out[3*BN_ANCHORS+k]/2);
        int x1 = (int)(nn_out[k] + nn_out[2*BN_ANCHORS+k]/2), y1 = (int)(nn_out[BN_ANCHORS+k] + nn_out[3*BN_ANCHORS+k]/2);
        draw_rect(ANNO_BUF, 320*3, x0, y0, x1, y1, 320, 320, 2);
    }
    Xil_DCacheFlushRange(ANNO_BUF, 320*320*3);
    jlogf("TEST DONE\r\n");
    while (1) sleep(1);
}

int main(void)
{
    u16 w = 480, h = 272, th = 1800, tv = 1000;
    u8 st0;
    int i, nfound, alarm_cnt = 0;

    jlog_init();
    jlogf("=== BEAR DETECTOR ===\r\n");
    bn_init();

    if (*(volatile u32 *)TEST_HDR == TEST_MAGIC &&
        *(volatile u32 *)(TEST_HDR + 4) == 0xA5A5A5A5u) run_test_mode();

    XGpio_Initialize(&axi_gpio_inst, AXI_GPIO_0_ID);
    XGpio_SetDataDirection(&axi_gpio_inst, 1, 0x07);
    lcd_id = LTDC_PanelID_Read(&axi_gpio_inst, 1);
    XGpio_SetDataDirection(&axi_gpio_inst, 1, 0x00);
    jlogf("LCD ID: 0x%x\r\n", lcd_id);

    buzzer_init();
    jlogf("BUZZ: PL E18 (J21 PL_TX) ready, 2 kHz passive tone\r\n");
    buzzer_tone_ms(120);                    /* power-on self test */

    switch (lcd_id) {
        case 0x4342: w = 480;  h = 272; vd_mode = VMODE_480x272;  break;
        case 0x4384: w = 800;  h = 480; vd_mode = VMODE_800x480;  break;
        case 0x7084: w = 800;  h = 480; vd_mode = VMODE_800x480;  break;
        case 0x7016: w = 1024; h = 600; vd_mode = VMODE_1024x600; break;
        case 0x1018: w = 1280; h = 800; vd_mode = VMODE_1280x800; break;
        default:     w = 480;  h = 272; vd_mode = VMODE_480x272;  break;
    }
    jlogf("video mode %dx%d\r\n", w, h);

    emio_init();
    /* 只使用左目（CAM0）：单路整幅采集，右目不再初始化、不再启动 VDMA。
       摄像头输出整幅 w×h，直接填满显示帧缓存，屏幕上就是一路全屏画面。 */
    st0 = ov5640_init(CAM0_CH0, w, h, th, tv);
    jlogf("cam0=%d (single camera / left only)\r\n", st0);

    run_vdma_frame_buffer(&cam0_vdma, CAM0_VDMA_ID, w, h, CAM0_BUF, 0, 0, ONLY_WRITE);

    clk_wiz_cfg(CLK_WIZ_ID, vd_mode.freq);
    DisplayInitialize(&dispCtrl, DISP_VTC_ID);
    DisplaySetMode(&dispCtrl, &vd_mode);
    DisplayStart(&dispCtrl);
    run_vdma_frame_buffer(&disp_vdma, DISP_VDMA_ID, w, h, CAM0_BUF, 0, 0, ONLY_READ);

    g_w = w; g_h = h;
    i = display_timer_init();
    jlogf("preview: live passthrough + frame-synced boxes (%s)\r\n", i == 0 ? "on" : "FAILED");
    jlogf("pipeline running\r\n");

    while (1) {
        int idx[16];
        float sc[16];
        {
            /* average 3 consecutive frames: suppresses the moire banding you get
               when a phone/monitor screen is photographed, and steadies scores */
            int k, n = 3 * BN_IN_W * BN_IN_H;
            preprocess_rgb((const unsigned char *)CAM0_BUF, w*3, w, h, nn_in);
            for (k = 1; k < 3; k++) {
                usleep(30000);
                preprocess_rgb((const unsigned char *)CAM0_BUF, w*3, w, h, nn_tmp);
                for (i = 0; i < n; i++) nn_in[i] += nn_tmp[i];
            }
            for (i = 0; i < n; i++) nn_in[i] *= (1.f / 3.f);
        }
        bn_forward(nn_in, nn_out);
        nfound = nms(nn_out, CONF_THRES, idx, sc, 16);
        /* temporal consistency: keep a box if it is strong, or if it overlaps
           a box kept in the previous inference (suppresses one-off false hits) */
        {
            static float pbox[8][4];
            static int pn = 0;
            int keep_n = 0, ci;
            for (ci = 0; ci < nfound && keep_n < 8; ci++) {
                int k = idx[ci];
                float x0 = (nn_out[k] - nn_out[2*BN_ANCHORS+k]/2 - g_dx) * w / g_nw;
                float y0 = (nn_out[BN_ANCHORS+k] - nn_out[3*BN_ANCHORS+k]/2 - g_dy) * h / g_nh;
                float x1 = (nn_out[k] + nn_out[2*BN_ANCHORS+k]/2 - g_dx) * w / g_nw;
                float y1 = (nn_out[BN_ANCHORS+k] + nn_out[3*BN_ANCHORS+k]/2 - g_dy) * h / g_nh;
                int ok = (sc[ci] >= 0.40f), pj;
                if (!ok) {
                    for (pj = 0; pj < pn; pj++) {
                        float ix0 = x0 > pbox[pj][0] ? x0 : pbox[pj][0];
                        float iy0 = y0 > pbox[pj][1] ? y0 : pbox[pj][1];
                        float ix1 = x1 < pbox[pj][2] ? x1 : pbox[pj][2];
                        float iy1 = y1 < pbox[pj][3] ? y1 : pbox[pj][3];
                        float iw = ix1 - ix0, ih = iy1 - iy0;
                        if (iw > 0 && ih > 0) {
                            float inter = iw * ih;
                            float ua = (x1-x0)*(y1-y0) + (pbox[pj][2]-pbox[pj][0])*(pbox[pj][3]-pbox[pj][1]) - inter;
                            if (ua > 0 && inter / ua > 0.30f) { ok = 1; break; }
                        }
                    }
                }
                if (ok) {
                    g_box[keep_n][0] = x0; g_box[keep_n][1] = y0;
                    g_box[keep_n][2] = x1; g_box[keep_n][3] = y1;
                    pbox[keep_n][0] = x0; pbox[keep_n][1] = y0;
                    pbox[keep_n][2] = x1; pbox[keep_n][3] = y1;
                    keep_n++;
                }
            }
            pn = keep_n;
            g_box_n = keep_n;
            nfound = keep_n;

            if (keep_n > 0) {
                float cx, cy, bw;
                bw = g_box[0][2] - g_box[0][0];
                if (bw < 40.f) bw = 40.f;
                if (bw > 200.f) bw = 200.f;
                cx = (g_box[0][0] + g_box[0][2]) * 0.5f;
                cy = (g_box[0][1] + g_box[0][3]) * 0.5f;
                trk_grab(cx, cy, bw, bw);
            } else {
                g_trk_on = 0;   /* target gone: stop the tracker at once */
            }
        }
        for (i = 0; i < 0; i++) {
            int k = idx[i];
            g_box[i][0] = (nn_out[k] - nn_out[2*BN_ANCHORS+k]/2 - g_dx) * w / g_nw;
            g_box[i][1] = (nn_out[BN_ANCHORS+k] - nn_out[3*BN_ANCHORS+k]/2 - g_dy) * h / g_nh;
            g_box[i][2] = (nn_out[k] + nn_out[2*BN_ANCHORS+k]/2 - g_dx) * w / g_nw;
            g_box[i][3] = (nn_out[BN_ANCHORS+k] + nn_out[3*BN_ANCHORS+k]/2 - g_dy) * h / g_nh;
        }
        g_box_n = (nfound > 8) ? 8 : nfound;

        if (nfound > 0) alarm_cnt++; else alarm_cnt = 0;

        /* ---- buzzer alarm ----
           the tone itself is generated by the 4 kHz timer, so a detection
           only has to start / extend it: the alarm sounds from the very
           first frame that contains the bear and keeps sounding (hold)
           until the bear has been gone for ALARM_HOLD_MS.             */
        {
            static int prev_det = 0;
            int det = (nfound > 0);
            if (det) {
                alarm_start(ALARM_HOLD_MS);
                if (!prev_det) jlogf("BUZZ: bear detected -> continuous alarm\r\n");
            } else if (prev_det) {
                jlogf("BUZZ: target gone, alarm stops after %d ms\r\n", ALARM_HOLD_MS);
            }
            prev_det = det;
        }

        jlog_rotate();
        {
            float mx = 0.f;
            int q;
            for (q = 0; q < BN_ANCHORS; q++) if (nn_out[4*BN_ANCHORS+q] > mx) mx = nn_out[4*BN_ANCHORS+q];
            if (alarm_cnt >= ALARM_FRAMES)
                jlogf("ALARM bear det=%d conf=%.2f max=%.2f trk=%d\r\n", nfound, nfound ? sc[0] : 0.f, mx, (int)g_trk_on);
            else
                jlogf("frame det=%d max=%.2f trk=%d cx=%d cy=%d\r\n", nfound, mx, (int)g_trk_on, (int)g_trk_cx, (int)g_trk_cy);
        }
        usleep(20000);      /* small pacing delay: keep AXI/DDR for the LCD */
    }
    return 0;
}
