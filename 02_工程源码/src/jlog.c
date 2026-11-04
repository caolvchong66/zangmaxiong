#include "jlog.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "xil_cache.h"

static jlog_t *jl = (jlog_t *)JLOG_PHYS_ADDR;

void jlog_init(void)
{
    jl->magic = 0;
    jl->len = 0;
    jl->seq = 0;
}

static void jl_flush(void)
{
    u32 n = jl->len + 16;
    if (n > JLOG_CAP + 16) n = JLOG_CAP + 16;
    jl->seq++;
    Xil_DCacheFlushRange((UINTPTR)jl, n);
}

void jlog_rotate(void)
{
    if (jl->len > (JLOG_CAP - 4096)) {
        jl->len = 0;
        jl->buf[0] = 0;
    }
}

void jlog_puts(const char *s)
{
    int n = (int)strlen(s);
    if (jl->len + n < JLOG_CAP) {
        memcpy((void *)(jl->buf + jl->len), s, n);
        jl->len += n;
        jl->buf[jl->len] = 0;
    }
    jl->magic = JLOG_MAGIC;
    jl_flush();
}

void jlogf(const char *fmt, ...)
{
    va_list ap;
    int n;
    if (jl->len >= JLOG_CAP) return;
    va_start(ap, fmt);
    n = vsnprintf((char *)(jl->buf + jl->len), JLOG_CAP - jl->len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        jl->len += (u32)n;
        if (jl->len > JLOG_CAP) jl->len = JLOG_CAP;
        jl->buf[JLOG_CAP-1] = 0;
    }
    jl->magic = JLOG_MAGIC;
    jl_flush();
}
