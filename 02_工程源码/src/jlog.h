#ifndef JLOG_H_
#define JLOG_H_

#include "xil_types.h"

/* Shared-memory log channel readable over JTAG (xsdb mrd) */
#define JLOG_PHYS_ADDR   0x0F000000u
#define JLOG_MAGIC       0x4A4C4F47u   /* "JLOG" */
#define JLOG_CAP         (192*1024)

typedef struct {
    volatile u32 magic;
    volatile u32 len;
    volatile u32 seq;
    char         buf[JLOG_CAP];
} jlog_t;

void jlog_init(void);
void jlog_puts(const char *s);
void jlogf(const char *fmt, ...);
void jlog_rotate(void);

#endif
