/* Host shim of the micro T-Kernel API: just enough for the pure logic and
   the subsystem tests to run on a PC.  Not part of the firmware. */
#ifndef HOST_TKERNEL_H
#define HOST_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int8_t B; typedef int16_t H; typedef int32_t W;
typedef uint8_t UB; typedef uint16_t UH; typedef uint32_t UW;
typedef int INT; typedef unsigned int UINT; typedef INT BOOL; typedef INT ID; typedef INT ER;
typedef INT PRI; typedef UW ATR; typedef INT SZ; typedef UW RELTIM; typedef W TMO;
typedef void (*FP)();
#define TRUE 1
#define FALSE 0
#define LOCAL static
#define EXPORT
#define IMPORT extern
#define CONST const
#define Inline static inline
#define E_OK 0
#define E_PAR (-17)
#define E_LIMIT (-34)
#define E_OBJ (-41)
#define E_IO (-57)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define TWF_ORW 1
#define TP_PRC1 1
#define TP_PRC2 2
typedef struct { void *exinf; ID wtsk; UINT flgptn; } T_RFLG;
typedef struct { W hi; UW lo; } SYSTIM;
ER tk_dis_dsp(void); ER tk_ena_dsp(void);
ER tk_dly_tsk(RELTIM t); ER tk_ref_flg(ID id, T_RFLG *r); ER tk_set_flg(ID id, UINT p);
ER tk_snd_mbf(ID id, const void *m, INT sz, TMO t); INT tk_rcv_mbf(ID id, void *m, TMO t);
ER tk_wup_tsk(ID id); INT tk_can_wup(ID id); ER tk_slp_tsk(TMO t);
UW in_w(UW a); void out_w(UW a, UW v); void set_w(UW a, UW v); void clr_w(UW a, UW v);
typedef struct { ATR intatr; FP inthdr; } T_DINT;
ER tk_def_int(UINT n, const T_DINT *d); void EnableInt(UINT n, INT l); void ClearInt(UINT n);
#define TA_HLNG 1
#define TK_SUPPORT_SMP 0
#endif
