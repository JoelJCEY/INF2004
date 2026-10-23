#include <stdio.h>
#include "../../app_program/subsystem1/subsystem1.h"
int main(){ S1_TELEMETRY t={0}; UB b[S1_PACKED_BYTES];
 t.seq=0x82; t.state=2; t.last_barcode=3; t.last_letter='A'; t.fault_mask=5; t.line_flags=3; t.line_err=-125;
 t.speed_l_mm_s=290; t.speed_r_mm_s=310; t.distance_mm=12340; t.enc_l=4660; t.enc_r=4672; t.hump_peak_mm=42;
 t.obstacle_mm=1990; t.obstacle_width_mm=210; t.bypass_side=2; t.motion_event=1;
 s1_pack(&t,b); printf("C pack: "); for(int i=0;i<S1_PACKED_BYTES;i++) printf("%02X ",b[i]); printf("\n");
 printf("{ "); for(int i=0;i<S1_PACKED_BYTES;i++) printf("0x%02X%s",b[i],i<S1_PACKED_BYTES-1?", ":""); printf(" }\n"); return 0;}
