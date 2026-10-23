#include <stdio.h>
#include <stdlib.h>
#include "../../app_program/subsystem3/barcode.h"
static const char *L="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
static const int P[]={0x034,0x121,0x061,0x160,0x031,0x130,0x070,0x025,0x124,0x064,0x109,0x049,0x148,0x019,0x118,0x058,0x00D,0x10C,0x04C,0x01C,0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,0x181,0x0C1,0x1C0,0x091,0x190,0x0D0,0x085,0x184,0x0C4,0x0A8,0x0A2,0x08A,0x02A};
static double rnd(double a,double b){return a+(b-a)*rand()/(double)RAND_MAX;}
/* stream: list of (black,width_mm) segments after a 60 mm quiet; returns decoded char or 0 / '?' */
static char feed(double *seg,int n,double v0,double v1,double noise){
 barcode_reset(); double total=0; for(int i=0;i<n;i++) total+=seg[i];
 double pos=-60,end=total+80; UB letter=0; char got=0;
 while(pos<end){double f=(pos+60)/(end+60); double v=v0+(v1-v0)*f; pos+=v*0.001;
  int black=0; if(pos>=0&&pos<total){double acc=0; for(int i=0;i<n;i++){acc+=seg[i]; if(pos<acc){black=(i%2==0);break;}}}
  if(rnd(0,1)<noise) black=!black;
  int r=barcode_feed(black,&letter); if(r==BCR_DECODED&&!got) got=letter;}
 return got;}
int main(){srand(1); int ok=0,wrong=0,miss=0,N=0;
 for(int t=0;t<3000;t++){int k=rand()%43; double nar=rnd(3,6), ratio=rnd(2.5,3.0); double v=rnd(150,500); double v1=v*rnd(0.85,1.15);
  int pats[3]={0x094,P[k],0x094}; double seg[29]; int n=0;
  for(int c=0;c<3;c++){for(int e=0;e<9;e++) seg[n++]=((pats[c]>>(8-e))&1)?nar*ratio*rnd(0.9,1.1):nar*rnd(0.9,1.1); if(c<2) seg[n++]=nar*rnd(0.9,1.1);}
  if(rand()%2){for(int i=0;i<14;i++){double x=seg[i];seg[i]=seg[28-i];seg[28-i]=x;}}
  char g=feed(seg,29,v,v1,0.002); N++; if(g==L[k]) ok++; else if(g) {wrong++; if(wrong<5) printf("wrong: %c->%c\n",L[k],g);} else miss++;}
 printf("random symbols: %d ok, %d wrong, %d missed of %d\n",ok,wrong,miss,N);
 int fp=0; for(int t=0;t<500;t++){double seg[1]={rnd(15,25)}; if(feed(seg,1,300,300,0.002)) fp++;}
 printf("junction stripes decoded as barcodes: %d of 500\n",fp);
 int part=0; for(int t=0;t<500;t++){int pats[3]={0x094,0x109,0x094}; double seg[29]; int n=0; for(int c=0;c<3;c++){for(int e=0;e<9;e++) seg[n++]=((pats[c]>>(8-e))&1)?9:3; if(c<2) seg[n++]=3;}
   if(feed(seg,15+rand()%13,300,300,0.0)) part++;}
 printf("cut-off barcodes decoded: %d of 500\n",part); return 0;}
