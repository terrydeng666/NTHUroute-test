#ifndef INC_FLUTE_H
#define INC_FLUTE_H

#define D 9         // LUT is used for d <= D, D <= 9
#define ROUTING 1   // 1 to construct routing, 0 to estimate WL only
#define REMOVE_DUPLICATE_PIN 1  // Remove dup. pin for flute_wl() & flute()
#define LOCAL_REFINEMENT 0      // Suggestion: Set to 1 if ACCURACY >= 5

#if D<=7
#define MGROUP 5040/4  // Max. # of groups, 7! = 5040
#define MPOWV 15  // Max. # of POWVs per group
#elif D==8
#define MGROUP 40320/4  // Max. # of groups, 8! = 40320
#define MPOWV 33  // Max. # of POWVs per group
#elif D==9
#define MGROUP 362880/4  // Max. # of groups, 9! = 362880
#define MPOWV 79  // Max. # of POWVs per group
#endif

#include "flute-ds.h"           // flute data structure
#include "flute-function.h"

#if REMOVE_DUPLICATE_PIN==1
  #define flutes_wl(d, xs, ys, s, acc) flutes_wl_RDP(d, xs, ys, s, acc) 
  #define flutes(d, xs, ys, s, acc) flutes_RDP(d, xs, ys, s, acc) 
#else
  #define flutes_wl(d, xs, ys, s, acc) flutes_wl_ALLD(d, xs, ys, s, acc) 
  #define flutes(d, xs, ys, s, acc) flutes_ALLD(d, xs, ys, s, acc) 
#endif

#define flutes_wl_ALLD(d, xs, ys, s, acc) flutes_wl_LMD(d, xs, ys, s, acc)
#define flutes_ALLD(d, xs, ys, s, acc) flutes_LMD(d, xs, ys, s, acc)

#define flutes_wl_LMD(d, xs, ys, s, acc) \
    (d<=D ? flutes_wl_LD(d, xs, ys, s) : flutes_wl_MD(d, xs, ys, s, acc))
#define flutes_LMD(d, xs, ys, s, acc) \
    (d<=D ? flutes_LD(d, xs, ys, s) : flutes_MD(d, xs, ys, s, acc))

#define max(x,y) ((x)>(y)?(x):(y))
#define min(x,y) ((x)<(y)?(x):(y))
#define abs(x) ((x)<0?(-(x)):(x))
#define ADIFF(x,y) ((x)>(y)?(x-y):(y-x))  // Absolute difference

struct csoln 
{
    unsigned char parent;
    unsigned char seg[11];  // Add: 0..i, Sub: j..10; seg[i+1]=seg[j-1]=0
    unsigned char rowcol[D-2];  // row = rowcol[]/16, col = rowcol[]%16, 
    unsigned char neighbor[2*D-2];
};

extern struct csoln *LUT[D+1][MGROUP];

#endif //INC_FLUTE_H
