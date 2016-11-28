/*******************************************************************************
* Piotr's Computer Vision Matlab Toolbox      Version 3.23
* Copyright 2014 Piotr Dollar.  [pdollar-at-gmail.com]
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/
#ifndef _SSE_HPP_
#define _SSE_HPP_
#include <xmmintrin.h> // SSE2:<e*.h>, SSE3:<p*.h>, SSE4:<s*.h>
#define RETf  __m128
#define RETi  __m128i

// set, load and store values
inline RETf SET( const float &x ) { return _mm_set1_ps(x); }
inline RETf SET( float x, float y, float z, float w ) { return _mm_set_ps(x,y,z,w); }
inline RETi SET( const int &x ) { return _mm_set1_epi32(x); }
inline RETf LD( const float &x ) { return _mm_load_ps(&x); }
inline RETf LDu( const float &x ) { return _mm_loadu_ps(&x); }
inline RETf STR( float &x, const __m128 y ) { _mm_store_ps(&x,y); return y; }
inline RETf STR1( float &x, const __m128 y ) { _mm_store_ss(&x,y); return y; }
inline RETf STRu( float &x, const __m128 y ) { _mm_storeu_ps(&x,y); return y; }
inline RETf STR( float &x, const float y ) { return STR(x,SET(y)); }

// arithmetic operators
inline RETi ADD( const __m128i x, const __m128i y ) { return _mm_add_epi32(x,y); }
inline RETf ADD( const __m128 x, const __m128 y ) { return _mm_add_ps(x,y); }
inline RETf ADD( const __m128 x, const __m128 y, const __m128 z ) {return ADD(ADD(x,y),z); }
inline RETf ADD( const __m128 a, const __m128 b, const __m128 c, const __m128 &d ) {return ADD(ADD(ADD(a,b),c),d); }
inline RETf SUB( const __m128 x, const __m128 y ) { return _mm_sub_ps(x,y); }
inline RETf MUL( const __m128 x, const __m128 y ) { return _mm_mul_ps(x,y); }
inline RETf MUL( const __m128 x, const float y ) { return MUL(x,SET(y)); }
inline RETf MUL( const float x, const __m128 y ) { return MUL(SET(x),y); }
inline RETf INC( __m128 &x, const __m128 y ) { return x = ADD(x,y); }
inline RETf INC( float &x, const __m128 y ) { __m128 t=ADD(LD(x),y); return STR(x,t); }
inline RETf DEC( __m128 &x, const __m128 y ) { return x = SUB(x,y); }
inline RETf DEC( float &x, const __m128 y ) { __m128 t=SUB(LD(x),y); return STR(x,t); }
inline RETf MINSSE( const __m128 x, const __m128 y ) { return _mm_min_ps(x,y); }
inline RETf RCP( const __m128 x ) { return _mm_rcp_ps(x); }
inline RETf RCPSQRT( const __m128 x ) { return _mm_rsqrt_ps(x); }

// logical operators
inline RETf AND( const __m128 x, const __m128 y ) { return _mm_and_ps(x,y); }
inline RETi AND( const __m128i x, const __m128i y ) { return _mm_and_si128(x,y); }
inline RETf ANDNOT( const __m128 x, const __m128 y ) { return _mm_andnot_ps(x,y); }
inline RETf OR( const __m128 x, const __m128 y ) { return _mm_or_ps(x,y); }
inline RETf XOR( const __m128 x, const __m128 y ) { return _mm_xor_ps(x,y); }

// comparison operators
inline RETf CMPGT( const __m128 x, const __m128 y ) { return _mm_cmpgt_ps(x,y); }
inline RETf CMPLT( const __m128 x, const __m128 y ) { return _mm_cmplt_ps(x,y); }
inline RETi CMPGT( const __m128i x, const __m128i y ) { return _mm_cmpgt_epi32(x,y); }
inline RETi CMPLT( const __m128i x, const __m128i y ) { return _mm_cmplt_epi32(x,y); }

// conversion operators
inline RETf CVT( const __m128i x ) { return _mm_cvtepi32_ps(x); }
inline RETi CVT( const __m128 x ) { return _mm_cvttps_epi32(x); }

#undef RETf
#undef RETi
#endif
