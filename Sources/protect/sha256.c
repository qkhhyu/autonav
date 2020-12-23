/*
* SHA256散列算法
* <kerndev@foxmail.com>
* 参考:http://csrc.nist.gov/publications/fips/fips180-2/fips180-2.pdf
*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sha256.h"

static const uint32_t H[8]=
{
	0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
};

static const uint32_t K[64]=
{
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

static uint32_t W[64];
static uint32_t M[16];
static uint32_t MAC[8];

//SHA256移位
static uint32_t sha256_shift(uint32_t input, int op1, int op2, int op3) 
{   
    uint32_t output;
    
    output = 0;
	output ^= ((input >> op1)|(input<<(32-op1)));     
	output ^= ((input >> op2)|(input<<(32-op2)));     

	if(op3 > 0)         
	{
		output ^= ((input >> op3)|(input<<(32-op3)));     
	}
	else     
	{                    
		output ^= (input >> (0-op3));     
	} 

    return output; 
}  

//生成16个M
//FIXME:len固定长度
static void sha256_input(unsigned char* input, int len)
{
	int i;
	for(i=0; i<14; i++)
	{
			M[15-i]  = input[i * 4];
			M[15-i] |= input[i * 4 + 1] << 8;
			M[15-i] |= input[i * 4 + 2] << 16;
			M[15-i] |= input[i * 4 + 3] << 24;	
	}
	M[1] = 0x80000000;
	M[0] = 0x000001b8;	
}


//生成64个W
static void sha256_prepare(void)
{
	int i; 
	uint32_t s1;
	uint32_t s2;

	for(i = 0; i < 16; i++)
	{
		W[i] = M[i];
	}
	for(i = 16; i < 64; i++)
	{       
		s1 = 0;
		s2 = 0;
		s1 = sha256_shift(W[i-2],17,19,-10);
		s2 = sha256_shift(W[i-15],7,18,-3);
		W[i] = (s1 + W[i-7] + s2 + W[i-16]);
	} 
}

//由W生成M
static void sha256_round(void)
{
	int i; 
	uint32_t A,B,C,D,E,F,G,I;    
	uint32_t t1,t2,s1,s2;

	A = H[0];
	B = H[1];
	C = H[2];
	D = H[3];
	E = H[4];
	F = H[5];
	G = H[6];
	I = H[7];

	for(i = 0; i < 64; i++)
	{
			s1 = sha256_shift(A, 2, 13, 22);
			t2 = s1 + ((A&B)^(A&C)^(B&C));
			s2 = sha256_shift(E, 6, 11, 25);
			t1 = I + s2 + ((E&F)^((~E)&G)) + K[i] + W[i];
			I  = G;
			G  = F;
			F  = E;
			E  = D + t1;
			D  = C;
			C  = B;
			B  = A;
			A  = t1 + t2;
	}
	MAC[0] = H[0]+A;
	MAC[1] = H[1]+B;
	MAC[2] = H[2]+C;
	MAC[3] = H[3]+D;
	MAC[4] = H[4]+E;
	MAC[5] = H[5]+F;
	MAC[6] = H[6]+G;
	MAC[7] = H[7]+I;
}

//SHA256算法接口
//output:输出32字节SHA256摘要
//input:待校验的数据
//len:数据长度(小于56字节)
void sha256(uint32_t output[8], void* input, int len)
{
	sha256_input(input,len);
	sha256_prepare();
	sha256_round();
	memcpy(output,MAC,32);
}
