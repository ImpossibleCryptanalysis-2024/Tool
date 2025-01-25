#include <iomanip>
#include <iostream>
#include <cmath>
#include <vector>
using namespace std;

#define N 8
#define siz 256

// skinny128-8bit
const int skinny8[256] = {
0x65 ,0x4c ,0x6a ,0x42 ,0x4b ,0x63 ,0x43 ,0x6b ,0x55 ,0x75 ,0x5a ,0x7a ,0x53 ,0x73 ,0x5b ,0x7b ,
0x35 ,0x8c ,0x3a ,0x81 ,0x89 ,0x33 ,0x80 ,0x3b ,0x95 ,0x25 ,0x98 ,0x2a ,0x90 ,0x23 ,0x99 ,0x2b ,
0xe5 ,0xcc ,0xe8 ,0xc1 ,0xc9 ,0xe0 ,0xc0 ,0xe9 ,0xd5 ,0xf5 ,0xd8 ,0xf8 ,0xd0 ,0xf0 ,0xd9 ,0xf9 ,
0xa5 ,0x1c ,0xa8 ,0x12 ,0x1b ,0xa0 ,0x13 ,0xa9 ,0x05 ,0xb5 ,0x0a ,0xb8 ,0x03 ,0xb0 ,0x0b ,0xb9 ,
0x32 ,0x88 ,0x3c ,0x85 ,0x8d ,0x34 ,0x84 ,0x3d ,0x91 ,0x22 ,0x9c ,0x2c ,0x94 ,0x24 ,0x9d ,0x2d ,
0x62 ,0x4a ,0x6c ,0x45 ,0x4d ,0x64 ,0x44 ,0x6d ,0x52 ,0x72 ,0x5c ,0x7c ,0x54 ,0x74 ,0x5d ,0x7d ,
0xa1 ,0x1a ,0xac ,0x15 ,0x1d ,0xa4 ,0x14 ,0xad ,0x02 ,0xb1 ,0x0c ,0xbc ,0x04 ,0xb4 ,0x0d ,0xbd ,
0xe1 ,0xc8 ,0xec ,0xc5 ,0xcd ,0xe4 ,0xc4 ,0xed ,0xd1 ,0xf1 ,0xdc ,0xfc ,0xd4 ,0xf4 ,0xdd ,0xfd ,
0x36 ,0x8e ,0x38 ,0x82 ,0x8b ,0x30 ,0x83 ,0x39 ,0x96 ,0x26 ,0x9a ,0x28 ,0x93 ,0x20 ,0x9b ,0x29 ,
0x66 ,0x4e ,0x68 ,0x41 ,0x49 ,0x60 ,0x40 ,0x69 ,0x56 ,0x76 ,0x58 ,0x78 ,0x50 ,0x70 ,0x59 ,0x79 ,
0xa6 ,0x1e ,0xaa ,0x11 ,0x19 ,0xa3 ,0x10 ,0xab ,0x06 ,0xb6 ,0x08 ,0xba ,0x00 ,0xb3 ,0x09 ,0xbb ,
0xe6 ,0xce ,0xea ,0xc2 ,0xcb ,0xe3 ,0xc3 ,0xeb ,0xd6 ,0xf6 ,0xda ,0xfa ,0xd3 ,0xf3 ,0xdb ,0xfb ,
0x31 ,0x8a ,0x3e ,0x86 ,0x8f ,0x37 ,0x87 ,0x3f ,0x92 ,0x21 ,0x9e ,0x2e ,0x97 ,0x27 ,0x9f ,0x2f ,
0x61 ,0x48 ,0x6e ,0x46 ,0x4f ,0x67 ,0x47 ,0x6f ,0x51 ,0x71 ,0x5e ,0x7e ,0x57 ,0x77 ,0x5f ,0x7f ,
0xa2 ,0x18 ,0xae ,0x16 ,0x1f ,0xa7 ,0x17 ,0xaf ,0x01 ,0xb2 ,0x0e ,0xbe ,0x07 ,0xb7 ,0x0f ,0xbf ,
0xe2 ,0xca ,0xee ,0xc6 ,0xcf ,0xe7 ,0xc7 ,0xef ,0xd2 ,0xf2 ,0xde ,0xfe ,0xd7 ,0xf7 ,0xdf ,0xff
};
const int skinny8_inv[256] = {
0xac ,0xe8 ,0x68 ,0x3c ,0x6c ,0x38 ,0xa8 ,0xec ,0xaa ,0xae ,0x3a ,0x3e ,0x6a ,0x6e ,0xea ,0xee ,
0xa6 ,0xa3 ,0x33 ,0x36 ,0x66 ,0x63 ,0xe3 ,0xe6 ,0xe1 ,0xa4 ,0x61 ,0x34 ,0x31 ,0x64 ,0xa1 ,0xe4 ,
0x8d ,0xc9 ,0x49 ,0x1d ,0x4d ,0x19 ,0x89 ,0xcd ,0x8b ,0x8f ,0x1b ,0x1f ,0x4b ,0x4f ,0xcb ,0xcf ,
0x85 ,0xc0 ,0x40 ,0x15 ,0x45 ,0x10 ,0x80 ,0xc5 ,0x82 ,0x87 ,0x12 ,0x17 ,0x42 ,0x47 ,0xc2 ,0xc7 ,
0x96 ,0x93 ,0x03 ,0x06 ,0x56 ,0x53 ,0xd3 ,0xd6 ,0xd1 ,0x94 ,0x51 ,0x04 ,0x01 ,0x54 ,0x91 ,0xd4 ,
0x9c ,0xd8 ,0x58 ,0x0c ,0x5c ,0x08 ,0x98 ,0xdc ,0x9a ,0x9e ,0x0a ,0x0e ,0x5a ,0x5e ,0xda ,0xde ,
0x95 ,0xd0 ,0x50 ,0x05 ,0x55 ,0x00 ,0x90 ,0xd5 ,0x92 ,0x97 ,0x02 ,0x07 ,0x52 ,0x57 ,0xd2 ,0xd7 ,
0x9d ,0xd9 ,0x59 ,0x0d ,0x5d ,0x09 ,0x99 ,0xdd ,0x9b ,0x9f ,0x0b ,0x0f ,0x5b ,0x5f ,0xdb ,0xdf ,
0x16 ,0x13 ,0x83 ,0x86 ,0x46 ,0x43 ,0xc3 ,0xc6 ,0x41 ,0x14 ,0xc1 ,0x84 ,0x11 ,0x44 ,0x81 ,0xc4 ,
0x1c ,0x48 ,0xc8 ,0x8c ,0x4c ,0x18 ,0x88 ,0xcc ,0x1a ,0x1e ,0x8a ,0x8e ,0x4a ,0x4e ,0xca ,0xce ,
0x35 ,0x60 ,0xe0 ,0xa5 ,0x65 ,0x30 ,0xa0 ,0xe5 ,0x32 ,0x37 ,0xa2 ,0xa7 ,0x62 ,0x67 ,0xe2 ,0xe7 ,
0x3d ,0x69 ,0xe9 ,0xad ,0x6d ,0x39 ,0xa9 ,0xed ,0x3b ,0x3f ,0xab ,0xaf ,0x6b ,0x6f ,0xeb ,0xef ,
0x26 ,0x23 ,0xb3 ,0xb6 ,0x76 ,0x73 ,0xf3 ,0xf6 ,0x71 ,0x24 ,0xf1 ,0xb4 ,0x21 ,0x74 ,0xb1 ,0xf4 ,
0x2c ,0x78 ,0xf8 ,0xbc ,0x7c ,0x28 ,0xb8 ,0xfc ,0x2a ,0x2e ,0xba ,0xbe ,0x7a ,0x7e ,0xfa ,0xfe ,
0x25 ,0x70 ,0xf0 ,0xb5 ,0x75 ,0x20 ,0xb0 ,0xf5 ,0x22 ,0x27 ,0xb2 ,0xb7 ,0x72 ,0x77 ,0xf2 ,0xf7 ,
0x2d ,0x79 ,0xf9 ,0xbd ,0x7d ,0x29 ,0xb9 ,0xfd ,0x2b ,0x2f ,0xbb ,0xbf ,0x7b ,0x7f ,0xfb ,0xff
};
// skinny64-4bit
const int skinny4[16] = { 0xc,0x6,0x9,0x0,0x1,0xa,0x2,0xb,0x3,0x8,0x5,0xd,0x4,0xe,0x7,0xf };
const int skinny4_inv[16] = { 0x3,0x4,0x6,0x8,0xc,0xa,0x1,0xe,0x9,0x2,0x5,0x7,0x0,0xb,0xd,0xf };
const int s_4bittest[16] = { 0x0, 0x1, 0x2, 0x5, 0x3, 0x4, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf };
const int s_4bittest_inv[16] = { 0, 1, 2, 4, 5, 3, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

const int aes[256] =
{ 0x63 ,0x7c ,0x77 ,0x7b ,0xf2 ,0x6b ,0x6f ,0xc5 ,0x30 ,0x01 ,0x67 ,0x2b ,0xfe ,0xd7 ,0xab ,0x76
,0xca ,0x82 ,0xc9 ,0x7d ,0xfa ,0x59 ,0x47 ,0xf0 ,0xad ,0xd4 ,0xa2 ,0xaf ,0x9c ,0xa4 ,0x72 ,0xc0
,0xb7 ,0xfd ,0x93 ,0x26 ,0x36 ,0x3f ,0xf7 ,0xcc ,0x34 ,0xa5 ,0xe5 ,0xf1 ,0x71 ,0xd8 ,0x31 ,0x15
,0x04 ,0xc7 ,0x23 ,0xc3 ,0x18 ,0x96 ,0x05 ,0x9a ,0x07 ,0x12 ,0x80 ,0xe2 ,0xeb ,0x27 ,0xb2 ,0x75
,0x09 ,0x83 ,0x2c ,0x1a ,0x1b ,0x6e ,0x5a ,0xa0 ,0x52 ,0x3b ,0xd6 ,0xb3 ,0x29 ,0xe3 ,0x2f ,0x84
,0x53 ,0xd1 ,0x00 ,0xed ,0x20 ,0xfc ,0xb1 ,0x5b ,0x6a ,0xcb ,0xbe ,0x39 ,0x4a ,0x4c ,0x58 ,0xcf
,0xd0 ,0xef ,0xaa ,0xfb ,0x43 ,0x4d ,0x33 ,0x85 ,0x45 ,0xf9 ,0x02 ,0x7f ,0x50 ,0x3c ,0x9f ,0xa8
,0x51 ,0xa3 ,0x40 ,0x8f ,0x92 ,0x9d ,0x38 ,0xf5 ,0xbc ,0xb6 ,0xda ,0x21 ,0x10 ,0xff ,0xf3 ,0xd2
,0xcd ,0x0c ,0x13 ,0xec ,0x5f ,0x97 ,0x44 ,0x17 ,0xc4 ,0xa7 ,0x7e ,0x3d ,0x64 ,0x5d ,0x19 ,0x73
,0x60 ,0x81 ,0x4f ,0xdc ,0x22 ,0x2a ,0x90 ,0x88 ,0x46 ,0xee ,0xb8 ,0x14 ,0xde ,0x5e ,0x0b ,0xdb
,0xe0 ,0x32 ,0x3a ,0x0a ,0x49 ,0x06 ,0x24 ,0x5c ,0xc2 ,0xd3 ,0xac ,0x62 ,0x91 ,0x95 ,0xe4 ,0x79
,0xe7 ,0xc8 ,0x37 ,0x6d ,0x8d ,0xd5 ,0x4e ,0xa9 ,0x6c ,0x56 ,0xf4 ,0xea ,0x65 ,0x7a ,0xae ,0x08
,0xba ,0x78 ,0x25 ,0x2e ,0x1c ,0xa6 ,0xb4 ,0xc6 ,0xe8 ,0xdd ,0x74 ,0x1f ,0x4b ,0xbd ,0x8b ,0x8a
,0x70 ,0x3e ,0xb5 ,0x66 ,0x48 ,0x03 ,0xf6 ,0x0e ,0x61 ,0x35 ,0x57 ,0xb9 ,0x86 ,0xc1 ,0x1d ,0x9e
,0xe1 ,0xf8 ,0x98 ,0x11 ,0x69 ,0xd9 ,0x8e ,0x94 ,0x9b ,0x1e ,0x87 ,0xe9 ,0xce ,0x55 ,0x28 ,0xdf
,0x8c ,0xa1 ,0x89 ,0x0d ,0xbf ,0xe6 ,0x42 ,0x68 ,0x41 ,0x99 ,0x2d ,0x0f ,0xb0 ,0x54 ,0xbb ,0x16 };
const int aes_inv[256] = { 82, 9, 106, 213, 48, 54, 165, 56, 191, 64, 163, 158, 129, 243, 215, 251, 124, 227, 57, 130, 155, 47, 255, 135, 52,
142, 67, 68, 196, 222, 233, 203, 84, 123, 148, 50, 166, 194, 35, 61, 238, 76, 149, 11, 66, 250, 195, 78, 8, 46, 161, 102, 40, 217, 36,
178, 118, 91, 162, 73, 109, 139, 209, 37, 114, 248, 246, 100, 134, 104, 152, 22, 212, 164, 92, 204, 93, 101, 182, 146, 108, 112, 72, 80,
253, 237, 185, 218, 94, 21, 70, 87, 167, 141, 157, 132, 144, 216, 171, 0, 140, 188, 211, 10, 247, 228, 88, 5, 184, 179, 69, 6, 208, 44,
30, 143, 202, 63, 15, 2, 193, 175, 189, 3, 1, 19, 138, 107, 58, 145, 17, 65, 79, 103, 220, 234, 151, 242, 207, 206, 240, 180, 230, 115,
150, 172, 116, 34, 231, 173, 53, 133, 226, 249, 55, 232, 28, 117, 223, 110, 71, 241, 26, 113, 29, 41, 197, 137, 111, 183, 98, 14, 170,
24, 190, 27, 252, 86, 62, 75, 198, 210, 121, 32, 154, 219, 192, 254, 120, 205, 90, 244, 31, 221, 168, 51, 136, 7, 199, 49, 177, 18, 16,
89, 39, 128, 236, 95, 96, 81, 127, 169, 25, 181, 74, 13, 45, 229, 122, 159, 147, 201, 156, 239, 160, 224, 59, 77, 174, 42, 245, 176, 200,
235, 187, 60, 131, 83, 153, 97, 23, 43, 4, 126, 186, 119, 214, 38, 225, 105, 20, 99, 85, 33, 12, 125 };

const int s_5bittest[32] = { 29, 19, 2, 23, 12, 14, 7, 11, 16, 0, 8, 30, 6, 26, 17, 31, 1, 27, 5, 15, 20, 9, 24, 22, 4, 18, 25, 3, 21, 13, 10, 28 };
const int s_5bittest_inv[32] = { 9, 16, 2, 27, 24, 18, 12, 6, 10, 21, 30, 7, 4, 29, 5, 19, 8, 14, 25, 1, 20, 28, 23, 3, 22, 26, 13, 17, 31, 0, 11, 15 };


void Sbox_DDT(vector<vector<int>>& DDT, const int sbox[siz]) {
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            for (int x = 0; x < siz; x++)
                if ((sbox[x] ^ sbox[x ^ a]) == b) DDT[a][b]++;
        }
    }
}

void Sbox_BCT(vector<vector<int>>& BCT, const int sbox[siz], const int sbox_inv[siz]) {
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            for (int x = 0; x < siz; x++)
                if ((sbox_inv[sbox[x] ^ b] ^ sbox_inv[sbox[x ^ a] ^ b]) == a) BCT[a][b]++;
        }
    }
}

// [LWL22]
void Sbox_GBCT(vector<vector<vector<vector<int>>>>& GBCT, const int sbox[siz], const int sbox_inv[siz]) {
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            for (int c = 0; c < siz; c++) {
                for (int x = 0; x < siz; x++) {
                    GBCT[a][b][c][(sbox_inv[sbox[x] ^ b] ^ sbox_inv[sbox[x ^ a] ^ c])]++;
                }
            }
        }
    }
}

// [WP19], [DDV20]
void Sbox_UBCT(vector<vector<vector<int>>>& UBCT, const int sbox[siz], const int sbox_inv[siz]) {
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            for (int c = 0; c < siz; c++) {
                for (int x = 0; x < siz; x++) {
                    int flag1 = 0; int flag2 = 0;
                    if ((sbox_inv[sbox[x] ^ c] ^ sbox_inv[sbox[x ^ a] ^ c]) == a) flag1 = 1;
                    if ((sbox[x] ^ sbox[x ^ a]) == b) flag2 = 1;
                    if (flag1 && flag2) UBCT[a][b][c]++;
                }
            }
        }
    }
}

// [WP19], [DDV20]
void Sbox_LBCT(vector<vector<vector<int>>>& LBCT, const int sbox[siz], const int sbox_inv[siz]) {
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            for (int c = 0; c < siz; c++) {
                for (int x = 0; x < siz; x++) {
                    int flag1 = 0; int flag2 = 0;
                    if ((sbox_inv[sbox[x] ^ c] ^ sbox_inv[sbox[x ^ a] ^ c]) == a) flag1 = 1;
                    if ((sbox[x] ^ sbox[x ^ b]) == c) flag2 = 1;
                    if (flag1 && flag2) LBCT[a][b][c]++;
                }
            }
        }
    }
}

// [DDV20]
void Sbox_EBCT(vector<vector<vector<vector<int>>>>& EBCT, const int sbox[siz], const int sbox_inv[siz]) {
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            for (int c = 0; c < siz; c++) {
                for (int d = 0; d < siz; d++) {
                    for (int x = 0; x < siz; x++) {
                        int flag1 = 0; int flag2 = 0; int flag3 = 0;
                        if ((sbox_inv[sbox[x] ^ d] ^ sbox_inv[sbox[x ^ a] ^ d]) == a) flag1 = 1;
                        if ((sbox[x] ^ sbox[x ^ a]) == c) flag2 = 1;
                        if ((sbox[x] ^ sbox[x ^ b]) == d) flag3 = 1;
                        if (flag1 && flag2 && flag3) EBCT[a][b][c][d]++;
                    }
                }
            }
        }
    }
}

// [HBS21], [YSS+22]
void Sbox_DBCT(vector<vector<int>>& DBCT, const int sbox[siz], const int sbox_inv[siz]) {
    //int dbct[siz][siz][siz][siz] = { 0 };
    //int UBCT[siz][siz][siz] = { 0 };
    //int LBCT[siz][siz][siz] = { 0 };
    vector<vector<vector<vector<int>>>> dbct(siz, vector<vector<vector<int>>>(siz, vector<vector<int>>(siz, vector<int>(siz, 0))));
    vector<vector<vector<int>>> UBCT(siz, vector<vector<int>>(siz, vector<int>(siz, 0)));
    vector<vector<vector<int>>> LBCT(siz, vector<vector<int>>(siz, vector<int>(siz, 0)));
    Sbox_UBCT(UBCT, sbox, sbox_inv);
    Sbox_LBCT(LBCT, sbox, sbox_inv);

    for (int a = 0; a < siz; a++) {
        for (int d = 0; d < siz; d++) {
            for (int b = 0; b < siz; b++) {
                for (int c = 0; c < siz; c++) {
                    dbct[a][b][c][d] = UBCT[a][b][c] * LBCT[b][c][d];
                    DBCT[a][d] += dbct[a][b][c][d];
                }
            }
        }
    }

}

// [ZWT24]
void Sbox_GUBCT(vector<vector<vector<vector<vector<vector<int>>>>>>& GUBCT, const int sbox[siz], const int sbox_inv[siz]) {

    for (int a0 = 0; a0 < siz; a0++) {
        for (int a1_1 = 0; a1_1 < siz; a1_1++) {
            for (int b1_1 = 0; b1_1 < siz; b1_1++) {
                for (int b1_2 = 0; b1_2 < siz; b1_2++) {
                    for (int x = 0; x < siz; x++) {
                        int flag1 = ((sbox[x] ^ sbox[x ^ a0]) == a1_1) ? 1 : 0;
                        int flag2 = ((sbox[x] ^ b1_1 ^ sbox[x ^ a0] ^ b1_2) == (a1_1 ^ b1_1 ^ b1_2)) ? 1 : 0;
                        int flag3 = ((sbox_inv[sbox[x] ^ b1_1] ^ sbox_inv[sbox[x ^ a0] ^ b1_2]) == a0) ? 1 : 0;

                        if (flag1 && flag2 && flag3) GUBCT[a0][a0][a1_1][a1_1 ^ b1_1 ^ b1_2][b1_1][b1_2]++;

                    }
                }
            }
        }
    }
}

// [ZWT24]
void Sbox_GLBCT(vector<vector<vector<vector<vector<vector<int>>>>>>& GLBCT, const int sbox[siz], const int sbox_inv[siz]) {

    for (int a0_1 = 0; a0_1 < siz; a0_1++) {
        //for (int a0_2 = 0; a0_2 < siz; a0_2++) {
        for (int b0_1 = 0; b0_1 < siz; b0_1++) {
            for (int b0_2 = 0; b0_2 < siz; b0_2++) {
                //for (int b1_1 = 0; b1_1 < siz; b1_1++) {
                for (int b1 = 0; b1 < siz; b1++) {
                    for (int x = 0; x < siz; x++) {
                        int flag1 = ((sbox[x] ^ sbox[x ^ b0_1]) == b1) ? 1 : 0;
                        int flag2 = ((sbox[x ^ a0_1] ^ sbox[x ^ a0_1 ^ b0_2]) == b1) ? 1 : 0;
                        int flag3 = ((sbox_inv[sbox[x] ^ b1] ^ sbox_inv[sbox[x ^ a0_1] ^ b1]) == (a0_1 ^ b0_1 ^ b0_2)) ? 1 : 0;

                        if (flag1 && flag2 && flag3) GLBCT[a0_1][a0_1 ^ b0_1 ^ b0_2][b0_1][b0_2][b1][b1]++;

                    }
                }
                //}
            }
        }
        //}
    }
}

// [ZWT24]
void Sbox_GDBCT(vector<vector<int>>& GDBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<vector<vector<vector<vector<int>>>>>> GUBCT(siz, vector<vector<vector<vector<vector<int>>>>>(siz, vector<vector<vector<vector<int>>>>(siz, vector<vector<vector<int>>>(siz, vector<vector<int>>(siz, vector<int>(siz, 0))))));
    vector<vector<vector<vector<vector<vector<int>>>>>> GLBCT(siz, vector<vector<vector<vector<vector<int>>>>>(siz, vector<vector<vector<vector<int>>>>(siz, vector<vector<vector<int>>>(siz, vector<vector<int>>(siz, vector<int>(siz, 0))))));
    Sbox_GUBCT(GUBCT, sbox, sbox_inv);
    Sbox_GLBCT(GLBCT, sbox, sbox_inv);

    for (int a0 = 0; a0 < siz; a0++) {
        for (int b2 = 0; b2 < siz; b2++) {
            for (int a1_1 = 0; a1_1 < siz; a1_1++) {
                //for (int a1_2 = 0; a1_2 < siz; a1_2++) {
                for (int b1_1 = 0; b1_1 < siz; b1_1++) {
                    for (int b1_2 = 0; b1_2 < siz; b1_2++) {
                        GDBCT[a0][b2] += GUBCT[a0][a0][a1_1][a1_1 ^ b1_1 ^ b1_2][b1_1][b1_2] * GLBCT[a1_1][a1_1 ^ b1_1 ^ b1_2][b1_1][b1_2][b2][b2];
                    }
                }
                //}
            }
        }
    }

}

// Optimized Implementation of DBCT* ([WSW+24]) 
// GDBCT When a0=a0', b2=b2' in [ZWT24]
void Optimized_GUBCT(vector<vector<vector<vector<int>>>>& GUBCT, const int sbox[siz], const int sbox_inv[siz]) {
    for (int a0 = 0; a0 < siz; a0++) {
        for (int a1_1 = 0; a1_1 < siz; a1_1++) {
            for (int b1_1 = 0; b1_1 < siz; b1_1++) {
                for (int b1_2 = 0; b1_2 < siz; b1_2++) {
                    for (int x = 0; x < siz; x++) {
                        int flag1 = ((sbox[x] ^ sbox[x ^ a0]) == a1_1) ? 1 : 0;
                        int flag2 = ((sbox[x] ^ b1_1 ^ sbox[x ^ a0] ^ b1_2) == (a1_1 ^ b1_1 ^ b1_2)) ? 1 : 0;
                        int flag3 = ((sbox_inv[sbox[x] ^ b1_1] ^ sbox_inv[sbox[x ^ a0] ^ b1_2]) == a0) ? 1 : 0;

                        if (flag1 && flag2 && flag3) GUBCT[a0][a1_1][b1_1][b1_2]++;
                    }
                }
            }
        }
    }
}

void Optimized_GLBCT(vector<vector<vector<vector<int>>>>& GLBCT, const int sbox[siz], const int sbox_inv[siz]) {

    for (int a1_1 = 0; a1_1 < siz; a1_1++) {
        for (int b1_1 = 0; b1_1 < siz; b1_1++) {
            for (int b1_2 = 0; b1_2 < siz; b1_2++) {
                for (int b2 = 0; b2 < siz; b2++) {
                    for (int x = 0; x < siz; x++) {
                        int flag1 = ((sbox[x] ^ sbox[x ^ b1_1]) == b2) ? 1 : 0;
                        int flag2 = ((sbox[x ^ a1_1] ^ sbox[x ^ a1_1 ^ b1_2]) == b2) ? 1 : 0;
                        int flag3 = ((sbox_inv[sbox[x] ^ b2] ^ sbox_inv[sbox[x ^ a1_1] ^ b2]) == (a1_1 ^ b1_1 ^ b1_2)) ? 1 : 0;

                        if (flag1 && flag2 && flag3) GLBCT[a1_1][b1_1][b1_2][b2]++;

                    }
                }
            }
        }
    }
}

void Optimized_DBCT_A(vector<vector<int>>& DBCT_A, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<vector<vector<int>>>> GUBCT(siz, vector<vector<vector<int>>>(siz, vector<vector<int>>(siz, vector<int>(siz, 0))));
    vector<vector<vector<vector<int>>>> GLBCT(siz, vector<vector<vector<int>>>(siz, vector<vector<int>>(siz, vector<int>(siz, 0))));
    Optimized_GUBCT(GUBCT, sbox, sbox_inv);
    Optimized_GLBCT(GLBCT, sbox, sbox_inv);

    for (int a0 = 0; a0 < siz; a0++) {
        for (int b2 = 0; b2 < siz; b2++) {
            for (int a1_1 = 0; a1_1 < siz; a1_1++) {
                for (int b1_1 = 0; b1_1 < siz; b1_1++) {
                    for (int b1_2 = 0; b1_2 < siz; b1_2++) {
                        DBCT_A[a0][b2] += GUBCT[a0][a1_1][b1_1][b1_2] * GLBCT[a1_1][b1_1][b1_2][b2];
                    }
                }
            }
        }
    }

}

// New Impossible Tables
void Sbox_iUBCT(vector<vector<int>>& iUBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<int>> DDT(siz, vector<int>(siz, 0));
    vector<vector<int>> BCT(siz, vector<int>(siz, 0));
    Sbox_DDT(DDT, sbox);
    Sbox_BCT(BCT, sbox, sbox_inv);

    for (int a0 = 0; a0 < siz; a0++) {
        for (int b2 = 0; b2 < siz; b2++) {
            for (int a1_1 = 0; a1_1 < siz; a1_1++) {
                for (int a1_2 = 0; a1_2 < siz; a1_2++) {
                    if (DDT[a0][a1_1] > 0 && DDT[a0][a1_2] > 0 && BCT[a1_1][b2] > 0 && BCT[a1_2][b2] > 0) iUBCT[a0][b2]++;
                }
            }

        }
    }
}

void Sbox_iLBCT(vector<vector<int>>& iLBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<int>> DDT(siz, vector<int>(siz, 0));
    vector<vector<int>> BCT(siz, vector<int>(siz, 0));
    Sbox_DDT(DDT, sbox);
    Sbox_BCT(BCT, sbox, sbox_inv);

    for (int a0 = 0; a0 < siz; a0++) {
        for (int b2 = 0; b2 < siz; b2++) {
            for (int b1_1 = 0; b1_1 < siz; b1_1++) {
                for (int b1_2 = 0; b1_2 < siz; b1_2++) {
                    if (DDT[b1_1][b2] > 0 && DDT[b1_2][b2] > 0 && BCT[a0][b1_1] > 0 && BCT[a0][b1_2] > 0) iLBCT[a0][b2]++;
                }
            }

        }
    }
}

void Sbox_iGUBCT(vector<vector<int>>& iGUBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<vector<vector<int>>>> GBCT(siz, vector<vector<vector<int>>>(siz, vector<vector<int>>(siz, vector<int>(siz, 0))));
    vector<vector<int>> DDT(siz, vector<int>(siz, 0));
    vector<vector<int>> BCT(siz, vector<int>(siz, 0));
    Sbox_GBCT(GBCT, sbox, sbox_inv);
    Sbox_DDT(DDT, sbox);
    Sbox_BCT(BCT, sbox, sbox_inv);

    for (int a0 = 0; a0 < siz; a0++) {
        for (int b2 = 0; b2 < siz; b2++) {
            for (int a1_1 = 0; a1_1 < siz; a1_1++) {
                for (int a1_2 = 0; a1_2 < siz; a1_2++) {
                    if (DDT[a0][a1_1] > 0 && DDT[a0][a1_2] > 0 && GBCT[a1_1][b2][b2][a1_2] > 0) iGUBCT[a0][b2]++;
                }
            }

        }
    }
}

void Sbox_iGLBCT(vector<vector<int>>& iGLBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<vector<vector<int>>>> GBCT(siz, vector<vector<vector<int>>>(siz, vector<vector<int>>(siz, vector<int>(siz, 0))));
    vector<vector<int>> DDT(siz, vector<int>(siz, 0));
    vector<vector<int>> BCT(siz, vector<int>(siz, 0));
    Sbox_GBCT(GBCT, sbox, sbox_inv);
    Sbox_DDT(DDT, sbox);
    Sbox_BCT(BCT, sbox, sbox_inv);

    for (int a0 = 0; a0 < siz; a0++) {
        for (int b2 = 0; b2 < siz; b2++) {
            for (int b1_1 = 0; b1_1 < siz; b1_1++) {
                for (int b1_2 = 0; b1_2 < siz; b1_2++) {
                    if (DDT[b1_1][b2] > 0 && DDT[b1_2][b2] > 0 && GBCT[a0][b1_1][b1_2][a0] > 0) iGLBCT[a0][b2]++;
                    //if (DDT[b1_1][b2] > 0 && DDT[b1_2][b2] > 0 && BCT[a1_1][b2] > 0 && BCT[a1_2][b2] > 0) iGUBCT[a0][b2]++;
                }
            }

        }
    }
}

void Sbox_iGDBCT(vector<vector<int>>& iGDBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<int>> iGUBCT(siz, vector<int>(siz, 0));
    vector<vector<int>> iGLBCT(siz, vector<int>(siz, 0));
    Sbox_iGUBCT(iGUBCT, sbox, sbox_inv);
    Sbox_iGLBCT(iGLBCT, sbox, sbox_inv);

    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            iGDBCT[a][b] += iGUBCT[a][b] + iGLBCT[a][b];
        }
    }

}

// Optimized Implementation of iU/LBCT in ToSC Submission
void Optimized_UGBCT(vector<vector<vector<int>>>& UGBCT, const int sbox[siz], const int sbox_inv[siz]) {
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            for (int x = 0; x < siz; x++) {
                UGBCT[a][b][(sbox_inv[sbox[x] ^ b] ^ sbox_inv[sbox[x ^ a] ^ b])]++;
            }
        }
    }
}

void Optimized_LGBCT(vector<vector<vector<int>>>& GLBCT, const int sbox[siz], const int sbox_inv[siz]) {
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            for (int x = 0; x < siz; x++) {
                GLBCT[a][b][(sbox[x ^ a] ^ sbox[sbox_inv[sbox[x] ^ b] ^ a])]++;
            }
        }
    }
}

void Optimized_iUBCT(vector<vector<int>>& iUBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<vector<int>>> UGBCT(siz, vector<vector<int>>(siz, vector<int>(siz, 0)));
    vector<vector<int>> DDT(siz, vector<int>(siz, 0));
    Optimized_UGBCT(UGBCT, sbox, sbox_inv);
    Sbox_DDT(DDT, sbox);

    for (int a0 = 0; a0 < siz; a0++) {
        for (int b2 = 0; b2 < siz; b2++) {
            for (int a1_1 = 0; a1_1 < siz; a1_1++) {
                for (int a1_2 = 0; a1_2 < siz; a1_2++) {
                    if (DDT[a0][a1_1] > 0 && DDT[a0][a1_2] > 0 && UGBCT[a1_1][b2][a1_2] > 0) iUBCT[a0][b2]++;
                }
            }

        }
    }
}

void Optimized_iLBCT(vector<vector<int>>& iLBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<vector<int>>> LGBCT(siz, vector<vector<int>>(siz, vector<int>(siz, 0)));
    vector<vector<int>> DDT(siz, vector<int>(siz, 0));
    vector<vector<int>> BCT(siz, vector<int>(siz, 0));
    Optimized_LGBCT(LGBCT, sbox, sbox_inv);
    Sbox_DDT(DDT, sbox);

    for (int a0 = 0; a0 < siz; a0++) {
        for (int b2 = 0; b2 < siz; b2++) {
            for (int b1_1 = 0; b1_1 < siz; b1_1++) {
                for (int b1_2 = 0; b1_2 < siz; b1_2++) {
                    if (DDT[b1_1][b2] > 0 && DDT[b1_2][b2] > 0 && LGBCT[a0][b1_1][b1_2] > 0) iLBCT[a0][b2]++;
                }
            }

        }
    }
}

void Optimized_iDBCT(vector<vector<int>>& iGDBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<int>> iUBCT(siz, vector<int>(siz, 0));
    vector<vector<int>> iLBCT(siz, vector<int>(siz, 0));
    Optimized_iUBCT(iUBCT, sbox, sbox_inv);
    Optimized_iLBCT(iLBCT, sbox, sbox_inv);

    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            iGDBCT[a][b] += iUBCT[a][b] + iLBCT[a][b];
        }
    }

}

void Fast_iUBCT(vector<vector<int>>& iUBCT, const int sbox[siz], const int sbox_inv[siz]) {
    vector<vector<vector<int>>> UGBCT(siz, vector<vector<int>>(siz, vector<int>(siz, 0)));
    vector<vector<int>> DDT(siz, vector<int>(siz, 0));
    Optimized_UGBCT(UGBCT, sbox, sbox_inv);
    Sbox_DDT(DDT, sbox);

    int cnt = 0;
    for (int a0 = 0; a0 < siz; a0++) {
        for (int b2 = 0; b2 < siz; b2++) {
            bool flag = false;
            for (int a1_1 = 0; a1_1 < siz; a1_1++) {
                for (int a1_2 = 0; a1_2 < siz; a1_2++) {
                    if (DDT[a0][a1_1] > 0 && DDT[a0][a1_2] > 0 && UGBCT[a1_1][b2][a1_2] > 0) {
                        cnt++;
                        iUBCT[a0][b2] = 1;
                        flag = true;
                        break;
                    }
                }
                if (flag) break;
            }
            if (!flag) cout << hex << a0 << ' ' << b2 << endl;
        }
    }
    cout << cnt << endl;

}

int main() {
    // Fast Computation of iUBCT
    vector<vector<int>> iUBCT_Fast(siz, vector<int>(siz, 0));
    Fast_iUBCT(iUBCT_Fast, skinny8, skinny8_inv);

    // Computation of iU/LBCT
    vector<vector<int>> iUBCT(siz, vector<int>(siz, 0));
    Optimized_iUBCT(iUBCT, skinny8, skinny8_inv);
    cout << "Zero in iUBCT:" << endl;
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            if (iUBCT[a][b] == 0) cout << hex << a << ' ' << b << endl;
        }
    }

    // iLBCT
    vector<vector<int>> iLBCT(siz, vector<int>(siz, 0));
    Optimized_iLBCT(iLBCT, skinny8, skinny8_inv);
    cout << "Zero in iLBCT:" << endl;
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            if (iLBCT[a][b] == 0) cout <<hex << a << ' ' << b << endl;
        }
    }

    // Computation of DBCT*
    vector<vector<int>> DBCT_A(siz, vector<int>(siz, 0));
    Optimized_DBCT_A(DBCT_A, skinny8, skinny8_inv);
    cout << "Zero in DBCT*:" << endl;
    for (int a = 0; a < siz; a++) {
        for (int b = 0; b < siz; b++) {
            if (DBCT_A[a][b] == 0) cout << hex << a << ' ' << b << endl;
        }
    }

}
