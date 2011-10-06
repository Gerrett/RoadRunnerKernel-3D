/*
 * drivers/media/video/omap2/dss/gammatable.h
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * The method to calculate this gamma table for a given gamma values where
 * gamma values are ranging from 0.2 to 2.0 in steps of 0.2
 *
 * for palette = 0 : 255
 * table_values = unsgined char(((palette/255.0) ^ gamma ) * 255 )
 */

#ifndef _OMAP4_GAMMATABLE_H_
#define _OMAP4_GAMMATABLE_H_

#define GAMMA_TBL_SZ 256
#define NO_OF_GAMMA_TABLES 10

u32 GammaTable[] = {	
0x00000000 , 
0x00010101 , 
0x00010101 , 
0x00020202 ,
0x00030303 , 
0x00040404 , 
0x00040404 , 
0x00050505 , 
0x00060606 , 
0x00070707 , 
0x00070707 , 
0x00080808 , 
0x00090909 , 
0x000A090A , 
0x000A0A0A , 
0x000B0B0B ,
0x000C0C0C , 
0x000D0C0D , 
0x000D0D0D , 
0x000E0E0E ,
0x000F0F0F , 
0x00100F10 , 
0x00101010 , 
0x00111111 , 
0x00121212 , 
0x00131213 , 
0x00131313 , 
0x00141414 , 
0x00151515 , 
0x00161516 , 
0x00161616 , 
0x00171717 ,
0x00181818 , 
0x00191819 , 
0x001A191A , 
0x001A1A1A ,
0x001B1B1B , 
0x001C1B1C , 
0x001D1C1D , 
0x001D1D1D , 
0x001E1E1E , 
0x001F1E1F , 
0x00201F20 , 
0x00212021 , 
0x00212121 , 
0x00222122 , 
0x00232223 , 
0x00242324 ,
0x00252425 , 
0x00252525 , 
0x00262526 , 
0x00272627 ,
0x00282728 , 
0x00292829 , 
0x00292929 , 
0x002A292A , 
0x002B2A2B , 
0x002C2B2C , 
0x002D2C2D , 
0x002D2D2D , 
0x002E2D2E , 
0x002F2E2F , 
0x00302F30 , 
0x00313031 ,
0x00323132 , 
0x00333233 , 
0x00333233 , 
0x00343334 ,
0x00353435 , 
0x00363536 , 
0x00373637 , 
0x00383738 , 
0x00393739 , 
0x00393839 , 
0x003A393A , 
0x003B3A3B , 
0x003C3B3C , 
0x003D3C3D , 
0x003E3D3E , 
0x003F3E3F ,
0x00403E40 , 
0x00413F41 , 
0x00424042 , 
0x00424142 ,
0x00434243 , 
0x00444344 , 
0x00454445 , 
0x00464546 , 
0x00474647 , 
0x00484748 , 
0x00494749 , 
0x004A484A , 
0x004B494B , 
0x004C4A4C , 
0x004D4B4D , 
0x004E4C4E ,
0x004F4D4F , 
0x00504E50 , 
0x00514F51 , 
0x00525052 ,
0x00535153 , 
0x00545254 , 
0x00555355 , 
0x00565456 , 
0x00575557 , 
0x00585658 , 
0x00595759 , 
0x005A585A , 
0x005B595B , 
0x005C5A5C , 
0x005D5B5D , 
0x005E5D5E ,
0x00605E60 , 
0x00615F61 , 
0x00626062 , 
0x00636163 ,
0x00646264 , 
0x00656365 , 
0x00666466 , 
0x00676567 , 
0x00696769 , 
0x006A686A , 
0x006B696B , 
0x006C6A6C , 
0x006D6B6D , 
0x006E6C6E , 
0x00706D70 , 
0x00716F71 ,
0x00727072 , 
0x00737173 , 
0x00747274 , 
0x00767376 ,
0x00777577 , 
0x00787678 , 
0x007A777A , 
0x007B787B , 
0x007C7A7C , 
0x007D7B7D , 
0x007F7C7F , 
0x00807D80 , 
0x00817E81 , 
0x00828082 , 
0x00848184 , 
0x00858285 ,
0x00868386 , 
0x00878587 , 
0x00898689 , 
0x008A878A ,
0x008B888B , 
0x008C8A8C , 
0x008E8B8E , 
0x008F8C8F , 
0x00908D90 , 
0x00918F91 , 
0x00939093 , 
0x00949194 , 
0x00959295 , 
0x00969396 , 
0x00989598 , 
0x00999699 ,
0x009A979A , 
0x009B989B , 
0x009D9A9D , 
0x009E9B9E ,
0x009F9C9F , 
0x00A09DA0 , 
0x00A29EA2 , 
0x00A3A0A3 , 
0x00A4A1A4 , 
0x00A5A2A5 , 
0x00A6A3A6 , 
0x00A8A4A8 , 
0x00A9A6A9 , 
0x00AAA7AA , 
0x00ABA8AB , 
0x00ACA9AC ,
0x00AEAAAE , 
0x00AFABAF , 
0x00B0ADB0 , 
0x00B1AEB1 ,
0x00B2AFB2 , 
0x00B4B0B4 , 
0x00B5B1B5 , 
0x00B6B2B6 , 
0x00B7B3B7 , 
0x00B8B5B8 , 
0x00B9B6B9 , 
0x00BBB7BB , 
0x00BCB8BC , 
0x00BDB9BD , 
0x00BEBABE , 
0x00BFBBBF ,
0x00C0BCC0 , 
0x00C1BDC1 , 
0x00C2BFC2 , 
0x00C4C0C4 ,
0x00C5C1C5 , 
0x00C6C2C6 , 
0x00C7C3C7 , 
0x00C8C4C8 , 
0x00C9C5C9 , 
0x00CAC6CA , 
0x00CBC7CB , 
0x00CCC8CC , 
0x00CDC9CD , 
0x00CECACE , 
0x00CFCBCF , 
0x00D0CCD0 ,
0x00D2CDD2 , 
0x00D3CED3 , 
0x00D4D0D4 , 
0x00D5D1D5 ,
0x00D7D2D7 , 
0x00D8D3D8 , 
0x00D9D5D9 , 
0x00DAD6DA , 
0x00DCD7DC , 
0x00DDD8DD , 
0x00DEDADE , 
0x00DFDBDF , 
0x00E1DCE1 , 
0x00E2DDE2 , 
0x00E3DEE3 , 
0x00E4E0E4 ,
0x00E5E1E5 , 
0x00E6E2E6 , 
0x00E7E3E7 , 
0x00E8E4E8 ,
0x00EAE5EA , 
0x00EBE6EB , 
0x00ECE7EC , 
0x00EDE8ED , 
0x00EEE9EE , 
0x00EFEAEF , 
0x00EFEBEF , 
0x00F0ECF0 , 
0x00F1ECF1 , 
0x00F2EDF2 , 
0x00F3EEF3 , 
0x00F4EFF4 ,
0x00F5F0F5 , 
0x00F5F0F5 , 
0x00F6F1F6 , 
0x00F7F2F7 ,
0x00F8F3F8 , 
0x00F8F3F8 , 
0x00F9F4F9 , 
0x00FAF5FA , 
0x00FAF5FA , 
0x00FBF6FB , 
0x00FCF7FC , 
0x00FCF7FC , 
0x00FDF8FD , 
0x00FEF9FE , 
0x00FEF9FE , 
0x00FFFAFF };

u8 gamma_0_2[GAMMA_TBL_SZ] = {
0x0,
0x54, 0x60, 0x68, 0x6f,
0x74, 0x78, 0x7c, 0x7f,
0x82, 0x85, 0x87, 0x8a,
0x8c, 0x8e, 0x90, 0x92,
0x94, 0x96, 0x97, 0x99,
0x9a, 0x9c, 0x9d, 0x9e,
0xa0, 0xa1, 0xa2, 0xa3,
0xa5, 0xa6, 0xa7, 0xa8,
0xa9, 0xaa, 0xab, 0xac,
0xad, 0xae, 0xaf, 0xb0,
0xb0, 0xb1, 0xb2, 0xb3,
0xb4, 0xb5, 0xb5, 0xb6,
0xb7, 0xb8, 0xb8, 0xb9,
0xba, 0xba, 0xbb, 0xbc,
0xbc, 0xbd, 0xbe, 0xbe,
0xbf, 0xc0, 0xc0, 0xc1,
0xc2, 0xc2, 0xc3, 0xc3,
0xc4, 0xc4, 0xc5, 0xc6,
0xc6, 0xc7, 0xc7, 0xc8,
0xc8, 0xc9, 0xc9, 0xca,
0xca, 0xcb, 0xcb, 0xcc,
0xcc, 0xcd, 0xcd, 0xce,
0xce, 0xcf, 0xcf, 0xcf,
0xd0, 0xd0, 0xd1, 0xd1,
0xd2, 0xd2, 0xd3, 0xd3,
0xd3, 0xd4, 0xd4, 0xd5,
0xd5, 0xd5, 0xd6, 0xd6,
0xd7, 0xd7, 0xd7, 0xd8,
0xd8, 0xd9, 0xd9, 0xd9,
0xda, 0xda, 0xda, 0xdb,
0xdb, 0xdc, 0xdc, 0xdc,
0xdd, 0xdd, 0xdd, 0xde,
0xde, 0xde, 0xdf, 0xdf,
0xdf, 0xe0, 0xe0, 0xe0,
0xe1, 0xe1, 0xe1, 0xe2,
0xe2, 0xe2, 0xe3, 0xe3,
0xe3, 0xe4, 0xe4, 0xe4,
0xe5, 0xe5, 0xe5, 0xe5,
0xe6, 0xe6, 0xe6, 0xe7,
0xe7, 0xe7, 0xe8, 0xe8,
0xe8, 0xe8, 0xe9, 0xe9,
0xe9, 0xea, 0xea, 0xea,
0xea, 0xeb, 0xeb, 0xeb,
0xeb, 0xec, 0xec, 0xec,
0xed, 0xed, 0xed, 0xed,
0xee, 0xee, 0xee, 0xee,
0xef, 0xef, 0xef, 0xef,
0xf0, 0xf0, 0xf0, 0xf0,
0xf1, 0xf1, 0xf1, 0xf1,
0xf2, 0xf2, 0xf2, 0xf2,
0xf3, 0xf3, 0xf3, 0xf3,
0xf4, 0xf4, 0xf4, 0xf4,
0xf5, 0xf5, 0xf5, 0xf5,
0xf5, 0xf6, 0xf6, 0xf6,
0xf6, 0xf7, 0xf7, 0xf7,
0xf7, 0xf8, 0xf8, 0xf8,
0xf8, 0xf8, 0xf9, 0xf9,
0xf9, 0xf9, 0xfa, 0xfa,
0xfa, 0xfa, 0xfa, 0xfb,
0xfb, 0xfb, 0xfb, 0xfb,
0xfc, 0xfc, 0xfc, 0xfc,
0xfc, 0xfd, 0xfd, 0xfd,
0xfd, 0xfd, 0xfe, 0xfe,
0xfe, 0xfe, 0xff,
};

u8 gamma_0_4[GAMMA_TBL_SZ] = {
0x0,
0x1b, 0x24, 0x2b, 0x30,
0x34, 0x38, 0x3c, 0x3f,
0x42, 0x45, 0x48, 0x4b,
0x4d, 0x4f, 0x52, 0x54,
0x56, 0x58, 0x5a, 0x5c,
0x5d, 0x5f, 0x61, 0x63,
0x64, 0x66, 0x67, 0x69,
0x6a, 0x6c, 0x6d, 0x6f,
0x70, 0x71, 0x73, 0x74,
0x75, 0x77, 0x78, 0x79,
0x7a, 0x7b, 0x7d, 0x7e,
0x7f, 0x80, 0x81, 0x82,
0x83, 0x84, 0x85, 0x86,
0x88, 0x89, 0x8a, 0x8b,
0x8c, 0x8d, 0x8d, 0x8e,
0x8f, 0x90, 0x91, 0x92,
0x93, 0x94, 0x95, 0x96,
0x97, 0x98, 0x98, 0x99,
0x9a, 0x9b, 0x9c, 0x9d,
0x9d, 0x9e, 0x9f, 0xa0,
0xa1, 0xa1, 0xa2, 0xa3,
0xa4, 0xa5, 0xa5, 0xa6,
0xa7, 0xa8, 0xa8, 0xa9,
0xaa, 0xab, 0xab, 0xac,
0xad, 0xad, 0xae, 0xaf,
0xb0, 0xb0, 0xb1, 0xb2,
0xb2, 0xb3, 0xb4, 0xb4,
0xb5, 0xb6, 0xb6, 0xb7,
0xb8, 0xb8, 0xb9, 0xba,
0xba, 0xbb, 0xbb, 0xbc,
0xbd, 0xbd, 0xbe, 0xbf,
0xbf, 0xc0, 0xc0, 0xc1,
0xc2, 0xc2, 0xc3, 0xc3,
0xc4, 0xc5, 0xc5, 0xc6,
0xc6, 0xc7, 0xc8, 0xc8,
0xc9, 0xc9, 0xca, 0xca,
0xcb, 0xcc, 0xcc, 0xcd,
0xcd, 0xce, 0xce, 0xcf,
0xcf, 0xd0, 0xd0, 0xd1,
0xd2, 0xd2, 0xd3, 0xd3,
0xd4, 0xd4, 0xd5, 0xd5,
0xd6, 0xd6, 0xd7, 0xd7,
0xd8, 0xd8, 0xd9, 0xd9,
0xda, 0xda, 0xdb, 0xdb,
0xdc, 0xdc, 0xdd, 0xdd,
0xde, 0xde, 0xdf, 0xdf,
0xe0, 0xe0, 0xe1, 0xe1,
0xe2, 0xe2, 0xe3, 0xe3,
0xe4, 0xe4, 0xe5, 0xe5,
0xe5, 0xe6, 0xe6, 0xe7,
0xe7, 0xe8, 0xe8, 0xe9,
0xe9, 0xea, 0xea, 0xeb,
0xeb, 0xeb, 0xec, 0xec,
0xed, 0xed, 0xee, 0xee,
0xef, 0xef, 0xef, 0xf0,
0xf0, 0xf1, 0xf1, 0xf2,
0xf2, 0xf2, 0xf3, 0xf3,
0xf4, 0xf4, 0xf5, 0xf5,
0xf5, 0xf6, 0xf6, 0xf7,
0xf7, 0xf8, 0xf8, 0xf8,
0xf9, 0xf9, 0xfa, 0xfa,
0xfa, 0xfb, 0xfb, 0xfc,
0xfc, 0xfc, 0xfd, 0xfd,
0xfe, 0xfe, 0xff,
};

u8 gamma_0_6[GAMMA_TBL_SZ] = {
0x0,
0x9, 0xd, 0x11, 0x15,
0x18, 0x1a, 0x1d, 0x1f,
0x22, 0x24, 0x26, 0x28,
0x2a, 0x2c, 0x2e, 0x30,
0x32, 0x33, 0x35, 0x37,
0x39, 0x3a, 0x3c, 0x3d,
0x3f, 0x40, 0x42, 0x43,
0x45, 0x46, 0x48, 0x49,
0x4a, 0x4c, 0x4d, 0x4e,
0x50, 0x51, 0x52, 0x53,
0x55, 0x56, 0x57, 0x58,
0x5a, 0x5b, 0x5c, 0x5d,
0x5e, 0x5f, 0x61, 0x62,
0x63, 0x64, 0x65, 0x66,
0x67, 0x68, 0x69, 0x6b,
0x6c, 0x6d, 0x6e, 0x6f,
0x70, 0x71, 0x72, 0x73,
0x74, 0x75, 0x76, 0x77,
0x78, 0x79, 0x7a, 0x7b,
0x7c, 0x7d, 0x7e, 0x7f,
0x80, 0x81, 0x82, 0x82,
0x83, 0x84, 0x85, 0x86,
0x87, 0x88, 0x89, 0x8a,
0x8b, 0x8c, 0x8d, 0x8d,
0x8e, 0x8f, 0x90, 0x91,
0x92, 0x93, 0x94, 0x94,
0x95, 0x96, 0x97, 0x98,
0x99, 0x99, 0x9a, 0x9b,
0x9c, 0x9d, 0x9e, 0x9e,
0x9f, 0xa0, 0xa1, 0xa2,
0xa3, 0xa3, 0xa4, 0xa5,
0xa6, 0xa7, 0xa7, 0xa8,
0xa9, 0xaa, 0xaa, 0xab,
0xac, 0xad, 0xae, 0xae,
0xaf, 0xb0, 0xb1, 0xb1,
0xb2, 0xb3, 0xb4, 0xb4,
0xb5, 0xb6, 0xb7, 0xb7,
0xb8, 0xb9, 0xba, 0xba,
0xbb, 0xbc, 0xbd, 0xbd,
0xbe, 0xbf, 0xc0, 0xc0,
0xc1, 0xc2, 0xc2, 0xc3,
0xc4, 0xc5, 0xc5, 0xc6,
0xc7, 0xc7, 0xc8, 0xc9,
0xca, 0xca, 0xcb, 0xcc,
0xcc, 0xcd, 0xce, 0xce,
0xcf, 0xd0, 0xd0, 0xd1,
0xd2, 0xd3, 0xd3, 0xd4,
0xd5, 0xd5, 0xd6, 0xd7,
0xd7, 0xd8, 0xd9, 0xd9,
0xda, 0xdb, 0xdb, 0xdc,
0xdd, 0xdd, 0xde, 0xdf,
0xdf, 0xe0, 0xe1, 0xe1,
0xe2, 0xe2, 0xe3, 0xe4,
0xe4, 0xe5, 0xe6, 0xe6,
0xe7, 0xe8, 0xe8, 0xe9,
0xea, 0xea, 0xeb, 0xeb,
0xec, 0xed, 0xed, 0xee,
0xef, 0xef, 0xf0, 0xf0,
0xf1, 0xf2, 0xf2, 0xf3,
0xf4, 0xf4, 0xf5, 0xf5,
0xf6, 0xf7, 0xf7, 0xf8,
0xf8, 0xf9, 0xfa, 0xfa,
0xfb, 0xfb, 0xfc, 0xfd,
0xfd, 0xfe, 0xff,
};

u8 gamma_0_8[GAMMA_TBL_SZ] = {
0x0,
0x3, 0x5, 0x7, 0x9,
0xa, 0xc, 0xe, 0xf,
0x11, 0x13, 0x14, 0x16,
0x17, 0x19, 0x1a, 0x1b,
0x1d, 0x1e, 0x1f, 0x21,
0x22, 0x23, 0x25, 0x26,
0x27, 0x29, 0x2a, 0x2b,
0x2c, 0x2e, 0x2f, 0x30,
0x31, 0x32, 0x34, 0x35,
0x36, 0x37, 0x38, 0x39,
0x3b, 0x3c, 0x3d, 0x3e,
0x3f, 0x40, 0x41, 0x43,
0x44, 0x45, 0x46, 0x47,
0x48, 0x49, 0x4a, 0x4b,
0x4c, 0x4d, 0x4f, 0x50,
0x51, 0x52, 0x53, 0x54,
0x55, 0x56, 0x57, 0x58,
0x59, 0x5a, 0x5b, 0x5c,
0x5d, 0x5e, 0x5f, 0x60,
0x61, 0x62, 0x63, 0x64,
0x65, 0x66, 0x67, 0x68,
0x69, 0x6a, 0x6b, 0x6c,
0x6d, 0x6e, 0x6f, 0x70,
0x71, 0x72, 0x73, 0x74,
0x75, 0x76, 0x77, 0x78,
0x79, 0x7a, 0x7b, 0x7c,
0x7d, 0x7e, 0x7f, 0x80,
0x81, 0x82, 0x83, 0x84,
0x84, 0x85, 0x86, 0x87,
0x88, 0x89, 0x8a, 0x8b,
0x8c, 0x8d, 0x8e, 0x8f,
0x90, 0x91, 0x91, 0x92,
0x93, 0x94, 0x95, 0x96,
0x97, 0x98, 0x99, 0x9a,
0x9b, 0x9c, 0x9c, 0x9d,
0x9e, 0x9f, 0xa0, 0xa1,
0xa2, 0xa3, 0xa4, 0xa5,
0xa5, 0xa6, 0xa7, 0xa8,
0xa9, 0xaa, 0xab, 0xac,
0xac, 0xad, 0xae, 0xaf,
0xb0, 0xb1, 0xb2, 0xb3,
0xb4, 0xb4, 0xb5, 0xb6,
0xb7, 0xb8, 0xb9, 0xba,
0xba, 0xbb, 0xbc, 0xbd,
0xbe, 0xbf, 0xc0, 0xc0,
0xc1, 0xc2, 0xc3, 0xc4,
0xc5, 0xc6, 0xc6, 0xc7,
0xc8, 0xc9, 0xca, 0xcb,
0xcc, 0xcc, 0xcd, 0xce,
0xcf, 0xd0, 0xd1, 0xd1,
0xd2, 0xd3, 0xd4, 0xd5,
0xd6, 0xd6, 0xd7, 0xd8,
0xd9, 0xda, 0xdb, 0xdb,
0xdc, 0xdd, 0xde, 0xdf,
0xe0, 0xe0, 0xe1, 0xe2,
0xe3, 0xe4, 0xe5, 0xe5,
0xe6, 0xe7, 0xe8, 0xe9,
0xe9, 0xea, 0xeb, 0xec,
0xed, 0xee, 0xee, 0xef,
0xf0, 0xf1, 0xf2, 0xf2,
0xf3, 0xf4, 0xf5, 0xf6,
0xf6, 0xf7, 0xf8, 0xf9,
0xfa, 0xfa, 0xfb, 0xfc,
0xfd, 0xfe, 0xff,
};

u8 gamma_1_0[GAMMA_TBL_SZ] = {
0x0,
0x1, 0x2, 0x3, 0x4,
0x5, 0x6, 0x7, 0x8,
0x9, 0xa, 0xb, 0xc,
0xd, 0xe, 0xf, 0x10,
0x11, 0x12, 0x13, 0x14,
0x15, 0x16, 0x17, 0x18,
0x19, 0x1a, 0x1b, 0x1c,
0x1d, 0x1e, 0x1f, 0x20,
0x21, 0x22, 0x23, 0x24,
0x25, 0x26, 0x27, 0x28,
0x29, 0x2a, 0x2b, 0x2c,
0x2d, 0x2e, 0x2f, 0x30,
0x31, 0x32, 0x33, 0x34,
0x35, 0x36, 0x37, 0x38,
0x39, 0x3a, 0x3b, 0x3c,
0x3d, 0x3e, 0x3f, 0x40,
0x41, 0x42, 0x43, 0x44,
0x45, 0x46, 0x47, 0x48,
0x49, 0x4a, 0x4b, 0x4c,
0x4d, 0x4e, 0x4f, 0x50,
0x51, 0x52, 0x53, 0x54,
0x55, 0x56, 0x57, 0x58,
0x59, 0x5a, 0x5b, 0x5c,
0x5d, 0x5e, 0x5f, 0x60,
0x61, 0x62, 0x63, 0x64,
0x65, 0x66, 0x67, 0x68,
0x69, 0x6a, 0x6b, 0x6c,
0x6d, 0x6e, 0x6f, 0x70,
0x71, 0x72, 0x73, 0x74,
0x75, 0x76, 0x77, 0x78,
0x79, 0x7a, 0x7b, 0x7c,
0x7d, 0x7e, 0x7f, 0x80,
0x81, 0x82, 0x83, 0x84,
0x85, 0x86, 0x87, 0x88,
0x89, 0x8a, 0x8b, 0x8c,
0x8d, 0x8e, 0x8f, 0x90,
0x91, 0x92, 0x93, 0x94,
0x95, 0x96, 0x97, 0x98,
0x99, 0x9a, 0x9b, 0x9c,
0x9d, 0x9e, 0x9f, 0xa0,
0xa1, 0xa2, 0xa3, 0xa4,
0xa5, 0xa6, 0xa7, 0xa8,
0xa9, 0xaa, 0xab, 0xac,
0xad, 0xae, 0xaf, 0xb0,
0xb1, 0xb2, 0xb3, 0xb4,
0xb5, 0xb6, 0xb7, 0xb8,
0xb9, 0xba, 0xbb, 0xbc,
0xbd, 0xbe, 0xbf, 0xc0,
0xc1, 0xc2, 0xc3, 0xc4,
0xc5, 0xc6, 0xc7, 0xc8,
0xc9, 0xca, 0xcb, 0xcc,
0xcd, 0xce, 0xcf, 0xd0,
0xd1, 0xd2, 0xd3, 0xd4,
0xd5, 0xd6, 0xd7, 0xd8,
0xd9, 0xda, 0xdb, 0xdc,
0xdd, 0xde, 0xdf, 0xe0,
0xe1, 0xe2, 0xe3, 0xe4,
0xe5, 0xe6, 0xe7, 0xe8,
0xe9, 0xea, 0xeb, 0xec,
0xed, 0xee, 0xef, 0xf0,
0xf1, 0xf2, 0xf3, 0xf4,
0xf5, 0xf6, 0xf7, 0xf8,
0xf9, 0xfa, 0xfb, 0xfc,
0xfd, 0xfe, 0xff,
};

u8 gamma_1_2[GAMMA_TBL_SZ] = {
0x0,
0x0, 0x0, 0x1, 0x1,
0x2, 0x2, 0x3, 0x4,
0x4, 0x5, 0x5, 0x6,
0x7, 0x7, 0x8, 0x9,
0x9, 0xa, 0xb, 0xc,
0xc, 0xd, 0xe, 0xe,
0xf, 0x10, 0x11, 0x12,
0x12, 0x13, 0x14, 0x15,
0x15, 0x16, 0x17, 0x18,
0x19, 0x19, 0x1a, 0x1b,
0x1c, 0x1d, 0x1e, 0x1e,
0x1f, 0x20, 0x21, 0x22,
0x23, 0x24, 0x24, 0x25,
0x26, 0x27, 0x28, 0x29,
0x2a, 0x2b, 0x2c, 0x2c,
0x2d, 0x2e, 0x2f, 0x30,
0x31, 0x32, 0x33, 0x34,
0x35, 0x36, 0x36, 0x37,
0x38, 0x39, 0x3a, 0x3b,
0x3c, 0x3d, 0x3e, 0x3f,
0x40, 0x41, 0x42, 0x43,
0x44, 0x45, 0x46, 0x47,
0x48, 0x49, 0x4a, 0x4b,
0x4c, 0x4c, 0x4d, 0x4e,
0x4f, 0x50, 0x51, 0x52,
0x53, 0x54, 0x55, 0x56,
0x57, 0x58, 0x59, 0x5a,
0x5b, 0x5c, 0x5d, 0x5f,
0x60, 0x61, 0x62, 0x63,
0x64, 0x65, 0x66, 0x67,
0x68, 0x69, 0x6a, 0x6b,
0x6c, 0x6d, 0x6e, 0x6f,
0x70, 0x71, 0x72, 0x73,
0x74, 0x75, 0x76, 0x77,
0x78, 0x7a, 0x7b, 0x7c,
0x7d, 0x7e, 0x7f, 0x80,
0x81, 0x82, 0x83, 0x84,
0x85, 0x86, 0x87, 0x89,
0x8a, 0x8b, 0x8c, 0x8d,
0x8e, 0x8f, 0x90, 0x91,
0x92, 0x93, 0x95, 0x96,
0x97, 0x98, 0x99, 0x9a,
0x9b, 0x9c, 0x9d, 0x9e,
0xa0, 0xa1, 0xa2, 0xa3,
0xa4, 0xa5, 0xa6, 0xa7,
0xa9, 0xaa, 0xab, 0xac,
0xad, 0xae, 0xaf, 0xb0,
0xb2, 0xb3, 0xb4, 0xb5,
0xb6, 0xb7, 0xb8, 0xb9,
0xbb, 0xbc, 0xbd, 0xbe,
0xbf, 0xc0, 0xc1, 0xc3,
0xc4, 0xc5, 0xc6, 0xc7,
0xc8, 0xca, 0xcb, 0xcc,
0xcd, 0xce, 0xcf, 0xd0,
0xd2, 0xd3, 0xd4, 0xd5,
0xd6, 0xd7, 0xd9, 0xda,
0xdb, 0xdc, 0xdd, 0xde,
0xe0, 0xe1, 0xe2, 0xe3,
0xe4, 0xe6, 0xe7, 0xe8,
0xe9, 0xea, 0xeb, 0xed,
0xee, 0xef, 0xf0, 0xf1,
0xf3, 0xf4, 0xf5, 0xf6,
0xf7, 0xf9, 0xfa, 0xfb,
0xfc, 0xfd, 0xff,
};

u8 gamma_1_4[GAMMA_TBL_SZ] = {
0x0,
0x0, 0x0, 0x0, 0x0,
0x1, 0x1, 0x1, 0x2,
0x2, 0x2, 0x3, 0x3,
0x3, 0x4, 0x4, 0x5,
0x5, 0x6, 0x6, 0x7,
0x7, 0x8, 0x8, 0x9,
0x9, 0xa, 0xa, 0xb,
0xc, 0xc, 0xd, 0xd,
0xe, 0xf, 0xf, 0x10,
0x11, 0x11, 0x12, 0x13,
0x13, 0x14, 0x15, 0x15,
0x16, 0x17, 0x17, 0x18,
0x19, 0x1a, 0x1a, 0x1b,
0x1c, 0x1d, 0x1d, 0x1e,
0x1f, 0x20, 0x20, 0x21,
0x22, 0x23, 0x24, 0x24,
0x25, 0x26, 0x27, 0x28,
0x28, 0x29, 0x2a, 0x2b,
0x2c, 0x2d, 0x2d, 0x2e,
0x2f, 0x30, 0x31, 0x32,
0x33, 0x34, 0x34, 0x35,
0x36, 0x37, 0x38, 0x39,
0x3a, 0x3b, 0x3c, 0x3d,
0x3e, 0x3f, 0x40, 0x40,
0x41, 0x42, 0x43, 0x44,
0x45, 0x46, 0x47, 0x48,
0x49, 0x4a, 0x4b, 0x4c,
0x4d, 0x4e, 0x4f, 0x50,
0x51, 0x52, 0x53, 0x54,
0x55, 0x56, 0x57, 0x58,
0x59, 0x5a, 0x5b, 0x5c,
0x5d, 0x5f, 0x60, 0x61,
0x62, 0x63, 0x64, 0x65,
0x66, 0x67, 0x68, 0x69,
0x6a, 0x6b, 0x6d, 0x6e,
0x6f, 0x70, 0x71, 0x72,
0x73, 0x74, 0x75, 0x77,
0x78, 0x79, 0x7a, 0x7b,
0x7c, 0x7d, 0x7f, 0x80,
0x81, 0x82, 0x83, 0x84,
0x85, 0x87, 0x88, 0x89,
0x8a, 0x8b, 0x8c, 0x8e,
0x8f, 0x90, 0x91, 0x92,
0x94, 0x95, 0x96, 0x97,
0x98, 0x9a, 0x9b, 0x9c,
0x9d, 0x9f, 0xa0, 0xa1,
0xa2, 0xa3, 0xa5, 0xa6,
0xa7, 0xa8, 0xaa, 0xab,
0xac, 0xad, 0xaf, 0xb0,
0xb1, 0xb2, 0xb4, 0xb5,
0xb6, 0xb8, 0xb9, 0xba,
0xbb, 0xbd, 0xbe, 0xbf,
0xc1, 0xc2, 0xc3, 0xc4,
0xc6, 0xc7, 0xc8, 0xca,
0xcb, 0xcc, 0xce, 0xcf,
0xd0, 0xd2, 0xd3, 0xd4,
0xd6, 0xd7, 0xd8, 0xda,
0xdb, 0xdc, 0xde, 0xdf,
0xe0, 0xe2, 0xe3, 0xe4,
0xe6, 0xe7, 0xe8, 0xea,
0xeb, 0xec, 0xee, 0xef,
0xf1, 0xf2, 0xf3, 0xf5,
0xf6, 0xf8, 0xf9, 0xfa,
0xfc, 0xfd, 0xff,
};

u8 gamma_1_6[GAMMA_TBL_SZ] = {
0x0,
0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x1,
0x1, 0x1, 0x1, 0x1,
0x2, 0x2, 0x2, 0x3,
0x3, 0x3, 0x4, 0x4,
0x4, 0x5, 0x5, 0x5,
0x6, 0x6, 0x7, 0x7,
0x7, 0x8, 0x8, 0x9,
0x9, 0xa, 0xa, 0xb,
0xb, 0xc, 0xc, 0xd,
0xd, 0xe, 0xe, 0xf,
0xf, 0x10, 0x11, 0x11,
0x12, 0x12, 0x13, 0x14,
0x14, 0x15, 0x15, 0x16,
0x17, 0x17, 0x18, 0x19,
0x19, 0x1a, 0x1b, 0x1b,
0x1c, 0x1d, 0x1e, 0x1e,
0x1f, 0x20, 0x20, 0x21,
0x22, 0x23, 0x23, 0x24,
0x25, 0x26, 0x27, 0x27,
0x28, 0x29, 0x2a, 0x2b,
0x2b, 0x2c, 0x2d, 0x2e,
0x2f, 0x30, 0x31, 0x31,
0x32, 0x33, 0x34, 0x35,
0x36, 0x37, 0x38, 0x39,
0x39, 0x3a, 0x3b, 0x3c,
0x3d, 0x3e, 0x3f, 0x40,
0x41, 0x42, 0x43, 0x44,
0x45, 0x46, 0x47, 0x48,
0x49, 0x4a, 0x4b, 0x4c,
0x4d, 0x4e, 0x4f, 0x50,
0x51, 0x52, 0x53, 0x54,
0x55, 0x56, 0x57, 0x58,
0x59, 0x5b, 0x5c, 0x5d,
0x5e, 0x5f, 0x60, 0x61,
0x62, 0x63, 0x65, 0x66,
0x67, 0x68, 0x69, 0x6a,
0x6b, 0x6d, 0x6e, 0x6f,
0x70, 0x71, 0x72, 0x74,
0x75, 0x76, 0x77, 0x78,
0x7a, 0x7b, 0x7c, 0x7d,
0x7f, 0x80, 0x81, 0x82,
0x84, 0x85, 0x86, 0x87,
0x89, 0x8a, 0x8b, 0x8c,
0x8e, 0x8f, 0x90, 0x92,
0x93, 0x94, 0x95, 0x97,
0x98, 0x99, 0x9b, 0x9c,
0x9d, 0x9f, 0xa0, 0xa1,
0xa3, 0xa4, 0xa6, 0xa7,
0xa8, 0xaa, 0xab, 0xac,
0xae, 0xaf, 0xb1, 0xb2,
0xb3, 0xb5, 0xb6, 0xb8,
0xb9, 0xba, 0xbc, 0xbd,
0xbf, 0xc0, 0xc2, 0xc3,
0xc4, 0xc6, 0xc7, 0xc9,
0xca, 0xcc, 0xcd, 0xcf,
0xd0, 0xd2, 0xd3, 0xd5,
0xd6, 0xd8, 0xd9, 0xdb,
0xdc, 0xde, 0xdf, 0xe1,
0xe2, 0xe4, 0xe5, 0xe7,
0xe8, 0xea, 0xec, 0xed,
0xef, 0xf0, 0xf2, 0xf3,
0xf5, 0xf7, 0xf8, 0xfa,
0xfb, 0xfd, 0xff,
};

u8 gamma_1_8[GAMMA_TBL_SZ] = {
0x0,
0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x1,
0x1, 0x1, 0x1, 0x1,
0x1, 0x2, 0x2, 0x2,
0x2, 0x3, 0x3, 0x3,
0x3, 0x4, 0x4, 0x4,
0x5, 0x5, 0x5, 0x6,
0x6, 0x6, 0x7, 0x7,
0x7, 0x8, 0x8, 0x9,
0x9, 0x9, 0xa, 0xa,
0xb, 0xb, 0xc, 0xc,
0xd, 0xd, 0xe, 0xe,
0xf, 0xf, 0x10, 0x10,
0x11, 0x11, 0x12, 0x12,
0x13, 0x14, 0x14, 0x15,
0x15, 0x16, 0x16, 0x17,
0x18, 0x18, 0x19, 0x1a,
0x1a, 0x1b, 0x1c, 0x1c,
0x1d, 0x1e, 0x1e, 0x1f,
0x20, 0x21, 0x21, 0x22,
0x23, 0x24, 0x24, 0x25,
0x26, 0x27, 0x27, 0x28,
0x29, 0x2a, 0x2b, 0x2b,
0x2c, 0x2d, 0x2e, 0x2f,
0x30, 0x31, 0x31, 0x32,
0x33, 0x34, 0x35, 0x36,
0x37, 0x38, 0x39, 0x39,
0x3a, 0x3b, 0x3c, 0x3d,
0x3e, 0x3f, 0x40, 0x41,
0x42, 0x43, 0x44, 0x45,
0x46, 0x47, 0x48, 0x49,
0x4a, 0x4b, 0x4c, 0x4d,
0x4f, 0x50, 0x51, 0x52,
0x53, 0x54, 0x55, 0x56,
0x57, 0x58, 0x5a, 0x5b,
0x5c, 0x5d, 0x5e, 0x5f,
0x60, 0x62, 0x63, 0x64,
0x65, 0x66, 0x68, 0x69,
0x6a, 0x6b, 0x6c, 0x6e,
0x6f, 0x70, 0x71, 0x73,
0x74, 0x75, 0x77, 0x78,
0x79, 0x7a, 0x7c, 0x7d,
0x7e, 0x80, 0x81, 0x82,
0x84, 0x85, 0x86, 0x88,
0x89, 0x8a, 0x8c, 0x8d,
0x8f, 0x90, 0x91, 0x93,
0x94, 0x96, 0x97, 0x99,
0x9a, 0x9b, 0x9d, 0x9e,
0xa0, 0xa1, 0xa3, 0xa4,
0xa6, 0xa7, 0xa9, 0xaa,
0xac, 0xad, 0xaf, 0xb0,
0xb2, 0xb3, 0xb5, 0xb6,
0xb8, 0xb9, 0xbb, 0xbd,
0xbe, 0xc0, 0xc1, 0xc3,
0xc5, 0xc6, 0xc8, 0xc9,
0xcb, 0xcd, 0xce, 0xd0,
0xd2, 0xd3, 0xd5, 0xd7,
0xd8, 0xda, 0xdc, 0xdd,
0xdf, 0xe1, 0xe2, 0xe4,
0xe6, 0xe8, 0xe9, 0xeb,
0xed, 0xef, 0xf0, 0xf2,
0xf4, 0xf6, 0xf7, 0xf9,
0xfb, 0xfd, 0xff,
};

u8 gamma_2_0[GAMMA_TBL_SZ] = {
0x0,
0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x1,
0x1, 0x1, 0x1, 0x1,
0x1, 0x1, 0x2, 0x2,
0x2, 0x2, 0x2, 0x3,
0x3, 0x3, 0x3, 0x4,
0x4, 0x4, 0x4, 0x5,
0x5, 0x5, 0x5, 0x6,
0x6, 0x6, 0x7, 0x7,
0x7, 0x8, 0x8, 0x9,
0x9, 0x9, 0xa, 0xa,
0xb, 0xb, 0xb, 0xc,
0xc, 0xd, 0xd, 0xe,
0xe, 0xf, 0xf, 0x10,
0x10, 0x11, 0x11, 0x12,
0x12, 0x13, 0x13, 0x14,
0x14, 0x15, 0x16, 0x16,
0x17, 0x17, 0x18, 0x19,
0x19, 0x1a, 0x1b, 0x1b,
0x1c, 0x1d, 0x1d, 0x1e,
0x1f, 0x1f, 0x20, 0x21,
0x21, 0x22, 0x23, 0x24,
0x24, 0x25, 0x26, 0x27,
0x28, 0x28, 0x29, 0x2a,
0x2b, 0x2c, 0x2c, 0x2d,
0x2e, 0x2f, 0x30, 0x31,
0x32, 0x32, 0x33, 0x34,
0x35, 0x36, 0x37, 0x38,
0x39, 0x3a, 0x3b, 0x3c,
0x3d, 0x3e, 0x3f, 0x40,
0x41, 0x42, 0x43, 0x44,
0x45, 0x46, 0x47, 0x48,
0x49, 0x4a, 0x4b, 0x4c,
0x4d, 0x4f, 0x50, 0x51,
0x52, 0x53, 0x54, 0x55,
0x57, 0x58, 0x59, 0x5a,
0x5b, 0x5d, 0x5e, 0x5f,
0x60, 0x61, 0x63, 0x64,
0x65, 0x66, 0x68, 0x69,
0x6a, 0x6c, 0x6d, 0x6e,
0x70, 0x71, 0x72, 0x74,
0x75, 0x76, 0x78, 0x79,
0x7a, 0x7c, 0x7d, 0x7f,
0x80, 0x81, 0x83, 0x84,
0x86, 0x87, 0x89, 0x8a,
0x8c, 0x8d, 0x8f, 0x90,
0x92, 0x93, 0x95, 0x96,
0x98, 0x99, 0x9b, 0x9c,
0x9e, 0xa0, 0xa1, 0xa3,
0xa4, 0xa6, 0xa8, 0xa9,
0xab, 0xac, 0xae, 0xb0,
0xb1, 0xb3, 0xb5, 0xb6,
0xb8, 0xba, 0xbc, 0xbd,
0xbf, 0xc1, 0xc3, 0xc4,
0xc6, 0xc8, 0xca, 0xcb,
0xcd, 0xcf, 0xd1, 0xd3,
0xd4, 0xd6, 0xd8, 0xda,
0xdc, 0xde, 0xe0, 0xe1,
0xe3, 0xe5, 0xe7, 0xe9,
0xeb, 0xed, 0xef, 0xf1,
0xf3, 0xf5, 0xf7, 0xf9,
0xfb, 0xfd, 0xff,
};

u8 *gammaTablePtr[NO_OF_GAMMA_TABLES] = {gamma_0_2, gamma_0_4, gamma_0_6, gamma_0_8, gamma_1_0,
			 gamma_1_2, gamma_1_4, gamma_1_6, gamma_1_8, gamma_2_0};
#endif