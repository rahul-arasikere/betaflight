#pragma once

#define little_endian(x) ((unsigned char*)&x) // little endian

#define on_little_endian(x, f) ({\
		const typeof(x) _x = x;\
	  	const unsigned char *bytes = little_endian(_x);\
	    for (uint16_t i = 0; i < sizeof(x); i++) {\
	    	f(bytes[i]);\
		}\
	})

