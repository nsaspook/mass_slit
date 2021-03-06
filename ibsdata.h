/* 
 * File:   ibsdata.h
 * Author: gigi
 * sim data from IHC source controller to IHC display PC program
 * Created on December 25, 2013, 10:02 PM
 */

#ifndef IBSDATA_H
#define	IBSDATA_H

#ifdef	__cplusplus
extern "C" {
#endif


    const rom unsigned char ibs_stream[] = {
        0x8f, 0xbf, 0xbf, 0x69, 0x89, 0xb7, 0xbd, 0x65, 0x86, 0x88, 0x95, 0x6f, 0x80, 0x80, 0x88, 0x66,
        0x89, 0xb3, 0x90, 0x4f, 0xd0, 0x82, 0x96, 0x72, 0x8f, 0xbf, 0xbf, 0x49, 0x80, 0xa9, 0xb5, 0x78,
        0x81, 0xbd, 0x9e, 0x4a, 0x90, 0x97, 0x9c, 0x79, 0x80, 0x80, 0x8b, 0x42, 0x81, 0x9d, 0xb0, 0x61,
        0x80, 0x80, 0x80, 0x7a, 0xf0, 0x82, 0x84, 0x75, 0x8f, 0xbf, 0xbf, 0x69, 0x89, 0xb7, 0xa5, 0x65,
        0x86, 0x88, 0x9e, 0x6f, 0x80, 0x80, 0x88, 0x66, 0x89, 0xb3, 0xa2, 0x4f, 0xe0, 0x8f, 0xa8, 0x72,
        0x8f, 0xbf, 0xbf, 0x46, 0x80, 0x80, 0x80, 0x78, 0x81, 0xbd, 0x92, 0x4a, 0xa1, 0x86, 0x94, 0x79,
        0x80, 0x80, 0x86, 0x42, 0x90, 0xa8, 0xa8, 0x61, 0x80, 0x80, 0x80, 0x7a, 0xf0, 0x82, 0x84, 0x75,

        0x80, 0x80, 0x96, 0x69, 0x80, 0x80, 0x81, 0x65, 0x80, 0x80, 0x80, 0x6f, 0x80, 0x80, 0x81, 0x66,
        0x80, 0x80, 0x8d, 0x4f, 0xf0, 0x80, 0x80, 0x72, 0x8f, 0xbf, 0xbf, 0x47, 0x80, 0xa7, 0x88, 0x78,
        0x81, 0xbd, 0x99, 0x4a, 0xb0, 0x9f, 0x90, 0x79, 0x80, 0x80, 0x84, 0x42, 0x81, 0x9d, 0xb0, 0x61,
        0x80, 0x80, 0x80, 0x7a, 0xf0, 0x81, 0xb4, 0x75, 0x80, 0x80, 0x80, 0x69, 0x80, 0x80, 0x80, 0x65,
        0x80, 0x80, 0x80, 0x6f, 0x80, 0x80, 0x81, 0x66, 0x80, 0x80, 0x80, 0x4f, 0x80, 0x80, 0x80, 0x72,
        0x8f, 0xbf, 0xbf, 0x55, 0x80, 0xba, 0xa9, 0x78, 0x81, 0xbd, 0x97, 0x4a, 0x80, 0x87, 0xb4, 0x79,
        0x80, 0x80, 0x80, 0x42, 0x90, 0xa8, 0xa8, 0x61, 0x80, 0x80, 0x80, 0x7a, 0xf0, 0x81, 0xb4, 0x75,

        0x80, 0x80, 0xa9, 0x69, 0x80, 0x80, 0x80, 0x65, 0x89, 0x93, 0x81, 0x6f, 0x80, 0x80, 0x88, 0x66,
        0x8e, 0xa0, 0xa2, 0x4f, 0x91, 0x99, 0x84, 0x72, 0x89, 0x85, 0x82, 0x49, 0x80, 0xa9, 0xb5, 0x78,
        0x80, 0x80, 0x94, 0x4a, 0x90, 0x97, 0x9c, 0x79, 0x80, 0x80, 0x89, 0x42, 0x81, 0x9d, 0xb0, 0x61,
        0x80, 0x80, 0xb2, 0x7a, 0xf0, 0x82, 0x84, 0x75, 0x80, 0x80, 0x80, 0x69, 0x80, 0x80, 0x80, 0x65,
        0x89, 0x91, 0xa1, 0x6f, 0x80, 0x80, 0x88, 0x66, 0x8e, 0xa0, 0xa4, 0x4f, 0xa1, 0x9c, 0x8c, 0x72,
        0x89, 0x83, 0x83, 0x46, 0x80, 0x80, 0x80, 0x78, 0x80, 0x80, 0x8a, 0x4a, 0xa1, 0x86, 0x94, 0x79,
        0x80, 0x80, 0x91, 0x42, 0x90, 0xa8, 0xa8, 0x61, 0x80, 0x80, 0x80, 0x7a, 0xf0, 0x82, 0x84, 0x75,
        //                  d        d        d        m        d        d        d        m
        0x8f, 0x90, 0xab, 0x69, 0x88, 0x93, 0x9a, 0x65, 0x87, 0x84, 0x8d, 0x6f, 0x80, 0x80, 0x88, 0x66, // normal operation 10001111 10010000 10101011 01101001 10001000 10010011 10011010 01100101
        0x8b, 0x9c, 0x9c, 0x4f, 0xc0, 0x82, 0x96, 0x72, 0x88, 0x9b, 0x98, 0x55, 0x80, 0xba, 0xa9, 0x78,
        0x81, 0xb4, 0xbd, 0x4a, 0x80, 0x87, 0xb4, 0x79, 0x81, 0xb6, 0x85, 0x42, 0x90, 0xa8, 0xa8, 0x61,
        0x81, 0xaf, 0x8d, 0x7a, 0xf0, 0x82, 0x84, 0x75, 0x8f, 0x8f, 0xba, 0x69, 0x88, 0x93, 0xab, 0x65,
        0x87, 0x84, 0x8a, 0x6f, 0x80, 0x80, 0x88, 0x66, 0x8b, 0x9b, 0x8c, 0x4f, 0xd0, 0x82, 0x96, 0x72,
        0x88, 0x98, 0x9e, 0x49, 0x80, 0xa9, 0xb5, 0x78, 0x81, 0xb5, 0x81, 0x4a, 0x90, 0x97, 0x9c, 0x79,
        0x81, 0xb5, 0xba, 0x42, 0xa0, 0x91, 0x8c, 0x61, 0x81, 0xaf, 0xa8, 0x7a, 0xf0, 0x82, 0x84, 0x75,

        0x00
    };


#ifdef	__cplusplus
}
#endif

#endif	/* IBSDATA_H */

