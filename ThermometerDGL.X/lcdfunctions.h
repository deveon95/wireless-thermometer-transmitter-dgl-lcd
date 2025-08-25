/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef LCDFUNCTIONS_H
#define	LCDFUNCTIONS_H

#include <xc.h> // include processor files - each processor file is guarded.  

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code.
    int16_t adcToTemperature(uint16_t);
    void displayDigitSegments(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
    void displayNumber(int16_t, uint8_t, uint8_t, uint8_t);
    void displayTemperature(int16_t, uint8_t, uint8_t);
    void setBacklight(uint8_t enable);
    
    // Segment patterns = least significant bit is segA
    static const uint8_t segmentPatterns[] = {
      /*  '0'    '1'    '2'    '3'    '4'    '5'    '6'    '7' */
        0x3fu, 0x06u, 0x5bu, 0x4fu, 0x66u, 0x6du, 0x7cu, 0x07u,
      /*  '8'    '9'    'A'    'B'    'C'    'D'    'E'    'F'   null   - */
        0x7fu, 0x67u, 0x77u, 0x7cu, 0x39u, 0x5eu, 0x79u, 0x71u, 0x00u, 0x40};
    
    
    static const uint16_t temperatureConstants[] = {64945, 65022, 65099, 65149, 65188, 65219, 65246,  65269, 65289, 65308, 65325, 65341, 65356, 65369,   65382, 65394, 65406, 65417, 65427, 65437, 65447, 65456,   65465, 65474, 65482, 65491, 65498, 65506, 65514, 65521, 65528,   65535, 6, 12, 19, 25, 32, 38, 44, 50, 56, 61,   67, 73, 78, 84, 89, 94, 99, 105, 110, 115,   120, 125, 129, 134, 139, 144, 148, 153, 158,   162, 167, 171, 176, 180, 185, 189, 193, 198,   202, 206, 210, 215, 219, 223, 227, 231, 235,   239, 244, 248, 252, 256, 260, 264, 268, 272,   276, 280, 283, 287, 291, 295, 299, 303, 307,   311, 315, 319, 322, 326, 330, 334, 338, 342,   346, 350, 353, 357, 361, 365, 369, 373, 377,   380, 384, 388, 392, 396, 400, 404, 408, 412,   415, 419, 423, 427, 431, 435, 439, 443, 447,   451, 455, 459, 463, 467, 471, 475, 480, 484,   488, 492, 496, 500, 505, 509, 513, 517, 522,   526, 530, 535, 539, 544, 548, 552, 557, 562,   566, 571, 575, 580, 585, 590, 594, 599, 604,   609, 614, 619, 624, 629, 634, 639, 644, 650,   655, 660, 666, 671, 677, 683, 688, 694, 700,   706, 712, 718, 724, 730, 737, 743, 750, 756,   763, 770, 777, 784, 791, 798, 805, 813, 821,   828, 836, 844, 853, 861, 870, 879, 888, 897,   906, 916, 926, 936, 947, 958, 969, 980, 992,   1004, 1017, 1030, 1043, 1057, 1072, 1087,   1102, 1119, 1136, 1154, 1173, 1193, 1214,   1236, 1260, 1285, 1312, 1341, 1373, 1407,   1444, 1486, 1532, 1584, 1644, 1714, 1798,   1902, 2036, 2224, 2525, 3196, 3867};

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

