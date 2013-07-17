/* 
 * File:   map.h
 * Author: andrea
 *
 * Code originally developed by Eng. Enrico Di Lello
 * 
 * Created on February 8, 2013, 3:14 PM
 */

#ifndef _MAP_H_
#define _MAP_H_

#include <cstdlib>
#include <cstdio>

using namespace std;

#define FRAME_HEIGHT     1024
#define FRAME_WIDTH      1280

#define MAX_LINE_LEN  4096


#ifndef __OPENCV_CORE_TYPES_H__

typedef struct CvPoint2D32f {
    float x;
    float y;
}
CvPoint2D32f;

#endif

typedef struct {
    static const int height = FRAME_HEIGHT;
    static const int width = FRAME_WIDTH;
    float x_coord[FRAME_HEIGHT * FRAME_WIDTH];
    float y_coord[FRAME_HEIGHT * FRAME_WIDTH];

} PixelMap;

extern PixelMap pixelMap; // map between pixel coordinates and XY coordinates 

// Init
int InitPixelMap(char *map_file);

CvPoint2D32f getGlobalCoord(int x, int y);


//int PrintPixelMap(PixelMap pixelMap);	


#endif
