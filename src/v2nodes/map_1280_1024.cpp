/* 
 * File:   map.cpp
 * Author: andrea
 *
 * Code originally developed by Eng. Enrico Di Lello
 * 
 * Created on February 8, 2013, 3:14 PM
 */

#include "map_1280_1024.h"

PixelMap pixelMap; // map between pixel coordinates and XY coordinates 




//==============================================================================


int InitPixelMap(char *map_file) {
	
	FILE *map;

	map = fopen(map_file,"r");
	char line[MAX_LINE_LEN];
	int i;
	float indiceRiga;
	float indiceColonna;
	float x_coord;
	float y_coord;
	
	if (map == NULL)
	{
		printf("InitPixelMap: failed to open file\n");
		printf("%s\n",map_file);
	   // getchar();
		return -1;
	}
	printf("%s\n",map_file); //GIO
	//Read data from file
	for(i=0; i<pixelMap.height*pixelMap.width; i++){
		
		//Read one line at a time
		fgets(line,MAX_LINE_LEN,map);
		
		if (line == NULL)
		{
			printf("InitPixelMap: failed to read from file\n");
			fclose(map);
			//getchar();
			return -1;
		}
		
		else {
			sscanf(line,"%f\t%f\t%f\t%f",&indiceRiga,&indiceColonna,&x_coord,&y_coord);
			pixelMap.x_coord[i] = x_coord;
			pixelMap.y_coord[i] = y_coord;
                   //     printf("%2.2f %2.2f \n",pixelMap.x_coord[i],pixelMap.y_coord[i]);
		}
	}
	
	fclose(map);
	
	return 0;
}


CvPoint2D32f  getGlobalCoord(int x, int y){

    CvPoint2D32f  coord;
    coord.x = pixelMap.x_coord[y*pixelMap.width+x];
    coord.y = pixelMap.y_coord[y*pixelMap.width+x];
    
    
             //   printf("%2.2f %2.2f \n",coord.x, coord.y);
    
    
    return coord;
    
}


#ifdef DEBUG_C

int main(int argc, char * argv[]) {
    
    int i,j;
    
    char *name="Map_c2.txt";
    CvPoint2D32f coord;
    
    printf("%d\t%d\n",pixelMap.width, pixelMap.height);
    
//    return 0;
    
    // Load Map
    InitPixelMap(name);
    
        int n1,n2; 
        
  // for (i=0; i<pixelMap.width; i++)
  //      for (j=0; j<pixelMap.height; j++) {
    while(1){
   
        scanf ("%d",&n1);
         scanf ("%d",&n2);
    coord = getGlobalCoord(n1,n2);
              printf("%2.2f %2.2f \n",coord.x, coord.y);
              
       
    }
   //     }
    
//    for (i=0; i<pixelMap.width*pixelMap.height; i++)
//        printf("%2.2f %2.2f \n",pixelMap.x_coord[i],pixelMap.y_coord[i]);
    
              
    return 0;

}


#endif
