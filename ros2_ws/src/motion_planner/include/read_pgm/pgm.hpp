/*
This header file has been created by
Name : Aravind E Vijayan
       B110487EC
       National Institute of Technology Calicut
*/
#ifndef PGM_HPP
#define PGM_HPP

struct table	//image structure
	{
	int **data;
	int cols;
	int rows;
	};
/*
function to read pgm image
Usage :
	parameter must be the path of image passed from terminal
	the function returns the image as a 2-D matrix along with size informations
*/
table pgm_imread(char *); 

#endif
