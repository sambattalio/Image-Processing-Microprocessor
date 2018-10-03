//
//  main.h
//  haar_test
//
//  Created by Sam Battalio on 1/3/17.
//  Copyright © 2017 Sam Battalio. All rights reserved.
//

#ifndef main_h
#define main_h

double minArea = 300;

double filterContoursMinPerimeter = 0;
double filterContoursMinWidth = 0;
double filterContoursMaxWidth = 1000;
double filterContoursMinHeight = 0;
double filterContoursMaxHeight = 1000;
double filterContoursSolidity[] = {0, 100};
double filterContoursMaxVertices = 1000000;
double filterContoursMinVertices = 0;
double filterContoursMinRatio = 0;
double filterContoursMaxRatio = 10000;


double sizeConstant = sqrt(640*640 + 400*400);
double viewAngle = 125;
double dpp = viewAngle/sizeConstant;

double hfov = 53.83310923;//60;//53.83310923;//63.1; //approx
double h_deg = hfov / 640;
double vfov = 49.5;//52.963475;//49.5;//36.08884799;//49.5; //approx
double Y_HEIGHT_GOAL = 8.0 / 12.0; //top of top tape to top of bottom tape

void hslThreshold(Mat &input, double hue[],double sat[], double lum[], Mat &out);

void findContours(Mat &input, vector<vector<Point> > &contours);

void filterContours(Mat &input, vector<vector<Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, vector<vector<Point> > &output) ;

void sortContours(vector<vector<Point> > &contours, int &contIndexO,int &contIndexT, int camI);
void sortWithBiggest(vector<vector<Point> > &contours, int &contIndexO, int &contIndexT, int camI);


#endif /* main_h */
