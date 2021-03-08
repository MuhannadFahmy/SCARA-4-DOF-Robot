// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <math.h>

using namespace std;
using namespace Eigen;

const double pi = 2 * acos(0.0);
float toDegree(float x)
{
	return (180 * x) / pi;
}

float toRad(float x)
{
	return (pi * x) / 180;
}
const float H1 = 405.0;
const float H2 = 70.0;
const float jointH3Lim[] = { -200, -100 };
const float H4 = 220.0;
const float H5 = 55.0;
const float L2 = 195.0;
const float L3 = 142.0;
const float joint1Lim[] = { toRad(-150) , toRad(150) };
const float joint2Lim[] = { toRad(-100) , toRad(100) };
const float joint3Lim[] = { 35, 135 };
const float joint4Lim[] = { toRad(-160), toRad(160) };






int main()
{
	char ch;
	int c;
	bool sel = true;
	printf("Press any key to continue or press ESC to exit \n");
	c = _getch();
	do {
		if (c != 27)//27 is the value for ESC key press
		{
			printf("Press '1' to select the forward calculation or \n");
			printf("press '2' to select the inverse calculation \n");
			ch = _getch();
			switch (ch) {
			case '1':
				cout << "\n"; 
				cout << "Forward Kinematics\n";
				jointVectorInput();
				sel = true;
				break;
			case '2':
				cout << "\n"; 
				cout << "Inverse Kinematics\n";
				//Muhannad's function
				sel = true;
				break;
			default:
				cout << "Press ESC to exit\n";
				sel = false;
			}
		}
		else
				break;

	} while (sel);

	return 0;
}
