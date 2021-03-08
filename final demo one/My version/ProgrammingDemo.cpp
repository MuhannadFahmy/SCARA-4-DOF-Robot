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
#include "Kinematics.h"

using namespace std;
using namespace Eigen;



int main(int argc, char* argv[])
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
				moveToCoordinates();
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
