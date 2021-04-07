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
#include <vector>
#include "spline.h"

using namespace std;
using namespace Eigen;



int main(int argc, char* argv[])
{
	//char ch;
	//int c;
	//bool sel = true;
	//printf("Press any key to continue or press ESC to exit \n");
	//c = _getch();
	//do {
	//	if (c != 27)//27 is the value for ESC key press
	//	{
	//		printf("Press '1' to select the forward calculation or \n");
	//		printf("press '2' to select the inverse calculation \n");
	//		ch = _getch();
	//		switch (ch) {
	//		case '1':
	//			cout << "\n";
	//			cout << "Forward Kinematics\n";
	//			jointVectorInput();
	//			sel = true;
	//			break;
	//		case '2':
	//			cout << "\n";
	//			cout << "Inverse Kinematics\n";
	//			moveToCoordinates();
	//			sel = true;
	//			break;
	//		default:
	//			cout << "Press ESC to exit\n";
	//			sel = false;
	//		}
	//	}
	//	else
	//		break;

	//} while (sel);
	//
	//

	double x1 = 0.0;
	double x2 = 3.0;
	double x3 = 7;
	double x4 = 15.0;

	vector<double> vtheta1, vtime;
	vtheta1 = { 0, 100, -100, 120 };
	vtime = { 0,5,10,15 };

	//tk::spline::cspline;
	tk::spline S1;
	
	S1.set_boundary(tk::spline::first_deriv, 0.0,tk::spline::first_deriv, 0.0);
	S1.set_boundary(tk::spline::second_deriv, 0.0,tk::spline::second_deriv, 0.0);
	
	S1.set_points(vtime, vtheta1);

	// to get the first derivative -- double interpol = s.deriv(1,x);

	cout << "x1  " << S1(x1) << " & x2  " << S1(x2) << " & x3  " << S1(x3) << " & x4 " << S1(x4) << endl;


	return 0;
}
