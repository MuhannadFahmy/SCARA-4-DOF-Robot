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
#include "stdafx.h"
#include "interpolation.h"
#include <stdio.h>
#include <math.h>
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
	//
	//} while (sel);
	////
	

	//double x1 = 0.0;
	//double x2 = 12;
	//double x3 = 7;
	//double x4 = 5;

	//vector<double> vtheta1, vtime;
	//vtheta1 = { 0, 100, -100, 120 };
	//vtime = { 0,5,10,15 };
	//
	////tk::spline::cspline;
	//tk::spline S1;
	//
	//S1.set_boundary(tk::spline::first_deriv, 0.0,tk::spline::first_deriv, 0.0);
	//S1.set_boundary(tk::spline::second_deriv, 0.0,tk::spline::second_deriv, 0.0);
	//
	//S1.set_points(vtime, vtheta1);

	//cout << "x1  " << S1(x1) << " & x2  " << S1(x2) << " & x3  " << S1(x3) << " & x4 " << S1(x4) << endl;

	// to get the first derivative -- double interpol = s.deriv(1,x);


	//alglib::real_1d_array X = "[,5,10,15]";
	//alglib::real_1d_array Y = "[0, 100, -100, 120]";
	//alglib::real_2d_array tbl;

	//double t = 5;
	//double v;
	//double v1;
	//double v2;
	//double v3;
	//double d,ds,ds2;
	//alglib::spline1dinterpolant s;
	//alglib::ae_int_t natural_bound_type = 2;


	//alglib::spline1dbuildcubic(X, Y, s);
	//v = alglib::spline1dcalc(s, x1);
	//v1 = alglib::spline1dcalc(s, x2);
	//v2 = alglib::spline1dcalc(s, x3);
	//v3 = alglib::spline1dcalc(s, x4);
	//alglib::spline1ddiff(s, t,d,ds,ds2) ;
	//alglib::ae_int_t n;
	//alglib::spline1dunpack(s,n, tbl);

	//cout << "x1  " << v << " & x2  " << v1 << " & x3  " << v2 << " & x4 " << v3 << endl;

	//cout << "t  " << d << " & t'  " << ds << " & t''  " << ds2 << endl;

	//cout << tbl.tostring(2) << endl;

	////cout << "     " << tbl << "    " << endl;


	char ch;
	int c;
	bool sel = true;
	printf("Press any key to continue or press ESC to exit \n");
	c = _getch();
	do {

		if (c != 27)//27 is the value for ESC key press
		{
		pathPlanning();

		}
		else
		break;

	} while (sel);
}
