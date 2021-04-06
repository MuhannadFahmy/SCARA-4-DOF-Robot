#pragma once
#include "math.h"
#include <iostream>
#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
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
// just trying 
const float L1 = 195.0;
const float L2 = 142.0;
const float H1 = 405.0;
const float H2 = 70.0;
const float H4 = 220.0;
const float H5 = 55.0;
const float joint1Lim[] = { toRad(-150) , toRad(150) };
const float joint2Lim[] = { toRad(-100) , toRad(100) };
const float jointH3Lim[] = { -200, -100 };
const float joint3Lim[] = { -200, -100 };
const float joint4Lim[] = { toRad(-160), toRad(160) };

MatrixXf inverseKinematics(float gPos[]) //almost done
{

	float noSolution = abs((pow(gPos[0], 2) + pow(gPos[1], 2) - (pow(L1, 2) + pow(L2, 2))) / (2.0 * L1 * L2));  // checking for solution existance
	bool oneSolution = false;
	MatrixXf solVector(4, 2);
	solVector.setZero();

	gPos[3] = toRad(gPos[3]);


	// No solution case
	if (noSolution > 1)
	{
		cout << "There is no solution as b = " << noSolution << endl;
		return solVector;
	}

	// Calculating theta 2 values

	float b = ((pow(gPos[0], 2) + pow(gPos[1], 2) - (pow(L1, 2) + pow(L2, 2))) / (2 * L1 * L2));

	float theta2 = atan2(sqrt(1 - pow(b, 2)), b);

	if (theta2 < joint2Lim[0] || theta2 > joint2Lim[1])
	{
		cout << "Joint 2 out of limits as \u03B82 equals  " << toDegree(theta2) << endl;

		return solVector;
	}

	// if theta 2 in the limits then theta 2* will be in the limits

	int cond1, cond2;
	cond1 = sqrt(pow(gPos[0], 2) + pow(gPos[1], 2));
	cond2 = L1 + L2;


	if (cond1 == cond2)
	{
		oneSolution == true;
		cout << "There is only one solution" << endl;
	}

	// Calculating theta 1 values

	float c2 = cos(theta2);
	float s2 = sin(theta2);

	float theta1 = atan2(((L1 + L2 * c2) * gPos[1]) - ((L2 * s2) * gPos[0]), ((L1 + L2 * c2) * gPos[0]) + ((L2 * s2) * gPos[1]));

	if (theta1 < joint1Lim[0] || theta1 > joint1Lim[1])
	{
		cout << "Joint 1 out of limits as \u03B82 equals" << toDegree(theta1) << endl;

		return solVector;
	}

	float c2_2 = cos(-theta2);
	float s2_2 = sin(-theta2);

	float theta1_2 = atan2(((L1 + L2 * c2_2) * gPos[1]) - ((L2 * s2_2) * gPos[0]), ((L1 + L2 * c2_2) * gPos[0]) + ((L2 * s2_2) * gPos[1]));

	if ((theta1_2 < joint1Lim[0] || theta1_2 > joint1Lim[1]) && oneSolution == false)
	{
		cout << "There is only one solution" << toDegree(theta1_2) << endl;

		oneSolution = 1;
	}

	// Calculating theta 3 valuse

	float theta3 = gPos[3] - theta1 - theta2; while (theta3 > 2 * pi) { theta3 = theta3 - 2 * pi; }
	float theta3_2 =  gPos[3] - theta1_2 + theta2; while (theta3_2 > 2 * pi) { theta3_2 = theta3_2 - 2 * pi; }

	//cout << "\u03B84  is" << toDegree(-theta3) << " and \u03B84_2  is  " << toDegree(-theta3_2) << endl;

	if (theta3 < joint4Lim[0] || theta3 > joint4Lim[1])
	{
		cout << "Out of joint 4 lim as \u03B84 equals " << theta3 << endl;
		return solVector;
	}

	if ((theta3_2 < joint4Lim[0] || theta3_2 > joint4Lim[1]) && oneSolution == false)
	{
		cout << "Joint 4 out of limits as \u03B84 equals " << theta3 << endl;
		oneSolution = 1;
	}

	// Calculating Joint 3 value
	float d = -70 - gPos[2];   // this is important (405+70)-(135+z+410+d)

	if (d < joint3Lim[0] || d > joint3Lim[1])
	{
		cout << "Joint 3 out of lim as H3 equals" << d << endl;
		return solVector;
	}

	if (oneSolution == false)
	{
		solVector << toDegree(theta1), toDegree(theta1_2),
			toDegree(theta2), toDegree(-theta2),
			d, d,
			-toDegree(theta3), -toDegree(theta3_2);
	}
	else {
		solVector << toDegree(theta1), 0,
			toDegree(theta2), 0,
			d, 0,
			-toDegree(theta3), 0;
	}
	return solVector;
}

void moveToCoordinates() //WIP
{
	// initialization	
	JOINT S_1;
	JOINT S_2;
	JOINT S_3;
	JOINT current = { 90,0,-175,0 };
	MatrixXf solVector(4, 2);

	char ch;
	int c;
	const int ESC = 27;

	cout << "Please press any key to start or ESC to go back" << endl;
	c = _getch();

	while (1)
	{

		if (c != ESC)
		{
			// getting inputs from the user 
			float q[4];
			cout << "Please enter the value of X [mm]: ";
			cin >> q[0];
			cout << "Please enter the value of Y [mm]: ";
			cin >> q[1];
			cout << "Please enter the value of Z [mm]: ";
			cin >> q[2];
			cout << "Please enter the value of \u03C6 [°]: ";
			cin >> q[3];

			//making sure the input is not more than 2pi
			while (q[3] >= 360)
			{
				q[3] = q[3] - 360;
			}

			//solve for the inverse
			solVector = inverseKinematics(q);

			//display the solution/s
			cout << "The solution/s is/are: " << endl;
			cout << solVector << endl;

			//loading the solutions in 2 joints
			for (int i = 0; i < 4; i++)
			{
				S_1[i] = solVector(i, 0);
			}

			for (int i = 0; i < 4; i++)
			{
				S_2[i] = solVector(i, 1);
			}

			if (solVector(2, 0) != 0)
			{
				printf("Press '1' for solution 1, '2' for solution 2 or '3' for the closest solution  \n");

				ch = _getch();

				//executing the desired solution
				if (ch == '1')
				{
					MoveToConfiguration(S_1);
					for (int i = 0; i < 4; i++) { current[i] = S_1[i]; }
				}
				else if (ch == '2')
				{
					MoveToConfiguration(S_2);
					for (int i = 0; i < 4; i++) { current[i] = S_2[i]; }
				}
				else if (ch == '3')
				{
					//dif(circile_solution)
					// use get configuration and absoulute
					// change abs to more accurate
					float dif1_1, dif1_2, dif2_1, dif2_2;
					dif1_1 = S_1[0] - current[0]; while (dif1_1 > 360) { dif1_1 = dif1_1 - 360; } while (dif1_1 < -360) { dif1_1 = dif1_1 + 360; }
					dif1_2 = S_2[0] - current[0]; while (dif1_2 > 360) { dif1_2 = dif1_2 - 360; } while (dif1_2 < -360) { dif1_2 = dif1_2 + 360; }
					dif2_1 = S_1[1] - current[1]; while (dif2_1 > 360) { dif2_1 = dif2_1 - 360; } while (dif2_1 < -360) { dif2_1 = dif2_1 + 360; }
					dif2_2 = S_2[1] - current[1]; while (dif2_2 > 360) { dif2_2 = dif2_2 - 360; } while (dif2_2 < -360) { dif2_2 = dif2_2 + 360; }

					if ((abs(L1 * toRad(dif1_1)) + abs( L2 * toRad(dif2_1))) > (abs(L1 * toRad(dif1_2)) + abs(L2 * toRad(dif2_2))) || solVector(2, 1) == 0)
					{
						MoveToConfiguration(S_1);
						for (int i = 0; i < 4; i++) { current[i] = S_1[i]; }
					}
					else
					{
						MoveToConfiguration(S_2);
						for (int i = 0; i < 4; i++) { current[i] = S_2[i]; }
					}

				}

				c = _getch();
			}
		}
		else
			break;


	}


}

void jointVectorInput()
{
	/* Ask for the joint vector to be input in absolute coordinates and check that
	each of the responses falls within its limits.
	Store this as a joint vector called JV */
	float JV[4];
	bool lim, rep = true;
	JOINT g1;
	float Poise[4];
	float c1, s1, c12, s12, c24, s42;
	int c;

	do {
		do {
			cout << "Please enter the value of \u03B81 [°]: ";
			//degree symbol does not display in console either by text or HTML symbol call
			cin >> g1[0];
			JV[0] = toRad(g1[0]);
			if (JV[0] < joint1Lim[0] || JV[0] > joint1Lim[1])
			{
				cout << "Joint 1 out of limits as \u03B81 must be between -150° and +150°";
				lim = false;
			}
			else { lim = true; }
		} while (lim == false);
		do {
			cout << "Please enter the value of \u03B82 [°]: ";
			cin >> g1[1];
			JV[1] = toRad(g1[1]);
			if (JV[1] < joint2Lim[0] || JV[1] > joint2Lim[1])
			{
				cout << "Joint 2 out of limits as \u03B82 must be between -100° and +100°";
				lim = false;
			}
			else { lim = true; }
		} while (lim == false);
		do {
			cout << "Please enter the value of H3 [mm]: ";
			cin >> g1[2];
			JV[2] = g1[2];
			if (JV[2] < jointH3Lim[0] || JV[2] > jointH3Lim[1])
			{
				cout << "Joint 3 out of limits as H3 must be between -100mm and -200mm";
				lim = false;
			}
			else { lim = true; }
		} while (lim == false);
		do {
			cout << "Please enter the value of \u03B84 [°]: ";
			cin >> g1[3];
			JV[3] = toRad(g1[3]);
			if (JV[3] < joint4Lim[0] || JV[3] > joint4Lim[1])
			{
				cout << "Joint 4 out of limits as \u03B84 must be between -150° and +150°";
				lim = false;
			}
			else { lim = true; }
		} while (lim == false);

		/*Calculate the Poise of the gripper given the joint vector and print it numerically*/
		float Poise[4];
		float c1 = cos(JV[0]);
		float s1 = sin(JV[0]);
		float c12 = cos(JV[0] + JV[1]);
		float s12 = sin(JV[0] + JV[1]);
		float c24 = cos(JV[1] - JV[3]);
		float s42 = sin(JV[3] - JV[1]);
		Poise[0] = c1 * L1 + c12 * L2;
		Poise[1] = s1 * L1 + s12 * L2;
		Poise[2] = H1 + H2 - (JV[2] + 270 + H4 + H5);
		Poise[3] = atan2(c1 * s42 - s1 * c24, -s1 * s42 - c1 * c24) + pi;
		cout << "\n" << "The input joint vector will put the centre of the grabber at:\n";
		cout << "X = " << Poise[0] << " mm \n";
		cout << "Y = " << Poise[1] << " mm \n";
		cout << "Z = " << Poise[2] << " mm \n";
		cout << "\u03C6 = " << toDegree(Poise[3]) << " ° \n";

		/*Display the Poise of the gripper at the given joint vector*/

		DisplayConfiguration(g1);
		printf("\nPress any key to continue or press ESC to exit \n");
		c = _getch();
		if (c != 27)//27 is the value for ESC key press
		{
			cout << "\n\n";
			rep = true;
		}
		else
			break;

	} while (rep);
}
