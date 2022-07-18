

#include <fstream>
///// this if where your function needs to change
	double Positions[2000][8]

	bool re = 1;
	JOINT cur;

	for (int i = 0; i < index ; i++)
	{
		JOINT pos;
		pos[0] = theta1Pos[i]; pos[1] = theta2Pos[i]; pos[2] = distance3Pos[i]; pos[3] = theta4Pos[i];
		JOINT vel;
		vel[0] = theta1Vel[i]; vel[1] = theta2Vel[i]; vel[2] = distance3Vel[i]; vel[3] = theta4Vel[i];
		JOINT acc;
		acc[0] = theta1Acc[i]; acc[1] = theta2Acc[i]; acc[2] = distance3Acc[i]; acc[3] = theta4Acc[i];
		
		re = MoveWithConfVelAcc(pos, vel, acc);

		GetConfiguration(cur);
		
		for (int j = 0; j < 8; j=j+2)
			Positions[i][j]=pos[j];
		for (int k = 1; k < 8; k=k+2)
			Positions[i][k]=cur[k];

		cout << "moved? " << re << endl;

		//MoveToConfiguration(pos, true);
		cout << "planned pos" << pos[0] << "   " << pos[1] << "   " << pos[2] << "   " << pos[3] << endl;
		cout << "current pos" << cur[0] << "   " << cur[1] << "   " << cur[2] << "   " << cur[3] << endl;

		//wait(50);
		Sleep(20);// duration must be calculated as time in frame minus duration of execution

	} return Positions;

#include <fstream>

void write_csv(string name,double dataMatrix[2000][8],int rows)
{
    float time=0.0,Xcur,Ycur;
    float c1,c12,s1,s12;
    ofstream file(name);
	file << "Time,\u03B81_planned,\u03B81_current,\u03B82_planned,\u03B82_current,h3_planned,h3_current,\u03B84_planned,\u03B84_current,x,y\n";
    for(int i = 0; i < rows; ++i)
    {
		//calculate X and Y positions for each frame using the current position
    	c1 = cos(theta1Cur[i]); s1 = sin(theta1Cur[i]); c12 = cos(theta1Cur[i] + theta2Cur[i]);	s12 = sin(theta1Cur[i] + theta2Cur[i]);
    	Xcur = c1 * L1 + c12 * L2; Ycur = s1 * L1 + s12 * L2;
		//send values as single line accroding to header order
		file <<time+(0.04)<<","<<dataMatrix[i][0]<<","<<dataMatrix[i][1]<<","<<dataMatrix[i][2]<<","<<dataMatrix[i][3]<<","<<dataMatrix[i][4]<<","<<dataMatrix[i][5]<<","<<dataMatrix[i][6]<<","<<dataMatrix[i][7]<<","<<Xcur<<","<<Ycur<<"\n";
    }
    file.close();
}

}