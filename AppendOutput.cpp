	double theta1Cur[2000];
	double theta2Cur[2000];
	double distance3Cur[2000];
	double theta4Cur[2000];

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
		theta1Cur[i]=cur[0]; theta2Cur[i]=cur[1]; distance3Cur[i]=cur[2]; theta4Cur[i]=cur[3];

		cout << "moved? " << re << endl;

		//MoveToConfiguration(pos, true);
		cout << "planned pos" << pos[0] << "   " << pos[1] << "   " << pos[2] << "   " << pos[3] << endl;
		cout << "current pos" << cur[0] << "   " << cur[1] << "   " << cur[2] << "   " << cur[3] << endl;

		//wait(50);
		Sleep(20);// duration must be calculated as time in frame minus duration of execution

	}

#include <fstream>

void write_csv(double theta1Cur[],double theta1Pos,[]double theta2Pos[],double distance3Pos[],double theta4Pos[])
{
    float Xcur,Ycur;
    ofstream file("Output");
	file << "Time,\u03B81_planned,\u03B81_current,\u03B82_planned,\u03B82_current,h3_planned,h3_current,\u03B84_planned,\u03B84_current,x,y\n";
    for(int i = 0; i < index; ++i)
    {
    	//calculate X and Y positions for each frame using the current position
    	c1 = cos(theta1Cur[i]); s1 = sin(theta1Cur[i]); c12 = cos(theta1Cur[i] + theta2Cur[i]);	s12 = sin(theta1Cur[i] + theta2Cur[i]);
    	Xcur = c1 * L1 + c12 * L2; Ycur = s1 * L1 + s12 * L2;
		//send values as single line accroding to header order
		file <<indext[i]<<","<<theta1Pos[i]<<","<<theta1Cur[i]<<","<<","<<theta2Pos[i]<<","<<theta2Cur[i]<<","<<distance3Pos[i]<<","<<distance3Cur[i]<<","<<theta4Pos[i]<<","<<theta4Cur[i]<<","<<Xcur<<","<<Ycur<<"\n";
    }
    file.close();
}

}