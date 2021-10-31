#include "stdafx.h"
#include"robot.h"

SMSBL sm;
Driver dr;

int main(int argc, char** argv)
{
	if (argc < 2) {                  //检验是否设置端口
		cout << "argc error!" << endl;
		getchar();
		return 0;
	}
	cout << "serial:" << argv[1] << endl;
	if (!sm.begin(115200, argv[1])) {                  //检验波特率及端口是否正确并初始化舵机
		cout << "Failed to init smb motor!" << endl;
		getchar();
		return 0;
	}

	dr.ini1(sm);						//初始化机械臂

	delay(5000);

	float tmax = 16;

	for (float t = 0.00; t < tmax; t += 0.2)
	{
		cout << t << endl;                 //显示时间

		CTRL_DATA* data1 = new CTRL_DATA;

		data1->R.Position[0] = 160 + 0.5 * 100 * cos(t * Pi / 2) - 20;
		data1->R.Position[1] = 300 + 100 * sin(t * Pi / 2);
		data1->R.Position[2] = -270;

		data1->R.Posture[0] = 15 * Pi / 180;

		data1->L.Position[0] = -160 - 0.5 * 0 * cos(t * Pi / 2) + 20;
		data1->L.Position[1] = 300 - 100 * sin(t * Pi / 2);
		data1->L.Position[2] = -270;

		data1->L.Posture[0] = -15 * Pi / 180;

		dr.drive(data1, sm);                 //计算各关节角并写入舵机

		delete data1;

		delay(150);
	}

	return 0;
}


