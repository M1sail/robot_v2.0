#include "stdafx.h"
#include"robot.h"

SMSBL sm;
Driver dr;

int main(int argc, char** argv)
{
	if (argc < 2) {                  //�����Ƿ����ö˿�
		cout << "argc error!" << endl;
		getchar();
		return 0;
	}
	cout << "serial:" << argv[1] << endl;
	if (!sm.begin(115200, argv[1])) {                  //���鲨���ʼ��˿��Ƿ���ȷ����ʼ�����
		cout << "Failed to init smb motor!" << endl;
		getchar();
		return 0;
	}

	dr.ini1(sm);						//��ʼ����е��

	delay(5000);

	float tmax = 16;

	for (float t = 0.00; t < tmax; t += 0.2)
	{
		cout << t << endl;                 //��ʾʱ��

		CTRL_DATA* data1 = new CTRL_DATA;

		data1->R.Position[0] = 160 + 0.5 * 100 * cos(t * Pi / 2) - 20;
		data1->R.Position[1] = 300 + 100 * sin(t * Pi / 2);
		data1->R.Position[2] = -270;

		data1->R.Posture[0] = 15 * Pi / 180;

		data1->L.Position[0] = -160 - 0.5 * 0 * cos(t * Pi / 2) + 20;
		data1->L.Position[1] = 300 - 100 * sin(t * Pi / 2);
		data1->L.Position[2] = -270;

		data1->L.Posture[0] = -15 * Pi / 180;

		dr.drive(data1, sm);                 //������ؽڽǲ�д����

		delete data1;

		delay(150);
	}

	return 0;
}


