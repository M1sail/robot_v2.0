#pragma once
#include<cmath>
#include <ctime> 
#include <cstring>
#include <Windows.h>
#include <iostream>
#include "stdafx.h"
#include "SCServo/SCServo.h"

using namespace std;

//Ҽ����ͨ�ú�����

const float Pi = 3.1415926;

//3*1�������
void matrix_multiply_3x1(float C[], float A[][3], float B[]);

//3*3�������
void matrix_multiply3(float c[][3], float a[][3], float b[][3]);

//3*3��������
void matrix_inverse3(float B[][3], float A[][3]);

//ӳ�亯��
float map(float num, float s1min, float s1max, float s2min, float s2max);

//�ӳٿ��ƺ���
void delay(int time);

//�������������� �ṹ�� (ȫ�ֲ�����

struct data
{
	//λ�� (Position) \ ��̬ (Posture) \ ���� (Gesture)
	float Position[3];
	float Posture[3];
	float Gesturen[3];
};

struct CTRL_DATA
{
	struct data R = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	struct data L = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	float H[3] = { 0, 0, 0 };
};

//����������

class Driver
{
public:
	void ini1(SMSBL& sm);			//ini1 - ��ʼ�� �� ̧����
	void drive(const CTRL_DATA *data, SMSBL& sm, float fai = -1);

private:
	//=================��������=================//
	float armLen[4] = { 270, 240, 100, 160 };                  //������˳�

	float pos_R[3] = { 0, 0, 0 };						//����ĩ��λ�á���̬
	float pos_L[3] = { 0, 0, 0 };
	float ang_R[3] = { 0, 0, 0 };
	float ang_L[3] = { 0, 0, 0 };

	float th_R[7] = { 0, 0, 0, 0, 0, 0, 0 };                  //�����ؽڵĽǶ�
	float th_L[7] = { 0, 0, 0, 0, 0, 0, 0 };
	float th_H[3] = { 0, 0, 0 };
	float th[7] = { 0, 0, 0, 0, 0, 0, 0 };

	//=================������̱���=================//
	float II[3][3] = { {-1, 0, 0}, {0, -1, 0}, {0, 0, -1} };			        //����λ����

	float pos8[3] = { 0, 0, 0 };                   //ĩ������ֱ�����{0}λ�á���̬
	float rot8[3][3];

	float rotB_R[3][3] = { {0, 0, -1}, {0, 1, 0}, {1, 0, 0} };          //�ұ�����{0}�Ի�������ת����λ��
	float rotB_L[3][3] = { {0, 0, -1}, {0, 1, 0}, {1, 0, 0} };
	float posB_R[3] = { 0, 0, -armLen[3] };
	float posB_L[3] = { 0, 0, armLen[3] };

	float pos5[3] = { 0, 0, 0 };                 //��ؽ�λ������
	float usw[3] = { 0, 0, 0 };                  //ԭ�㵽��ؽڵĵ�λʸ��

	float fai_R = 80 * Pi / 180;                    //���νǦ�
	float fai_L = 90 * Pi / 180;

	//==============================��д�������==============================//
	int v = 200;                 //���ת�١����ٶ�
	int a = 20;

	int pth_R[7] = { 2047,2047,2047,1023,2047,2047,2047 };                 // �����ʼλ��
	int pth_L[7] = { 2047,2047,2047,1023,2047,2047,2047 };
	int pth_H[3] = { 2047,2047,2047 };
	int pth_R_pre[7] = { 2047,2047,2047,1023,2047,2047,2047 };
	int pth_L_pre[7] = { 2047,2047,2047,1023,2047,2047,2047 };
	int pth_H_pre[3] = { 2047,2047,2047 };

	int ptm[7];
	int ptm_read[7];
	int ptm_pre[7];

	int map_R[7][4] = { {-90, 90, 3071, 1023},    //�ؽڽǶȵ����λ�õ�ӳ�����
								   {0, 100, 1023, 2161},
								   {0, 120, 3071, 1705},
								   {0, 125, 2047, 625},
								   {-90, 90, 3071, 1023},
								   {70, 110, 1477, 2617},
								   {-75, 75, 2900, 1194}};
	int map_L[7][4] = { {-90, 90, 1023, 3071},    //�ؽڽǶȵ����λ�õ�ӳ�����
								  {80, 180, 1933, 3071},
								  {0, 180, 3071, 1023},
								  {0, 125, 2047, 625},
								  {-90, 90, 3071, 1023},
								  {70, 110, 2617, 1477},
								  {-75, 75, 1194, 2900}};
	int map_H[3][4] = { {-90, 90, 3071, 1023},    //�ؽڽǶȵ����λ�õ�ӳ�����
								   {-20, 20, 2617, 1477},
								   {-22, 90, 1797, 3071} };
	int map0[7][4];

private:
	//=================���㡢��д=================//
	void clc(float pos82B[], float ang82B[], int isRL, float fai = -1);

	int insure(int p[], int isRL);

	void wr(float p[], int is_RL, SMSBL& sm);
};