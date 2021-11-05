#pragma once
#include<cmath>
#include <ctime> 
#include <cstring>
#include <Windows.h>
#include <iostream>
#include "stdafx.h"
#include "SCServo/SCServo.h"

using namespace std;

//壹——通用函数：

const float Pi = 3.1415926;

//3*1矩阵相乘
void matrix_multiply_3x1(float C[], float A[][3], float B[]);

//3*3矩阵相乘
void matrix_multiply3(float c[][3], float a[][3], float b[][3]);

//3*3矩阵求逆
void matrix_inverse3(float B[][3], float A[][3]);

//映射函数
float map(float num, float s1min, float s1max, float s2min, float s2max);

//延迟控制函数
void delay(int time);

//贰——控制数据 结构体 (全局参数）

struct data
{
	//位置 (Position) \ 姿态 (Posture) \ 手势 (Gesture)
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

//叁——驱动

class Driver
{
public:
	void ini1(SMSBL& sm);			//ini1 - 初始化 · 抬起手
	void drive(const CTRL_DATA *data, SMSBL& sm, float fai = -1);

private:
	//=================公共变量=================//
	float armLen[4] = { 270, 240, 100, 160 };                  //定义各杆长

	float pos_R[3] = { 0, 0, 0 };						//左右末端位置、姿态
	float pos_L[3] = { 0, 0, 0 };
	float ang_R[3] = { 0, 0, 0 };
	float ang_L[3] = { 0, 0, 0 };

	float th_R[7] = { 0, 0, 0, 0, 0, 0, 0 };                  //各个关节的角度
	float th_L[7] = { 0, 0, 0, 0, 0, 0, 0 };
	float th_H[3] = { 0, 0, 0 };
	float th[7] = { 0, 0, 0, 0, 0, 0, 0 };

	//=================计算过程变量=================//
	float II[3][3] = { {-1, 0, 0}, {0, -1, 0}, {0, 0, -1} };			        //负单位矩阵

	float pos8[3] = { 0, 0, 0 };                   //末端相对手臂坐标{0}位置、姿态
	float rot8[3][3];

	float rotB_R[3][3] = { {0, 0, -1}, {0, 1, 0}, {1, 0, 0} };          //右臂坐标{0}对基坐标旋转矩阵及位移
	float rotB_L[3][3] = { {0, 0, -1}, {0, 1, 0}, {1, 0, 0} };
	float posB_R[3] = { 0, 0, -armLen[3] };
	float posB_L[3] = { 0, 0, armLen[3] };

	float pos5[3] = { 0, 0, 0 };                 //腕关节位置坐标
	float usw[3] = { 0, 0, 0 };                  //原点到腕关节的单位矢量

	float fai_R = 80 * Pi / 180;                    //臂形角φ
	float fai_L = 90 * Pi / 180;

	//==============================读写舵机变量==============================//
	int v = 200;                 //舵机转速、加速度
	int a = 20;

	int pth_R[7] = { 2047,2047,2047,1023,2047,2047,2047 };                 // 舵机初始位置
	int pth_L[7] = { 2047,2047,2047,1023,2047,2047,2047 };
	int pth_H[3] = { 2047,2047,2047 };
	int pth_R_pre[7] = { 2047,2047,2047,1023,2047,2047,2047 };
	int pth_L_pre[7] = { 2047,2047,2047,1023,2047,2047,2047 };
	int pth_H_pre[3] = { 2047,2047,2047 };

	int ptm[7];
	int ptm_read[7];
	int ptm_pre[7];

	int map_R[7][4] = { {-90, 90, 3071, 1023},    //关节角度到舵机位置的映射矩阵
								   {0, 100, 1023, 2161},
								   {0, 120, 3071, 1705},
								   {0, 125, 2047, 625},
								   {-90, 90, 3071, 1023},
								   {70, 110, 1477, 2617},
								   {-75, 75, 2900, 1194}};
	int map_L[7][4] = { {-90, 90, 1023, 3071},    //关节角度到舵机位置的映射矩阵
								  {80, 180, 1933, 3071},
								  {0, 180, 3071, 1023},
								  {0, 125, 2047, 625},
								  {-90, 90, 3071, 1023},
								  {70, 110, 2617, 1477},
								  {-75, 75, 1194, 2900}};
	int map_H[3][4] = { {-90, 90, 3071, 1023},    //关节角度到舵机位置的映射矩阵
								   {-20, 20, 2617, 1477},
								   {-22, 90, 1797, 3071} };
	int map0[7][4];

private:
	//=================计算、读写=================//
	void clc(float pos82B[], float ang82B[], int isRL, float fai = -1);

	int insure(int p[], int isRL);

	void wr(float p[], int is_RL, SMSBL& sm);
};