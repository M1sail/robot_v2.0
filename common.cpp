#include "stdafx.h"
#include "robot.h"  

//================================3*3æÿ’Ûœ‡≥À================================
void matrix_multiply3(float c[][3], float a[][3], float b[][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                c[i][j] = c[i][j] + a[i][k] * b[k][j];
            }
        }
    }
}

//================================3*1æÿ’Ûœ‡≥À================================
void matrix_multiply_3x1(float C[], float A[][3], float B[])
{
    for (int i = 0; i < 3; i++) {
        C[i] = 0;
        for (int k = 0; k < 3; k++) {
            C[i] = C[i] + A[i][k] * B[k];
        }
    }
}

//================================3*3æÿ’Û«ÛƒÊ================================
void matrix_inverse3(float B[][3], float A[][3])
{
    float detA = A[0][0] * A[1][1] * A[2][2] + A[0][1] * A[1][2] * A[2][0] + A[0][2] * A[1][0] * A[2][1] - A[0][2] * A[1][1] * A[2][0] - A[0][1] * A[1][0] * A[2][2] - A[0][0] * A[1][2] * A[2][1];
    B[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]) / detA;
    B[0][1] = -(A[0][1] * A[2][2] - A[2][1] * A[0][2]) / detA;
    B[0][2] = (A[0][1] * A[1][2] - A[1][1] * A[0][2]) / detA;
    B[1][0] = -(A[1][0] * A[2][2] - A[2][0] * A[1][2]) / detA;
    B[1][1] = (A[0][0] * A[2][2] - A[2][0] * A[0][2]) / detA;
    B[1][2] = -(A[0][0] * A[1][2] - A[1][0] * A[0][2]) / detA;
    B[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]) / detA;
    B[2][1] = -(A[0][0] * A[2][1] - A[2][0] * A[0][1]) / detA;
    B[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]) / detA;
}

float map(float num, float s1min, float s1max, float s2min, float s2max)            //∞—num¥”s1∑∂Œß”≥…‰µΩs2∑∂Œß
{
    float num1;
    num1 = (num - s1min) * (s2max - s2min) / (s1max - s1min) + s2min;
    return num1;
}

void delay(int time)					//µ•Œª∫¡√Î
{
    clock_t   now = clock();

    while (clock() - now < time);
}


