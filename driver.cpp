#include "stdafx.h"
#include "robot.h"  

//isRL  0右1左

void Driver::clc(float pos82B[], float ang82B[], int isRL)
{
    float fai = 0;

    float rot82B[3][3] = { {cos(ang82B[0]) * cos(ang82B[1]), cos(ang82B[0]) * sin(ang82B[1]) * sin(ang82B[2]) - sin(ang82B[0]) * cos(ang82B[2]), cos(ang82B[0]) * sin(ang82B[1]) * cos(ang82B[2]) + sin(ang82B[0]) * sin(ang82B[2])},
                                     {sin(ang82B[0]) * cos(ang82B[1]), sin(ang82B[0]) * sin(ang82B[1]) * sin(ang82B[2]) + cos(ang82B[0]) * cos(ang82B[2]), sin(ang82B[0]) * sin(ang82B[1]) * cos(ang82B[2]) - cos(ang82B[0]) * sin(ang82B[2])},
                                     {-sin(ang82B[1]), cos(ang82B[1]) * sin(ang82B[2]), cos(ang82B[1]) * cos(ang82B[2])} };
    //----------计算手掌相对肩关节坐标系{0}的位置和转角
    if (isRL == 0)
    {
        float temp[3]; matrix_multiply_3x1(temp, rotB_R, pos82B);
        for (int i = 0; i < 3; i++) { pos8[i] = temp[i] + posB_R[i]; }
        matrix_multiply3(rot8, rotB_R, rot82B);
    }
    else if (isRL == 1)
    {
        float temp[3]; matrix_multiply_3x1(temp, rotB_L, pos82B);
        for (int i = 0; i < 3; i++) { pos8[i] = temp[i] + posB_L[i]; }
        matrix_multiply3(rot8, rotB_L, rot82B);
    }

    //----------先求手腕位置
    for (int i = 0; i < 3; i++) { pos5[i] = pos8[i] + rot8[i][1] * (-armLen[2]); }

    //----------臂形角为零时，利用腕关节{5}在基座标{0}的位置计算θ1~θ4
    th[2] = 0;                  //th[2]即为θ3 = 0

    float lsw = sqrt(pos5[0] * pos5[0] + pos5[1] * pos5[1] + pos5[2] * pos5[2]);                //利用肩肘腕三角形求θ4
    th[3] = Pi - acos((armLen[0] * armLen[0] + armLen[1] * armLen[1] - lsw * lsw) / (2 * armLen[0] * armLen[1]));

    float TA = armLen[0] + armLen[1] * cos(th[3]);
    float TB = -armLen[1] * sin(th[3]);
    float TC = -pos5[2];
    th[1] = abs(TC - TA) > 0.001 ? 2 * atan((-TB - sqrt(TA * TA + TB * TB - TC * TC)) / (TC - TA)) : Pi;

    th[0] = atan(pos5[1] / pos5[0]);

    //----------臂形角不为零时，再求θ1~θ3
    float rot30[][3] = { {cos(th[0]) * cos(th[1]), -sin(th[0]), cos(th[0]) * sin(th[1])},
                                {sin(th[0]) * cos(th[1]), cos(th[0]), sin(th[0]) * sin(th[1])},
                                {-sin(th[1]), 0, cos(th[1])} };                 //坐标系{3}相对于{0}的旋转矩阵
    for (int i = 0; i < 3; i++) { usw[i] = pos5[i] / lsw; }                 //原点到腕关节的单位矢量
    float uswx[][3] = { {0, -usw[2], usw[1]},
                                {usw[2], 0, -usw[0]},
                                {-usw[1], usw[0], 0} };                 //usw的斜对称矩阵
    float As[3][3]; matrix_multiply3(As, uswx, rot30);
    float Bs_1[3][3]; matrix_multiply3(Bs_1, uswx, uswx);
    float Bs_2[3][3]; matrix_multiply3(Bs_2, Bs_1, rot30);
    float Bs[3][3]; matrix_multiply3(Bs, Bs_2, II);
    float tusw[3][3] = { {usw[0] * usw[0], usw[0] * usw[1], usw[0] * usw[2]},
                                  {usw[1] * usw[0], usw[1] * usw[1], usw[1] * usw[2]},
                                  {usw[2] * usw[0], usw[2] * usw[1], usw[2] * usw[2]} };
    float Cs[3][3]; matrix_multiply3(Cs, tusw, rot30);                  //系数A、B、C

    //----------臂形角φ
    if (isRL == 0) { fai = fai_R; }
    else { fai = fai_L; }

    float fai_temp; float funSig[7]; float funTol; float fai_min = 0; float funTol_min = 10;

    for (int i = 0; i < 11; i++)
    {
        fai_temp = fai + (i - 5) * Pi / 180;
        th[0] = atan((sin(fai_temp) * As[1][2] + cos(fai_temp) * Bs[1][2] + Cs[1][2]) / (sin(fai_temp) * As[0][2] + cos(fai_temp) * Bs[0][2] + Cs[0][2]));
        th[1] = acos(sin(fai_temp) * As[2][2] + cos(fai_temp) * Bs[2][2] + Cs[2][2]);
        th[2] = acos(-(sin(fai_temp) * As[2][0] + cos(fai_temp) * Bs[2][0] + Cs[2][0]) / sin(th[1]));
        //----------th5~th7
        float rot5[][3] = { {-cos(th[3]) * (sin(th[0]) * sin(th[2]) - cos(th[0]) * cos(th[1]) * cos(th[2])) - cos(th[0]) * sin(th[1]) * sin(th[3]),
                                  -sin(th[0]) * cos(th[2]) - cos(th[0]) * cos(th[1]) * sin(th[2]),
                                  -sin(th[3]) * (sin(th[0]) * sin(th[2]) - cos(th[0]) * cos(th[1]) * cos(th[2])) + cos(th[0]) * sin(th[1]) * cos(th[3])},
                                {cos(th[3]) * (cos(th[0]) * sin(th[2]) + sin(th[0]) * cos(th[1]) * cos(th[2])) - sin(th[0]) * sin(th[1]) * sin(th[3]),
                                 cos(th[0]) * cos(th[2]) - sin(th[0]) * cos(th[1]) * sin(th[2]),
                                 sin(th[3]) * (cos(th[0]) * sin(th[2]) + sin(th[0]) * cos(th[1]) * cos(th[2])) + sin(th[0]) * sin(th[1]) * cos(th[3])},
                               {-cos(th[1]) * sin(th[3]) - sin(th[1]) * cos(th[2]) * cos(th[3]),
                                sin(th[1]) * sin(th[2]),
                                -sin(th[1]) * cos(th[2]) * sin(th[3]) + cos(th[1]) * cos(th[3])} };
        float rot5_inv[3][3]; matrix_inverse3(rot5_inv, rot5);
        float rot8_5[3][3]; matrix_multiply3(rot8_5, rot5_inv, rot8);
        th[5] = acos(rot8_5[2][2]);
        th[4] = atan(rot8_5[1][2] / rot8_5[0][2]);
        th[6] = atan(rot8_5[2][0] / rot8_5[2][1]);

        //----------适应度函数
        for (int j = 0; j < 7; j++) { funSig[j] = 1 - 4 * (map_R[j][0] - th[j] * 180 / Pi) * (th[j] * 180 / Pi - map_R[j][1]) / pow((map_R[j][0] - map_R[j][1]), 2); }
        funTol = funSig[5] + 2 * funSig[6];

        fai_min = funTol_min > funTol ? fai_temp : fai_min;
        funTol_min = funTol_min > funTol ? funTol : funTol_min;
    }

    fai = fai_min;
    if (isRL == 0) { fai_R = fai; }
    else { fai_L = fai; }

    th[0] = atan((sin(fai) * As[1][2] + cos(fai) * Bs[1][2] + Cs[1][2]) / (sin(fai) * As[0][2] + cos(fai) * Bs[0][2] + Cs[0][2]));
    th[1] = acos(sin(fai) * As[2][2] + cos(fai) * Bs[2][2] + Cs[2][2]);
    th[2] = atan((-sin(fai) * As[2][1] - cos(fai) * Bs[2][1] - Cs[2][1]) / (sin(fai) * As[2][0] + cos(fai) * Bs[2][0] + Cs[2][0]));
    if (th[2] < 0) { th[2] = th[2] + Pi; };

    //----------最后求θ5~θ7
    float rot5[][3] = { {-cos(th[3]) * (sin(th[0]) * sin(th[2]) - cos(th[0]) * cos(th[1]) * cos(th[2])) - cos(th[0]) * sin(th[1]) * sin(th[3]),
                                -sin(th[0]) * cos(th[2]) - cos(th[0]) * cos(th[1]) * sin(th[2]),
                                -sin(th[3]) * (sin(th[0]) * sin(th[2]) - cos(th[0]) * cos(th[1]) * cos(th[2])) + cos(th[0]) * sin(th[1]) * cos(th[3])},
                                {cos(th[3]) * (cos(th[0]) * sin(th[2]) + sin(th[0]) * cos(th[1]) * cos(th[2])) - sin(th[0]) * sin(th[1]) * sin(th[3]),
                                 cos(th[0]) * cos(th[2]) - sin(th[0]) * cos(th[1]) * sin(th[2]),
                                 sin(th[3]) * (cos(th[0]) * sin(th[2]) + sin(th[0]) * cos(th[1]) * cos(th[2])) + sin(th[0]) * sin(th[1]) * cos(th[3])},
                                {-cos(th[1]) * sin(th[3]) - sin(th[1]) * cos(th[2]) * cos(th[3]),
                                 sin(th[1]) * sin(th[2]),
                                 -sin(th[1]) * cos(th[2]) * sin(th[3]) + cos(th[1]) * cos(th[3])} };
    float rot5_inv[3][3]; matrix_inverse3(rot5_inv, rot5);
    float rot8_5[3][3]; matrix_multiply3(rot8_5, rot5_inv, rot8);
    th[4] = atan(rot8_5[1][2] / rot8_5[0][2]);
    th[5] = acos(rot8_5[2][2]);
    th[6] = atan(rot8_5[2][0] / rot8_5[2][1]);

    if (isRL == 0)
    {
        for (int i = 0; i < 7; i++)
        {
            th_R[i] = th[i];
        }
    }
    else
    {
        for (int i = 0; i < 7; i++)
        {
            th_L[i] = th[i];
        }
    }
}

int Driver::insure(int p[], int isRL)                   //判定是否超出舵机转角范围
{
    int cnt = 0;
    for (int i = 0; i < 7; i++)
    {
        if (abs(p[i] - (map0[i][2] + map0[i][3]) / 2) < abs((map0[i][2] - map0[i][3]) / 2) + 2)
        {
            cnt += pow(10, i);
        }
    }

    if (cnt == 1111111)
        return 1;

    else
    {
        cout << isRL << ":" << cnt << endl;
        return 0;
    }
}

void Driver::wr(float pth[], int isRL, SMSBL& sm)
{
    int p[7] = { 0, 0, 0, 0, 0, 0, 0 }; //
    int num;

    if (isRL == 0)
    {
        memcpy(map0, map_R, 112);
        num = 1;
    }
    else if (isRL == 1)
    {
        memcpy(map0, map_L, 112);
        num = 11;
    }
    else {}

    for (int i = 0; i < 7; i++)
    {
        p[i] = int(map(pth[i] * 180 / Pi, map0[i][0], map0[i][1], map0[i][2], map0[i][3]));
    }

    if (isRL == 0)
    {
        for (int i = 0; i < 7; i++)
        {
            pth_R_pre[i] = pth_R[i];
            pth_R[i] = p[i];
            ptm[i] = pth_R[i];
            ptm_pre[i] = pth_R_pre[i];
        }
    }
    else if (isRL == 1)
    {
        for (int i = 0; i < 7; i++)
        {
            pth_L_pre[i] = pth_L[i];
            pth_L[i] = p[i];
            ptm[i] = pth_L[i];
            ptm_pre[i] = pth_L_pre[i];
        }
    }
    else {}

    if (insure(p, isRL))
    {
        for (int i = 0; i < 7; i++)
        {
            ptm_read[i] = sm.ReadPos(num + i);
        }


        for (int i = 0; i < 7; i++)
        {
            //v = 4 * abs(ptm[i] - ptm_pre[i]); if (v > 1000) v = 1000;           //两帧目标位置差
            v = 4 * abs(ptm[i] - ptm_read[i]); if (v > 1000) v = 1000;          //目标位置与当前位置差
            if (i == 5) v = 2 * v;
            a = 0.1 * v; if (a < 1) a = 1;

            if (v <= 10) p[i] = ptm_pre[i] + 1;

            sm.WritePosEx(num + i, p[i], v, a);
        }
    }
    else   cout << "动作自然一点~";
}

void Driver::drive(const CTRL_DATA* data, SMSBL& sm)
{
    for (int i = 0; i < 3; i++)
    {
        pos_R[i] = data->R.Position[i];
        ang_R[i] = data->R.Posture[i];
        pos_L[i] = data->L.Position[i];
        ang_L[i] = data->L.Posture[i];
    }

    clc(pos_R, ang_R, 0);
    clc(pos_L, ang_L, 1);

    wr(th_R, 0, sm);
    wr(th_L, 1, sm);
}

//ini1 - 初始化 · 抬起手
void Driver::ini1(SMSBL& sm)
{
    int num = 1;

    for (int i = 0; i < 7; i++)
    {
        switch (i)
        {
        case 5:
            sm.WritePosEx(num + i, pth_R[i], 2 * v, 2 * a);
            break;
        default:
            sm.WritePosEx(num + i, pth_R[i], v, a);
            break;
        }
    }

    num = 11;

    for (int i = 0; i < 7; i++)
    {
        switch (i)
        {
        case 5:
            sm.WritePosEx(num + i, pth_L[i], 2 * v, 2 * a);
            break;
        default:
            sm.WritePosEx(num + i, pth_L[i], v, a);
            break;
        }
    }
}