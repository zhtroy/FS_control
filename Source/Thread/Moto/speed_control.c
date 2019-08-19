#include "speed_control.h"

#define TYPE_ACCEL_SLOW     (0)
#define TYPE_ACCEL_MEDIUM   (1)
#define TYPE_ACCEL_FAST     (2)
#define TYPE_ACCEL_JUMP     (3)

/* 最大减速度，单位(RPM/SECOND) */
#define ACCEL_MAX_RPM       (320)
/* 平均加减速度，单位(RPM/SECOND) */
#define ACCEL_AVARAGE_RPM   (80)

#define SPEED_CONTROL_TT

#ifdef SPEED_CONTROL_TT
/*****************************************************************************
 * 函数名称: int32_t SpeedGenerate(int32_t vc, int32_t ve)
 * 函数说明: 速度曲线拟合
 * 输入参数:
 *          vc: 当前速度
 *          ve: 期望速度
 * 输出参数: 无
 * 返 回 值: vg-目标速度
 * 备注:
 *    vc: Current Speed.
 *    ve: Expected Speed.
 *    vi: Initial Speed.
 *    vg: Goal Speed.
 *    te: Expected Delta Time.
 *    td: Delta Time.
*****************************************************************************/
int32_t SpeedGenerate(int32_t vc, int32_t ve)
{
    static int32_t lastExpSpeed = 0;
    static uint8_t type = TYPE_ACCEL_JUMP;
    static int32_t a0 = 0;
    static int32_t a1 = 0;
    static int32_t aMax = 0;
    static int32_t vi = 0;
    static int32_t deltaSpeed = 0;
    static int32_t te = 0;

    int32_t vg = 0;
    int64_t temp = 0;
    static int32_t td = 0;
    static int32_t t0 = 0;
    static int32_t t0t1 = 0;

    /*
     * 期望速度(expSpeed)或者期望时间(expDeltaTime)变化时，
     * a)重新计算加速参数
     * b)记录当前时刻
     * c)记录期望速度和期望时间
     */
    //if((ve != lastExpSpeed) || (te != lastExpDeltaTime))
    if((ve != lastExpSpeed))
    {
        /*根据平均加速度计算期望时间*/
        deltaSpeed = ve - vc;
        te = abs(deltaSpeed*1000/ACCEL_AVARAGE_RPM);

        /*根据te的范围以及最大加速度计算曲线类型*/
        if(te == 0)
        {
            type = TYPE_ACCEL_JUMP;
            //vg = ve;
        }
        else
        {

            a0 = deltaSpeed*2000/te;
            a1 = a0/2;
            if(deltaSpeed >0)
                aMax = ACCEL_MAX_RPM;
            else
                aMax = -ACCEL_MAX_RPM;

            if(abs(a0) <= ACCEL_MAX_RPM)
            {
                type = TYPE_ACCEL_SLOW;
            }
            else if(abs(a1) <= ACCEL_MAX_RPM)
            {
                t0t1 = abs(deltaSpeed)*1000/ACCEL_MAX_RPM;
                t0 = te - t0t1;
                type = TYPE_ACCEL_MEDIUM;
            }
            else
            {
                type = TYPE_ACCEL_FAST;
            }

        }/*if(expDeltaTime == 0)->else*/

        lastExpSpeed = ve;
        vi = vc;
        td = 0;
    }/*if((expSpeed !=....*/

    /*计算vg*/
    {
        //userGetMS(&curTimeMs);
        //td = curTimeMs - timeMs;
        /*
         * 计算Tap：50ms
         */
        td += 50;

        if(type == TYPE_ACCEL_SLOW)
        {
            /*
             * 平缓对称S曲线
             */
            if(td > te)
            {
                vg = ve;
            }
            else if(td > (te/2))
            {
                temp = te - td;
                vg = ve - temp*temp*a0/te/1000;
            }
            else
            {
                temp = td;
                vg = vi + temp*temp*a0/te/1000;
            }
        }/*if(type == TYPE_ACCEL_SLOW)*/
        else if(type == TYPE_ACCEL_MEDIUM)
        {
            /*
             * 陡峭对称S曲线
             */

            //t1 = expDeltaTime - 2*t0;
            if(td > te)
            {
                vg = ve;
            }
            else if(td > t0t1)
            {
                temp = te - td;
                vg = ve - temp*temp*aMax/t0/2/1000;
            }
            else if(td > t0)
            {
                temp = te - td - t0/2;
                vg = ve - temp*aMax/1000;
            }
            else
            {
                temp = td;
                vg = vi + temp*temp*aMax/t0/2/1000;
            }
        }/*else if(type == TYPE_ACCEL_MEDIUM)*/
        else if(type == TYPE_ACCEL_FAST)
        {
            /*
             * 固定加速度曲线
             */
            if(td > te)
            {
                vg = ve;
            }
            else
            {
                vg = vi + a1*td/1000;
            }
        }/*else if(type == TYPE_ACCEL_FAST)*/
        else
        {
            vg = ve;
        }
    }
    return vg;
}
#endif

