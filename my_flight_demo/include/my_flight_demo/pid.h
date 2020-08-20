#include <math.h>

typedef struct _pid{
float SetPoint;
float ActualPoint;
float OutPutVel;
float err;
float err_next;
float err_last;
float Kp, Ki, Kd;
}Pid;
 
 
class Pid_control
{
public:
void PID_init(float kp,float ki,float kd,float SetPoint,float ActualPoint);
bool PID_realize(float point,float ActualPoint);
int index;
Pid pid;
};


void Pid_control::PID_init(float kp,float ki,float kd,float SetPoint,float ActualPoint)
{
pid.SetPoint = SetPoint;
pid.ActualPoint = ActualPoint;
pid.OutPutVel=0.0;
pid.err = 0.0;
pid.err_last = 0.0;
pid.err_next = 0.0;
pid.Kp = kp;
pid.Ki = ki;
pid.Kd = kd;
}
 
bool Pid_control::PID_realize(float point,float ActualPoint){
pid.SetPoint = point;
pid.ActualPoint=ActualPoint;
pid.err = pid.SetPoint - pid.ActualPoint;
if (abs(pid.err)<0.1)
{
    pid.err_last = pid.err_next;
    pid.err_next = pid.err;
    pid.OutPutVel=0;
    return true;
}
float incrementSpeed = pid.Kp*(pid.err - pid.err_next) + pid.Ki*pid.err + pid.Kd*(pid.err - 2 * pid.err_next + pid.err_last);
pid.OutPutVel += incrementSpeed;
pid.err_last = pid.err_next;
pid.err_next = pid.err;

return false;
}