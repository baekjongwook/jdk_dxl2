#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#define division 1000 // Don't touch

typedef struct _PID
{
    double input;
    double target;

    double E;
    double E_old;

    double sumE;
    double sumELimit;
    
    double diffE;

    double kP;
    double kI;
    double kD;

    double output;
    double outputLimit;
} PID;

void PID_Init(PID *dst, double kP, double kI, double kD, double sumELimit, double outputLimit)
{
    dst->kP = kP;
    dst->kI = kI;
    dst->kD = kD;
    dst->sumELimit = sumELimit;
    dst->outputLimit = outputLimit;
}

void PID_Control(PID *dst, double target, double input)
{
    dst->input = input;
    dst->target = target;

    dst->E = dst->target - dst->input;
    dst->sumE += dst->E;
    dst->diffE = dst->E - dst->E_old;

    if (dst->sumE > dst->sumELimit)
    {
        dst->sumE = dst->sumELimit;
    }
    else if (dst->sumE < -dst->sumELimit)
    {
        dst->sumE = -dst->sumELimit;
    }

    dst->output = (dst->kP * dst->E + dst->kI * dst->sumE + dst->kD * dst->diffE) / division;

    dst->E_old = dst->E;

    if (dst->output > dst->outputLimit)
    {
        dst->output = dst->outputLimit;
    }
    else if (dst->output < -dst->outputLimit)
    {
        dst->output = -dst->outputLimit;
    }
}

#endif // PID_CONTROL_H