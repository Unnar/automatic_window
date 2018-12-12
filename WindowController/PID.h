//Pulled (and modified) from
//http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/


class PID {
    private:
        unsigned long lastTime;
        double Input, Output, Setpoint;
        double ITerm, lastInput;
        double kp, ki, kd;
        int SampleTime;
        double outMin, outMax;
        bool inAuto; 
         
        #define MANUAL 0
        #define AUTOMATIC 1

        void initialize()
        {
           lastInput = Input;
           ITerm = Output;
           if(ITerm > outMax) ITerm = outMax;
           else if(ITerm < outMin) ITerm = outMin;
        }

    public:
        PID(double _Setpoint, double _kp, double _ki, double _kd, 
            double _outMin, double _outMax, int _SampleTimeMillis) {
            Setpoint = _Setpoint;
            kp = _kp;
            ki = _ki;
            kd = _kd;
            outMin = _outMin;
            outMax = _outMax;
            SampleTime = _SampleTimeMillis;
            inAuto = true;
            ITerm = 0;
            lastInput = 0;
        }

        double compute(double _Input)
        {
           if(!inAuto) return -1;
           Input = _Input;
           unsigned long now = millis();
           int timeChange = (now - lastTime);
           if(timeChange>=SampleTime)
           {
              /*Compute all the working error variables*/
              double error = Setpoint - Input;
              ITerm += (ki * error);
              if(ITerm > outMax) ITerm = outMax;
              else if(ITerm < outMin) ITerm = outMin;
              double dInput = (Input - lastInput);
         
              /*Compute PID Output*/
              Output = kp * error + ITerm - kd * dInput;
              if(Output > outMax) Output = outMax;
              else if(Output < outMin) Output = outMin;
         
              /*Remember some variables for next time*/
              lastInput = Input;
              lastTime = now;
           }
           return outMin+outMax-Output;
        }
         
        void setTunings(double Kp, double Ki, double Kd)
        {
          double SampleTimeInSec = ((double)SampleTime)/1000;
           kp = Kp;
           ki = Ki * SampleTimeInSec;
           kd = Kd / SampleTimeInSec;
        }
         
        void setSampleTime(int NewSampleTime)
        {
           if (NewSampleTime > 0)
           {
              double ratio  = (double)NewSampleTime
                              / (double)SampleTime;
              ki *= ratio;
              kd /= ratio;
              SampleTime = (unsigned long)NewSampleTime;
           }
        }
         
        void setOutputLimits(double Min, double Max)
        {
           if(Min > Max) return;
           outMin = Min;
           outMax = Max;
            
           if(Output > outMax) Output = outMax;
           else if(Output < outMin) Output = outMin;
         
           if(ITerm > outMax) ITerm = outMax;
           else if(ITerm < outMin) ITerm = outMin;
        }

         
        void setMode(int Mode)
        {
            bool newAuto = (Mode == AUTOMATIC);
            if(newAuto && !inAuto)
            {  /*we just went from manual to auto*/
                initialize();
            }
            inAuto = newAuto;
        }

        void setSetpoint(double _Setpoint){
            Setpoint = _Setpoint;
        }

};

