//Pulled (and modified) from
//http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/


class PID {
    private:
        unsigned long lastTime;
        double input, output, target;
        double iTerm, lastInput;
        double kp, ki, kd;
        int sampleTime;
        double outMin, outMax;
        bool inAuto; 
         
        #define MANUAL 0
        #define AUTOMATIC 1

        void initialize()
        {
           lastInput = input;
           iTerm = output;
           if(iTerm > outMax) iTerm = outMax;
           else if(iTerm < outMin) iTerm = outMin;
        }

    public:
        PID(double _target, double _kp, double _ki, double _kd, 
            double _outMin, double _outMax, int _sampleTimeMillis) {
            target = _target;
            kp = _kp;
            ki = _ki;
            kd = _kd;
            outMin = _outMin;
            outMax = _outMax;
            sampleTime = _sampleTimeMillis;
            inAuto = true;
            iTerm = 0;
            lastInput = 0;
        }

        double compute(double _input)
        {
           if(!inAuto) return -1;
           input = _input;
           unsigned long now = millis();
           int timeChange = (now - lastTime);
           if(timeChange>=sampleTime)
           {
              /*Compute all the working error variables*/
              double error = target - input;
              iTerm += (ki * error);
              if(iTerm > outMax) iTerm = outMax;
              else if(iTerm < outMin) iTerm = outMin;
              double dInput = (input - lastInput);
         
              /*Compute PID Output*/
              output = kp * error + iTerm - kd * dInput;
              if(output > outMax) output = outMax;
              else if(output < outMin) output = outMin;
         
              /*Remember some variables for next time*/
              lastInput = input;
              lastTime = now;
           }
           return outMin+outMax-output;
        }
         
        void setTunings(double _kp, double _ki, double _kd)
        {
          double sampleTimeInSec = ((double)sampleTime)/1000;
           kp = _kp;
           ki = _ki * sampleTimeInSec;
           kd = _kd / sampleTimeInSec;
        }
         
        void setSampleTime(int _sampleTime)
        {
           if (_sampleTime > 0)
           {
              double ratio  = (double)_sampleTime
                              / (double)sampleTime;
              ki *= ratio;
              kd /= ratio;
              sampleTime = (unsigned long)_sampleTime;
           }
        }
         
        void setOutputLimits(double mn, double mx)
        {
           if(mn > mx) return;
           outMin = mn;
           outMax = mx;
            
           if(output > outMax) output = outMax;
           else if(output < outMin) output = outMin;
         
           if(iTerm > outMax) iTerm = outMax;
           else if(iTerm < outMin) iTerm = outMin;
        }

         
        void setMode(int _mode)
        {
            bool newAuto = (_mode == AUTOMATIC);
            if(newAuto && !inAuto)
            {  /*we just went from manual to auto*/
                initialize();
            }
            inAuto = newAuto;
        }

        void setTarget(double _target){
            target = _target;
        }

};
