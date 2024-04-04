class Kalman
{   
    private:
        float KalmanState;
        float KalmanUncertainty;
        float KalmanInputRate;
        float KalmanMeasurement;
        float KalmanControlMatrix;
    public:
    Kalman(float KalmanState,float KalmanUncertainty,float KalmanControlMatrix=0.04)
    {
        KalmanState = KalmanState;
        KalmanUncertainty = KalmanUncertainty;
        KalmanControlMatrix = KalmanControlMatrix;
    }
    double ComputeKalman(float KalmanInputRate,float KalmanMeasurement,float KalmanControlMatrix){
        KalmanState = KalmanState + (KalmanControlMatrix * KalmanInputRate);
        KalmanUncertainty = KalmanUncertainty + 0.04*0.04*4*4;
        float KalmanGain = KalmanUncertainty/(KalmanUncertainty + 3*3);
        KalmanState = KalmanState + KalmanGain*(KalmanMeasurement - KalmanState);
        KalmanUncertainty = (1-KalmanGain)*KalmanUncertainty;
        return KalmanState;
    }
};
