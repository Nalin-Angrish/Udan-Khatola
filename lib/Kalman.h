class Kalman
{   
    private:
        float KalmanState;
        float KalmanUncertainty;
        float KalmanInputRate;
        float KalmanMeasurement;
        float KalmanControlMatrix;
        float KalmanInputRateError;
        float KalmanMeasurementError;
    public:
    Kalman(float KalmanState,float KalmanUncertainty, float KalmanInputRateError, float KalmanMeasurementError,float KalmanControlMatrix=0.04)
    {
        KalmanState = KalmanState;
        KalmanUncertainty = KalmanUncertainty;
        KalmanControlMatrix = KalmanControlMatrix;
        KalmanInputRateError = KalmanInputRateError;
        KalmanMeasurementError = KalmanMeasurementError;
    }
    void UpdateKalman(float KalmanMeasurement,float KalmanInputRate){
        KalmanMeasurement = KalmanMeasurement;
        KalmanInputRate = KalmanInputRate;
    }
    float ReturnKalman(){
        return KalmanState;
    }
    void ComputeKalman(float KalmanState,float KalmanUncertainty,float KalmanInputRate,float KalmanMeasurement,float KalmanControlMatrix,float KalmanInputRateError,float KalmanMeasurementError){
        KalmanState = KalmanState + (KalmanControlMatrix * KalmanInputRate);
        KalmanUncertainty = KalmanUncertainty + KalmanControlMatrix*KalmanControlMatrix*KalmanInputRateError*KalmanInputRateError;
        float KalmanGain = KalmanUncertainty/(KalmanUncertainty + KalmanMeasurementError*KalmanMeasurementError);
        KalmanState = KalmanState + KalmanGain*(KalmanMeasurement - KalmanState);
        KalmanUncertainty = (1-KalmanGain)*KalmanUncertainty;
    }
};
