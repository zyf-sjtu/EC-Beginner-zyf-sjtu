#include "main.h"
#include "PID.h"

#include "stm32f4xx_hal_can.h"



class Motor
{
public:
    enum Motortype
    {
        M3508,
        M2006
    };
    enum ControlMode
    {
        POSITION_SPEED,
        SPEED,
        TORQUE
    };

    explicit Motor(Motortype motortype,float ratio,ControlMode mode,PID ppid,PID spid);
    void canRxMsgCallback(const uint8_t rxdata[8]);

    float angle() const;
    float speedRpm() const;
    float currentAmps() const;
    //float temperatureC() const;
    bool hasFeedback() const;



    void setTxIntensity(uint8_t motor_id,float angle_ref);
    uint8_t* getTxData();
private:

    Motortype motor_type_;
    ControlMode control_mode_;
    PID ppid_;
    PID spid_;
    const float ratio_;
    volatile uint16_t ecd_angle_=0;
    volatile int16_t speedRpm_=0;
    volatile int16_t currentA_=0;
    volatile uint16_t last_ecd_=0;
    volatile int32_t total_ecd_=0;
    volatile float speed_=0;
    int16_t delta_=0;
    //float tempC_=0;
    volatile float angle_=0;
    bool received_=false;
    volatile float intensity_float_=0;
    uint8_t tx_data_[8];
    static constexpr uint16_t kEncoderRange=8192;
    bool stop_flag_=false;
    float feedforward_speed_=0;
    float feedforward_intensity_=10;
    float kp_feedforward_intensity_=0;
    float kp_feedforward_speed_=0;
    void handleIntensity(float angle_ref);
    void RxDecode(const uint8_t* rx_data_);


};
