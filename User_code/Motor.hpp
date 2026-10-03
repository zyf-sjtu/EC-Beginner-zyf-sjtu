#include "main.h"
#include "stm32f4xx_hal_can.h"


class Motor
{
public:
    explicit Motor(float ratio);
    void canRxMsgCallback(const uint8_t rxdata[8]);

    float angle() const;
    float speedRpm() const;
    float currentAmps() const;
    float temperatureC() const;
    bool hasFeedback() const;

    void setTxCurrent(float amperes, uint8_t motor_id);
    uint8_t* getTxData();
private:
    const float ratio_;
    float ecd_angle_=0;
    float speedRpm_=0;
    float currentA_=0;
    float tempC_=0;
    float angle_=0;
    float last_ecd_=0;
    bool received_=false;

    uint8_t tx_data_[8];
    static constexpr uint16_t kEncoderRange=8192;

    void RxDecode(const uint8_t* rx_data_);


};
