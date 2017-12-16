#include "mbed.h"
#include "C12832.h"
#include "OdinWiFiInterface.h"
#include "http_request.h"
#include "Sht31.h"
#include "CCS811.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

#define USE_I2C_2V8

// GLOBAL VARIABLES HERE
OdinWiFiInterface wifi;
Sht31   temp_sensor(PF_0, PF_1);
CCS811  air_sensor(PF_0, PF_1);
C12832  lcd(PE_14, PE_12, PD_12, PD_11, PE_9);
InterruptIn put_button(PF_2);
volatile bool put_clicked = false;
double Environment_array[4]={};
int type;
int par=0;
int i=0;
// FUNCTION DEFINITIONS HERE

//lcd_print
void lcd_print(const char* message) {
    lcd.cls();
    lcd.locate(0, 3);
    lcd.printf(message);
}
//activate the button
void send_get_put() {
    put_clicked = true;
}

// temperature and humidity sensor
std::pair<double, double> read_temp() {
    double t = temp_sensor.readTemperature();
    double h = temp_sensor.readHumidity();
    return std::make_pair(t, h);
}

// air quality sensor
std::pair<std::uint16_t, std::uint16_t> read_air() {
    air_sensor.init();
    std::uint16_t eco2, tvoc;
    air_sensor.readData(&eco2, &tvoc);
    return std::make_pair(eco2, tvoc);
}
 //Laser Sensor
VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;
    
    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return Status;
}


int main() {

    // MAIN CODE HERE

// connect the mircroprocessor to the wifi    
lcd_print("Connecting...");
int ret = wifi.connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
if (ret != 0) {
    lcd_print("Connection error.");
    return -1;
}
lcd_print("Successfully connected!");
 //Setup laser
    int var=1, measure=0;
    int ave=0, sum=0;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData    = &RangingMeasurementData;
    
    // Initialize Comms laster
    pMyDevice->I2cDevAddr      = 0x52;
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  400;
    
    
    VL53L0X_RdWord(&MyDevice, VL53L0X_REG_OSC_CALIBRATE_VAL,0);
    VL53L0X_DataInit(&MyDevice); 
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    
    VL53L0X_StaticInit(pMyDevice); 
    VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads); // Device Initialization
    VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal); // Device Initialization
    VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
    VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536)); //High Accuracy mode, see API PDF
    VL53L0X_SetLimitCheckValue(pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18*65536)); //High Accuracy mode, see API PDF
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 200000); //High Accuracy mode, see API PDF
    VL53L0X_StartMeasurement(pMyDevice);
 // OPERATIONS    
    while(1) {
       // send the data containing the temperature, humidity, and air quality to the HTTP pyhton server
        NetworkInterface* net = &wifi;
        HttpRequest* request = new HttpRequest(net, HTTP_PUT, "http://10.248.138.136:8080/");
        request->set_header("Content-Type", "application/json");
        char body[64];
        snprintf(body, 64, "%f %f %d %d", read_temp().first,read_temp().second,read_air().first,read_air().second);
        HttpResponse* response = request->send(body, strlen(body));
        lcd_print(response->get_body_as_string().c_str());
        delete request;
        
       
        lcd.cls();
        lcd.locate(0,1);
        lcd.printf("[DISTANCE]");
        // the output data from the sensor will contain an average of values
            while(var<=10){
                WaitMeasurementDataReady(pMyDevice);
                VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);
                measure=pRangingMeasurementData->RangeMilliMeter;
                sum=sum+measure;
                VL53L0X_ClearInterruptMask(pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
                VL53L0X_PollingDelay(pMyDevice);
                var++;
                }
        ave=sum/var;
        // introduce a sensity parameter
        if(par==0) {
            par = ave;
            }
        // every time the sensor measures a value and a value of 1 is sent to the HTTP server which is equivalent to an object passing by
        if((par - ave) >1000){
            lcd.locate(0,15);
            lcd.printf("%Person passed by"); // Print to LCD values
            int i=1;
            NetworkInterface* net = &wifi;
            HttpRequest* request = new HttpRequest(net, HTTP_PUT, "http://10.248.138.136:8080/");
            request->set_header("Content-Type", "application/json");
            char body[64];
            snprintf(body, 64, "%d",i );
            HttpResponse* response = request->send(body, strlen(body));
            lcd_print(response->get_body_as_string().c_str());
            delete request; 
            }
        var=1;
        sum=0; 
        lcd.locate(0,15);
        lcd.printf("%dmm", ave); // Print to LCD values
        //wait(1);
    }
}