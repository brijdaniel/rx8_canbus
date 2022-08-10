/*  CAN Message format:
    <canID, , len, d0, d1, d2, d3, d4, d5, d6, d7>
    canID: 2015 (0x7DF) = request, 2024 (0x7E8) = response to request
    d0 = Query length (typically 2 or 3) 
    d1 = Service (1 = current data, 2 = freeze frame)
    d2 = PID
    d3 = extra bits for PID if required
    d4-d7 = Not used

    Example request for throttle position (PID 17): 
    <2015,2,1,17,0,0,0,0,0>
    Reply: 
    <2024,3,65,17,0,0,0,0,0> = <requested 17, Throttle = 0 to 255, 0,0,0,0>
*/
#include <SPI.h>
#include "mcp_can.h" // https://github.com/yexiaobo-seeedstudio/CAN_BUS_Shield

INT32U canId = 0x000;

long lastRefreshTime = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];

//String BuildMessage="";
int MSGIdentifier=0;

int coolant_temp = 0;
bool fan_status = 0;

// steinhart from here https://www.allaboutcircuits.com/industry-articles/how-to-obtain-the-temperature-value-from-a-thermistor-measurement/
float L1 = log(23700); //lowest temp = -20
float L2 = log(286); //middle temp = 80
float L3 = log(24.8); //highest temp = 179

float Y1 = -1/20;
float Y2 = 1/80;
float Y3 = 1/179;

float G2 = (Y2-Y1)/(L2-L1);
float G3 = (Y3-Y1)/(L3-L1);

float C = (1/(L1+L2+L3))*((G3-G2)/(L3-L2));
float B = G2 - C*(pow(L1, 2) + L1*L2 + pow(L2, 2));
float A = Y1 - L1*(B + pow(L1, 2)*C);

void setup() {
    Serial.begin(38400);
    START_INIT:

    if (CAN_OK == CAN.begin(CAN_500KBPS)) {
        Serial.println("CAN BUS Shield init ok!");
    } else {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
        goto START_INIT;
    }

    // Connect AREF to 3.3V and use that as VCC for reading oil temp and pressure sensors, less noisy
    //analogReference(EXTERNAL);

    // Pin to drive fan relay
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
}

void toggle_fan() {
    digitalWrite(2, !digitalRead(2));
    fan_status = !fan_status;
}

void read_CAN() {

}

void OBD2_request(int mode, int pid) {
    unsigned char msg[7] = {2, mode, pid, 0, 0, 0, 0};
    // <canId, ext, len, msg>
    CAN.sendMsgBuf(0x7DF, 0, 8, msg); 
}

/*
void OBD2_response(String msg) {
    // <canId, len, mode, pid, data, ...>
    // mode = requested mode + 0x40, so mode 1 returns 0x41 = 65
    // E.g. coolant temp response: <2024,3,65,5,56,0,0,0,0,>

}
*/

/*
void print_can_msg(void) {
    Serial.print("<");Serial.print(canId);Serial.print(",");
    for (int i = 0; i<len; i++) { 
        BuildMessage = BuildMessage + buf[i] + ",";
    }
    Serial.print(BuildMessage);
    Serial.println(">");
    BuildMessage="";
}
*/

int oil_temp(void) {
    // which analog pin to connect
    #define THERMISTORPIN A0

    // Measuring across 10kOhm resistor
    float R2 = 10000;

    // Samples to average over
    #define NUMSAMPLES 5

    // take N samples in a row, with a slight dela
    uint8_t i;
    float average;
    int samples[NUMSAMPLES];
    for (i=0; i< NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    delay(10);
    }

    // average all the samples out
    average = 0;
    for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
    }
    average /= NUMSAMPLES;

    // convert ADC samples to volts
    float voltage = average*(5.0/1023.0);
    Serial.print("Oil Temp Voltage: ");
    Serial.println(voltage);

    // convert the value to resistance
    // R1 = R2*(Vin/Vout - 1)
    float R1;
    R1 = R2*(5/voltage -1);
    
    // convert resistance to temperature (degC)
    // use steinhart vals calculated above
    float T = 1 / (A + B*log(R1) + C*pow(log(R1), 3));
    T = T - 273;
    
    
    /*
    float logR1, T;
    float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
    logR1 = log(R1);
    T = (1.0 / (c1 + c2*logR1 + c3*logR1*logR1*logR1));
    T = T - 273.15;

    // or steinhart method
    float steinhart, T;
    steinhart = R1 / R2;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= 3950;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (25 + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;  
    T = steinhart;
    */
    
    Serial.print("Oil Temperature: "); 
    Serial.print(T);
    Serial.print(", Thermistor resistance "); 
    Serial.println(R1);
    return T;
}

int oil_pressure(void) {
    // which analog pin to connect
    #define SENSORPIN A1

    // take N samples in a row, with a slight delay
    uint8_t i;
    float average;
    float pressure;
    int samples[NUMSAMPLES];
    for (i=0; i< NUMSAMPLES; i++) {
    samples[i] = analogRead(SENSORPIN);
    delay(10);
    }

    // average all the samples out
    average = 0;
    for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
    }
    average /= NUMSAMPLES;

    // convert ADC samples to volts
    float voltage = average*(5.0/1023.0);
    
    Serial.print("Oil Pressure Voltage: ");
    Serial.println(voltage);

    // Convert voltage to pressure based on graph provided by Redarc
    pressure = 36*voltage - 18; // This is not perfect but pretty close (~1psi)
    Serial.print("Oil Pressure (psi): ");
    Serial.println(pressure);
    return pressure;
}

void loop() {
    // Request coolant temp (PID 05) every 1s
    // <2015,2,1,5,0,0,0,0,0>
    if(millis() - lastRefreshTime >= 1000) {
        lastRefreshTime += 1000;
        OBD2_request(1, 5);
    }

    // Read available message
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
        CAN.readMsgBuf(&len, buf);
        canId = CAN.getCanId();

        // Response to request for coolant temp
        if (canId==2024) {
            //print_can_msg();
            // If PID = 5 = coolant temp
            if (buf[2]==5) {
                coolant_temp = buf[3] - 40; // OBD2 Coolant temp formula = A-40
                Serial.print("Coolant temp = ");Serial.println(coolant_temp);
                // If temp too high then turn on fans
                if (fan_status==0 && coolant_temp>=90) {
                    toggle_fan();
                } else if (fan_status==1 && coolant_temp<=86) {
                    toggle_fan();
                }
            }
        }

        // Respond to TORQUE requests for oil temp and pressure
        if (canId==2015) {
            //print_can_msg();
            // If request for oil temp (PID 92)
            if (buf[2]==92) {
                unsigned char OilTempRead[7] = {4, 65, 92, oil_temp(), 224, 185, 147}; // <len, mode (65=0x41=1), pid, data, data, data, data>
                CAN.sendMsgBuf(0x7E8, 0, 7, OilTempRead); // canID 0x7E8 = 2024 = response to request
            // If request for oil pressure (PID 205)
            } else if(buf[2]==205) {
                unsigned char OilPressureRead[7] = {4, 65, 205, oil_pressure(), 224, 185, 147};
                CAN.sendMsgBuf(0x7E8, 0, 7, OilPressureRead);
            }
        }
    }
}
