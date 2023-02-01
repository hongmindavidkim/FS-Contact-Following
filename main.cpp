#include "DigitalOut.h"
#include "ThisThread.h"
#include "mbed.h"
#include "mbed_wait_api.h"
#include "neuralnet.h"
#include "math.h"
#include "FPSensor.h"
#include <cstdint>

#define PI 3.14159265358979

RawSerial pc(PA_2, PA_3, 921600);
uint16_t sense_data_raw[8];
float sense_data_decode[5];

float l1[12]; // to be evaluated on-line
float l2[25];

float in_vec[8];
float out_vec[5];
int out_vec_pr[5];
uint16_t offsets[8];

FPSensor sensor1(PA_7, PA_6, PA_5, PA_4); // for spine
// FPSensor sensor1(D11, D12, D13, D10); //for nucleo
Timer t;

void calibrateSensor(uint16_t* offsets){
    
    float temp_offsets[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int samps = 10;
    
    for (int i=0; i<samps; i++){
        for (int j=0; j<8; j++){
            temp_offsets[j] += (float)sensor1.binary(j);
        }
        wait_us(100000);
        printf("getting offsets\r\n");
    }
    
    for (int i=0; i<8; i++){
        temp_offsets[i] = temp_offsets[i]/((float)samps); // get overall offset
        offsets[i] = (uint16_t)temp_offsets[i]; // convert to int
    }

}
void readSensor(uint16_t* data){
    // read 8 sensor values over SPI
    
    for (int i=0; i<8; i++){
        data[i] = sensor1.binary(i);
    }
    
}
    
void decodeSensor(float input[8], float* output) {
    // decode sensor data here....521*4 operations (multiply,add,activation,add)
    // reset values
    for (int i = 0; i<12; i++){
        l1[i] = 0.0f;
    }
    for (int i = 0; i<25; i++){
        l2[i] = 0.0f;
    }
    for (int i = 0; i<5; i++){
        output[i] = 0.0f;
    }
        
    // layer 1
    for(int i = 0; i<12; i++){ // for each node in the next layer
        for(int j = 0; j<8; j++){ // add contribution of node in prev. layer
            l1[i] +=  (w1[j][i]*input[j]); 
        }
        l1[i] += b1[i]; // add bias
        l1[i] = fmaxf(0.0f, l1[i]); // relu activation
    }
        
    // layer 2
    for(int i = 0; i<25; i++){ // for each node in the next layer
        for(int j = 0; j<12; j++){ // add contribution of node in prev. layer
            l2[i] += (w2[j][i]*l1[j]);
        }
        l2[i] += b2[i]; // add bias
        if (l2[i]<0.0f) { // elu activation
            l2[i] = exp(l2[i]) - 1.0f; // alpha implicitly set to 1.0 here
        }
    }   
    
    // layer 3
    for(int i = 0; i<5; i++){ // for each node in the next layer
        for(int j = 0; j<25; j++){ // add contribution of node in prev. layer
            output[i] += w3[j][i]*l2[j];
            
        }
        output[i] += b3[i];// add bias
        output[i] = 1.0f/(1.0f + exp(-output[i])); // sigmoid activation 
    }  
    
}


int main()
{
    printf("Calibrating Sensor....\r\n");
    calibrateSensor(offsets);
    int looptime;
    t.start();
    while (true) {
        t.reset();
        readSensor(sense_data_raw);
        // printf("%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d\r\n",sense_data_raw[0],sense_data_raw[1],sense_data_raw[2],sense_data_raw[3],sense_data_raw[4],sense_data_raw[5],sense_data_raw[6],sense_data_raw[6]);
        for (int i=0; i<8; i++) {
        in_vec[i] = 0.0f;
        in_vec[i] = (((float)(sense_data_raw[i]-offsets[i])) + abs(minims[i+5])) / maxS;
        }

        decodeSensor(in_vec, sense_data_decode);
        
        for (int i=0; i<5; i++) {
        out_vec[i] = 0.0f;
        out_vec[i] = (sense_data_decode[i]*maxims[i]) - abs(minims[i]);
        out_vec_pr[i] = (int)(out_vec[i]*100.0f);
        }
        // looptime = 0;
        looptime = t.read_us();    
        printf("%+09.4f,%+09.4f,%+09.4f,%+09.4f,%+09.4f,%3d\r\n",out_vec[0],out_vec[1],out_vec[2],(out_vec[3]/PI)*180,(out_vec[4]/PI)*180,looptime);
    }
}




