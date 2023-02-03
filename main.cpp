#include "mbed.h"
#include "math_ops.h"
#include <cstring>
#include "leg_message.h"

// length of receive/transmit buffers
#define RX_LEN 66
#define TX_LEN 66

// length of outgoing/incoming messages
#define DATA_LEN 30
#define CMD_LEN  66

// Master CAN ID ///
#define CAN_ID 0x0


/// Value Limits ///
 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -65.0f
 #define V_MAX 65.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f
 
 /// Joint Soft Stops ///
 #define A_LIM_P 1.5f
 #define A_LIM_N -1.5f
 #define H_LIM_P 5.0f
 #define H_LIM_N -5.0f
 #define K_LIM_P 0.2f
 #define K_LIM_N 7.7f
 #define KP_SOFTSTOP 100.0f
 #define KD_SOFTSTOP 0.4f;

#define ENABLE_CMD 0xFFFF
#define DISABLE_CMD 0x1F1F

#define KP 0.0f
#define KD 0.0f

//Extra belt 28:18 belt reduction on the knees;
#define gr_knee 0.643f;

/// TELEOP VALUES ///
float kp_q = 100.0f; //175.0f;
float kd_q = 0.75f; //1.0f; 
float scaling = 0.25;

// gravity compensation at the hip
float g = 9.81;
float m_up = 0.13f;
float m_low = 0.06f;

//grav comp and friction comp terms
float tau_g1;//Leg 1 gravity torques
float frict_1;

//gravity compensation
#define L1 0.0577f
#define L2 0.2088f
#define L3 0.175f
#define b 0.0f
#define gcomp_quadrant_offset 1.571f

//offsets for the joint coordinate space
const float offset[3] = {0.0f, 3.493f, -2.766f};


DigitalOut led(PC_5);
AnalogIn pot(PA_4);


RawSerial       pc(PA_2, PA_3, 921600);
CAN          can1(PB_12, PB_13, 1000000);  // CAN Rx pin name, CAN Tx pin name

CANMessage   rxMsg1, rxMsg2;
CANMessage   txMsg1, txMsg2;
CANMessage   a1_can, h1_can, k1_can;    //TX Messages
int                     ledState;
Ticker                  sendCAN;
int                     counter = 0;
int                     counter2;
volatile bool           msgAvailable = false;
Ticker loop;

DigitalIn estop(PB_15);

leg_state l1_state;
leg_control l1_control;

uint16_t x = 0;
uint16_t count = 0;

int control_mode = 1;
int is_standing = 0;
int enabled = 0;

 // generates fake spi data from spi command
void teleop();
void zero_all();
void enable();


/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]

void pack_cmd(CANMessage * msg, joint_control joint){
     
     /// limit data to be within bounds ///
     float p_des = fminf(fmaxf(P_MIN, joint.p_des), P_MAX);                    
     float v_des = fminf(fmaxf(V_MIN, joint.v_des), V_MAX);
     float kp = fminf(fmaxf(KP_MIN, joint.kp), KP_MAX);
     float kd = fminf(fmaxf(KD_MIN, joint.kd), KD_MAX);
     float t_ff = fminf(fmaxf(T_MIN, joint.t_ff), T_MAX);
     /// convert floats to unsigned ints ///
     uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);            
     uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
     uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
     uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
     uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
     /// pack ints into the can buffer ///
     msg->data[0] = p_int>>8;                                       
     msg->data[1] = p_int&0xFF;
     msg->data[2] = v_int>>4;
     msg->data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
     msg->data[4] = kp_int&0xFF;
     msg->data[5] = kd_int>>4;
     msg->data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
     msg->data[7] = t_int&0xff;
     }
     
/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]] 
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]

void unpack_reply(CANMessage msg, leg_state * leg){
    /// unpack ints from can buffer ///
    uint16_t id = msg.data[0];
    uint16_t p_int = (msg.data[1]<<8)|msg.data[2];
    uint16_t v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
    uint16_t i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
    /// convert uints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    
    if(id==1){
        leg->a.p = p;
        leg->a.v = v;
        leg->a.t = t;
        }
    else if(id==2){
        leg->h.p = p+offset[1];
        leg->h.v = v;
        leg->h.t = t;
        }
    //add in the GEAR RATIO SCALING HERE
    else if(id==3){
        leg->k.p = p*gr_knee + offset[2];
        leg->k.v = v*gr_knee;
        leg->k.t = t*gr_knee;
        }
    } 

 void rxISR1() {
    can1.read(rxMsg1);                    // read message into Rx message storage
    unpack_reply(rxMsg1, &l1_state);
}

void PackAll(){
    pack_cmd(&a1_can, l1_control.a); 
    pack_cmd(&h1_can, l1_control.h); 
    pack_cmd(&k1_can, l1_control.k); 
    }

void WriteAll(){
    //toggle = 1;
    can1.write(a1_can);
    wait(.00002);
    can1.write(h1_can);
    wait(.00002);
    can1.write(k1_can);
    wait(.00002);
    //toggle = 0;
    }

void sendCMD(){
    counter ++;

    PackAll();

    if(counter>100){
        // printf("%.3f %.3f %.3f  \n\r", l1_state.a.p, l1_state.h.p, l1_state.k.p);
        printf("%.3f %.3f %.3f  \n\r", l1_state.a.p, l1_state.h.p, l1_state.k.p);
        counter = 0 ;
        }
    
    WriteAll();
    
    }



    
void Zero(CANMessage * msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFE;
    WriteAll();
    }

void EnterMotorMode(CANMessage * msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFC;
    //WriteAll();
    }
    
void ExitMotorMode(CANMessage * msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFD;
    //WriteAll();
    }
void serial_isr(){
     /// handle keyboard commands from the serial terminal ///
     while(pc.readable()){
        char c = pc.getc();
        //led = !led;
        switch(c){
            case(27):
                //loop.detach();
                printf("\n\r exiting motor mode \n\r");
                ExitMotorMode(&a1_can);
                ExitMotorMode(&h1_can);
                ExitMotorMode(&k1_can);
                enabled = 0;
                break;
            case('m'):
                printf("\n\r entering motor mode \n\r");
                EnterMotorMode(&a1_can);
                EnterMotorMode(&h1_can);
                EnterMotorMode(&k1_can);
                wait(.5);
                enabled = 1;
                //loop.attach(&sendCMD, .001);
                break;
            case('s'):
                printf("\n\r standing \n\r");
                counter2 = 0;
                is_standing = 1;
                //stand();
                break;
            case('z'):
                printf("\n\r zeroing \n\r");
                Zero(&a1_can);
                Zero(&h1_can);
                Zero(&k1_can);
                break;
            }
        }
        WriteAll();
        
    }
    
uint32_t xor_checksum(uint32_t* data, size_t len)
{
    uint32_t t = 0;
    for(int i = 0; i < len; i++)   
        t = t ^ data[i];
    return t;
}

int softstop_joint(joint_state state, joint_control * control, float limit_p, float limit_n){
    if((state.p)>=limit_p){
        //control->p_des = limit_p;
        control->v_des = 0.0f;
        control->kp = 0;
        control->kd = KD_SOFTSTOP;
        control->t_ff += KP_SOFTSTOP*(limit_p - state.p);
        return 1;
    }
    else if((state.p)<=limit_n){
        //control->p_des = limit_n;
        control->v_des = 0.0f;
        control->kp = 0;
        control->kd = KD_SOFTSTOP;
        control->t_ff += KP_SOFTSTOP*(limit_n - state.p);
        return 1;
    }
    return 0;
    
    }
    
void zero_all() {
    pc.printf("\n\r Zeroing All Motors...\n\r");
    Zero(&a1_can);
    Zero(&h1_can);
    Zero(&k1_can);
}

void teleop() {
    //can add a +bv at the knee to compensate for friction as well
    frict_1 = b*l1_state.k.v;

    // add gravity compensation terms (compensate entire leg with hip)
    tau_g1 = (L2*0.5f*cosf(l1_state.h.p)*g*m_up) + ((L2*cosf(l1_state.h.p) + L3*0.5f*cosf(l1_state.h.p+l1_state.k.p))*g*m_low);

    l1_control.a.kp = KP;
    l1_control.a.kd = KD;
    // l1_control.a.t_ff = scaling*(kp_q*(l2_state.a.p - l1_state.a.p) + kd_q*(l2_state.a.v - l1_state.a.v));
    l1_control.a.t_ff = 0;

    l1_control.h.kp = KP;
    l1_control.h.kd = KD;
    // l1_control.h.t_ff = scaling*(kp_q*(l2_state.h.p - l1_state.h.p) + kd_q*(l2_state.h.v - l1_state.h.v)) + tau_g1;
    l1_control.h.t_ff = tau_g1;

    l1_control.k.kp = KP;
    l1_control.k.kd = KD;
    // l1_control.k.t_ff = scaling*(kp_q*(l2_state.k.p - l1_state.k.p) + kd_q*(l2_state.k.v - l1_state.k.v)) + frict_1;
    l1_control.k.t_ff = 0;

    PackAll();
    WriteAll();    
}

void enable() {
    printf("\n\r entering motor mode \n\r");
    EnterMotorMode(&a1_can);
    EnterMotorMode(&h1_can);
    EnterMotorMode(&k1_can);
    WriteAll();
    wait(.5);
    enabled = 1;
}

void print_pos() {
    if(counter>200){
        printf("AbAd: %.5f, %3.5f / Hip: %.5f, %3.5f / Knee: %.5f, %3.5f \n\r", (l1_state.a.p/PI)*180, l1_state.a.t, (l1_state.h.p/PI)*180, l1_state.h.t, (l1_state.k.p/PI)*180, l1_state.k.t);
        counter = 0 ;
    }
}
    
int main() {
    wait(1);
    //led = 1;
    pc.attach(&serial_isr);
    estop.mode(PullUp);
    //spi.format(16, 0);
    //spi.frequency(1000000);
    //spi.reply(0x0);
    //cs.fall(&spi_isr);

    // can1.frequency(1000000);                     // set bit rate to 1Mbps
    // can1.attach(&rxISR1);                 // attach 'CAN receive-complete' interrupt handler
    can1.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0); //set up can filter
    //can2.frequency(1000000);                     // set bit rate to 1Mbps
    //can2.attach(&rxISR2);                 // attach 'CAN receive-complete' interrupt handler
    // can2.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0); //set up can filter
    
    
    NVIC_SetPriority(TIM5_IRQn, 1);
    NVIC_SetPriority(CAN1_RX0_IRQn, 3);
    //NVIC_SetPriority(CAN2_RX0_IRQn, 3);
    
    printf("\n\r Telop System\n\r");
    //printf("%d\n\r", RX_ID << 18);
    
    a1_can.len = 8;                         //transmit 8 bytes
    h1_can.len = 8;
    k1_can.len = 8;
    rxMsg1.len = 6;                          //receive 6 bytes

    a1_can.id = 0x1;                        
    h1_can.id = 0x2;
    k1_can.id = 0x3;

    pack_cmd(&a1_can, l1_control.a); 
    pack_cmd(&h1_can, l1_control.h); 
    pack_cmd(&k1_can, l1_control.k); 
    WriteAll();

    zero_all();

    enable();
            
    while(1) {
        counter++;
        can1.read(rxMsg1);                    // read message into Rx message storage
        unpack_reply(rxMsg1, &l1_state);
        wait_us(10);

        //read the potentiometer
        // scaling = fmaxf(fminf(pot.read(), 0.75), 0.25);

        //run teleop
        teleop(); //run the teleop function

        //debugging print, comment out after debugging is funished
        print_pos();

        }
        

        
        
    }