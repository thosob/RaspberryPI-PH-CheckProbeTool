/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: tsobieroy
 *
 * Created on 2. Mai 2017, 11:42
 */

#include <iostream>
#include <cstdlib>
#include <string>
#include <string.h>
#include <cmath>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>   
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <cstdint>
#include <arpa/inet.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <inttypes.h> 



using namespace std;

#define ADS1015_CONVERSIONDELAY         (1)
#define ADS1115_CONVERSIONDELAY         (8)
#define ADS1015_REG_POINTER_MASK        (0x03)
#define ADS1015_REG_POINTER_CONVERT     (0x00)
#define ADS1015_REG_POINTER_CONFIG      (0x01)
#define ADS1015_REG_POINTER_LOWTHRESH   (0x02)
#define ADS1015_REG_POINTER_HITHRESH    (0x03)
#define ADS1015_REG_CONFIG_OS_MASK      (0x8000)
#define ADS1015_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
#define ADS1015_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
#define ADS1015_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

#define ADS1015_REG_CONFIG_MUX_MASK     (0x7000)
#define ADS1015_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
#define ADS1015_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
#define ADS1015_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
#define ADS1015_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
#define ADS1015_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
#define ADS1015_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
#define ADS1015_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
#define ADS1015_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

#define ADS1015_REG_CONFIG_PGA_MASK     (0x0E00)
#define ADS1015_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
#define ADS1015_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
#define ADS1015_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
#define ADS1015_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
#define ADS1015_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
#define ADS1015_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

#define ADS1015_REG_CONFIG_MODE_MASK    (0x0100)
#define ADS1015_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
#define ADS1015_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

#define ADS1015_REG_CONFIG_DR_MASK      (0x00E0)  
#define ADS1015_REG_CONFIG_DR_128SPS    (0x0000)  // 128 samples per second
#define ADS1015_REG_CONFIG_DR_250SPS    (0x0020)  // 250 samples per second
#define ADS1015_REG_CONFIG_DR_490SPS    (0x0040)  // 490 samples per second
#define ADS1015_REG_CONFIG_DR_920SPS    (0x0060)  // 920 samples per second
#define ADS1015_REG_CONFIG_DR_1600SPS   (0x0080)  // 1600 samples per second (default)
#define ADS1015_REG_CONFIG_DR_2400SPS   (0x00A0)  // 2400 samples per second
#define ADS1015_REG_CONFIG_DR_3300SPS   (0x00C0)  // 3300 samples per second

#define ADS1015_REG_CONFIG_CMODE_MASK   (0x0010)
#define ADS1015_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
#define ADS1015_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

#define ADS1015_REG_CONFIG_CPOL_MASK    (0x0008)
#define ADS1015_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
#define ADS1015_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

#define ADS1015_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
#define ADS1015_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
#define ADS1015_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

#define ADS1015_REG_CONFIG_CQUE_MASK    (0x0003)
#define ADS1015_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
#define ADS1015_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
#define ADS1015_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
#define ADS1015_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)

int readADC_SingleEnded(int fd, int channel) {

    int ADS_address = fd; // Address of our device on the I2C bus
    int I2CFile;

    uint8_t writeBuf[3]; // Buffer to store the 3 bytes that we write to the I2C device
    uint8_t readBuf[2]; // 2 byte buffer to store the data read from the I2C device

    int16_t val; // Stores the 16 bit value of our ADC conversion

    I2CFile = open("/dev/i2c-1", O_RDWR); // Open the I2C device

    ioctl(I2CFile, I2C_SLAVE, ADS_address); // Specify the address of the I2C Slave to communicate with

    // These three bytes are written to the ADS1115 to set the config register and start a conversion 
    // There are 3 registers  one is the config register which is accessed by writing one to the buffer    
    writeBuf[0] = 1; // This sets the pointer register so that the following two bytes write to the config register
    // Modifying adressing part
    // 100 : AIN P = AIN0 and AIN N = GND => Results in 0xC3 11000011 CHANNEL 0
    // 101 : AIN P = AIN1 and AIN N = GND => Results in 0xD3 11010011 CHANNEL 1
    // 110 : AIN P = AIN2 and AIN N = GND => Results in 0xE3 11100011 CHANNEL 2
    // 111 : AIN P = AIN3 and AIN N = GND => Results in 0xF3 11110011 CHANNEL 3
    switch(channel){
        //set up 0
        case 0: writeBuf[1] = 0xC3; break;
        //set up 1
        case 1: writeBuf[1] = 0xD3; break;
        //set up 2
        case 2: writeBuf[1] = 0xE3; break;
        //set up channel 3
        case 3: writeBuf[1] = 0xF3; break;
        //set up default is channel 0
        default: writeBuf[1] = 0xC3; break;
    }
    
    //writeBuf[1] = 0xC3; // This sets the 8 MSBs of the config register (bits 15-8) to 11000011
    writeBuf[2] = 0x03; // This sets the 8 LSBs of the config register (bits 7-0) to 00000011

    // Initialize the buffer used to read data from the ADS1115 to 0
    readBuf[0] = 0;
    readBuf[1] = 0;

    // Write writeBuf to the ADS1115, the 3 specifies the number of bytes we are writing,
    // this begins a single conversion
    write(I2CFile, writeBuf, 3);

    // Wait for the conversion to complete, this requires bit 15 to change from 0->1
    while ((readBuf[0] & 0x80) == 0) // readBuf[0] contains 8 MSBs of config register, AND with 10000000 to select bit 15
    {
        read(I2CFile, readBuf, 2); // Read the config register into readBuf
    }

    writeBuf[0] = 0; // Set pointer register to 0 to read from the conversion register
    write(I2CFile, writeBuf, 1);

    read(I2CFile, readBuf, 2); // Read the contents of the conversion register into readBuf

    val = readBuf[0] << 8 | readBuf[1]; // Combine the two bytes of readBuf into a single 16 bit result 

    printf("Voltage Reading %f (V) \n", (float) val * 4.096 / 32767.0); // Print the result to terminal, first convert from binary value to mV

    close(I2CFile);
    return val;
}

string floatToString(float f) {

    std::ostringstream ss;
    ss << f;
    std::string s(ss.str());
    return s;
}

float get_Probe_mV(int ph_Probe_Address, int i2c_Port) {
    //Loading phAdress first file descriptor, then register
    int raw = readADC_SingleEnded(ph_Probe_Address, i2c_Port);
    raw = raw >> 8 | ((raw << 8) &0xffff);
    //std::cout << raw << endl;
    //3.3 equals the voltage
    //Design Decision: 3.3V implementation   *
    //4096 - 12bit in total 
    if (raw > 0) {
        return (((float) raw / 4096) * 3.3) * 1000;
    } else {
        return -1;
    }
}

/*
 * 
 */
int main(int argc, char** argv) {

    if (argc != 2) {
        std::cout << "Error: You did not provide required arguments." << endl
                << "Usage: CheckProbe PhProbeI2CAddress"
                << "Syntax for port is Port:FD e.g. 77:0"
                << endl
                << "To achieve that simply convert hex to decimal in multiplying 4*16 and add d=13 to it."
                << endl;
        return 0;
    }
    //Initialize variables    
    string str = argv[1];
    int pos = str.find_first_of(':');
    string i2c_FD = str.substr(0, pos);
    string ph_Address = str.substr(pos + 1);
    int ph_Probe_Address;

    uint i2c_Address = stoi(i2c_FD); //0x4d;
    uint i2c_Port = stoi(ph_Address);




    //All temperature files
    //All Ph probes            
    if (i2c_Address >= 0) {
        //Setting Up Gpio
        wiringPiSetupGpio();
        //Access ph-Probe
        ph_Probe_Address = wiringPiI2CSetup(i2c_Address);
        cout << "Getting Value from: " << i2c_Address << ":" << i2c_Port << endl;
        while (true) {
            cout << readADC_SingleEnded(i2c_Address, i2c_Port) << endl;
            sleep(1);
        }

        //return success        
        return 0;
    } else {
        std::cout << -1 << endl;
        //return fail
        return 1;
    }
}

