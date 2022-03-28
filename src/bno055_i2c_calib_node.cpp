/* bno055_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a BNO055I2CDriver class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

#include <imu_bno055/bno055_i2c_driver.h>
#include "watchdog/watchdog.h"
#include <csignal>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <memory>

#define CALIB_DATA_SIZE 22

enum CALIB_STATUS {
    NOT_CALIBRATED = 0x00,
    FULL_CALIBRATED = 0x01
};

class BNO055I2CCalibNode {
    public:
        BNO055I2CCalibNode(int argc, char* argv[]);
        void run();

        int  calibrateIMU();
        void readCalibration();
        void writeCalibration();

    private:
        ros::NodeHandle* nh;
        ros::NodeHandle* nh_priv;
        std::unique_ptr<imu_bno055::BNO055I2CDriver> imu;

        std::string param_device;
        int param_address;
        double param_rate;
        std::string param_operation_mode;
        std::string param_calib_file_name;

        diagnostic_msgs::DiagnosticStatus current_status;

        int calibration_counter;
        uint8_t calib_data[CALIB_DATA_SIZE];
};

BNO055I2CCalibNode::BNO055I2CCalibNode(int argc, char* argv[]) {
    ros::init(argc, argv, "bno055_calib_node");
    nh = new ros::NodeHandle();
    nh_priv = new ros::NodeHandle("~");

    if(!nh || !nh_priv) {
        ROS_FATAL("Failed to initialize node handles");
        ros::shutdown();
        return;
    }

    nh_priv->param("device", param_device, (std::string)"/dev/i2c-10");
    nh_priv->param("address", param_address, (int)BNO055_ADDRESS_A);
    nh_priv->param("operation_mode", param_operation_mode, (std::string)"IMU");
    nh_priv->param("calib_file_name", param_calib_file_name, (std::string)"calibration_imu");
 
    imu = std::make_unique<imu_bno055::BNO055I2CDriver>(param_device, param_address);

    imu->init();

    // init calibration counter
    calibration_counter = 0;

    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);

}

void BNO055I2CCalibNode::run() {

    std::cerr << "======= IMU Calibration Starts =======" << std::endl;

    int status;

    // Set mode
    if (param_operation_mode.compare("IMU") == 0)
    {
        imu->setMode(BNO055_OPERATION_MODE_IMUPLUS); 
    } else if (param_operation_mode.compare("NDOF") == 0)
    {
        imu->setMode(BNO055_OPERATION_MODE_NDOF);
    } else
    {
        std::cerr << "Operation Mode NOT SUPPORTED!!" << std::endl;
        return;
    }

    while(ros::ok()) {
        status = calibrateIMU();
        
        if (status == FULL_CALIBRATED)
        {
            // read calibration
            readCalibration();

            // write calibration
            writeCalibration();

            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int  BNO055I2CCalibNode::calibrateIMU()
{
    // calibrate
    int status = NOT_CALIBRATED;

    // Get calibration status
    int response = imu->calibrate();

    int system_calib_status = (response >> 6) & 0x03;
    int gyro_calib_status   = (response >> 4) & 0x03;
    int acce_calib_status   = (response >> 2) & 0x03;
    int mag_calib_status    = (response >> 0) & 0x03;

    // Calibration only in IMU mode
    if (param_operation_mode.compare("IMU") == 0)
    {
        std::cerr << "gyro_calib_status: " << gyro_calib_status << ", "
                  << "acce_calib_status: " << acce_calib_status <<  std::endl;
    
        if (gyro_calib_status == 3 && acce_calib_status == 3)
        {
            calibration_counter++;
        }
    } else if (param_operation_mode.compare("NDOF") == 0)
    {
        std::cerr << "system_calib_status: " << system_calib_status << ", "
                  << "gyro_calib_status: "   << gyro_calib_status   << ", "
                  << "acce_calib_status: "   << acce_calib_status   << ", "
                  << "mag_calib_status: "    << mag_calib_status    << std::endl;
    
        // It is hard to get system calib status to 3 when all sensors' 
        // stauts (especially accelerometer) are 3.
        // Follow Sec. 3.10 of bno055 datasheet.
        //
        // Even though system_calib_status = 3, when we read the calib profile, it is not 3.
        // So it should be okay not to check system_calib_status
        if (system_calib_status == 3 && gyro_calib_status == 3 && 
            acce_calib_status   == 3 && mag_calib_status  == 3)
        {
            calibration_counter++;
        }
    }

    if (calibration_counter >= 3)
    {
        status = FULL_CALIBRATED;
    }

    return status;
}

void BNO055I2CCalibNode::readCalibration()
{
    int accel_offset_x, accel_offset_y, accel_offset_z;
    int mag_offset_x,   mag_offset_y,   mag_offset_z;
    int gyro_offset_x,  gyro_offset_y,  gyro_offset_z;
    int accel_radius;
    int mag_radius;

    // Read calibration data
    imu->readCalib(calib_data);

    accel_offset_x = (calib_data[1] << 8) | calib_data[0];
    accel_offset_y = (calib_data[3] << 8) | calib_data[2];
    accel_offset_z = (calib_data[5] << 8) | calib_data[4];

    mag_offset_x   = (calib_data[7] << 8) | calib_data[6];
    mag_offset_y   = (calib_data[9] << 8) | calib_data[8];
    mag_offset_z   = (calib_data[11] << 8) | calib_data[10];

    gyro_offset_x  = (calib_data[13] << 8) | calib_data[12];
    gyro_offset_y  = (calib_data[15] << 8) | calib_data[14];
    gyro_offset_z  = (calib_data[17] << 8) | calib_data[16];

    accel_radius   = (calib_data[19] << 8) | calib_data[18];
    mag_radius     = (calib_data[21] << 8) | calib_data[20];

    // Print calibration info
    std::cerr << "[Calibration data]\n" << std::endl; 
    std::cerr << "accel_offset_x: " << accel_offset_x << std::endl; 
    std::cerr << "accel_offset_y: " << accel_offset_y << std::endl; 
    std::cerr << "accel_offset_z: " << accel_offset_z << std::endl; 
    std::cerr << "\n" << std::endl; 
    
    std::cerr << "mag_offset_x: " << mag_offset_x << std::endl; 
    std::cerr << "mag_offset_y: " << mag_offset_y << std::endl; 
    std::cerr << "mag_offset_z: " << mag_offset_z << std::endl; 
    std::cerr << "\n" << std::endl; 
    
    std::cerr << "gyro_offset_x: " << gyro_offset_x << std::endl; 
    std::cerr << "gyro_offset_y: " << gyro_offset_y << std::endl; 
    std::cerr << "gyro_offset_z: " << gyro_offset_z << std::endl; 
    std::cerr << "\n" << std::endl; 

    std::cerr << "accel_radius: " << accel_radius << std::endl; 
    std::cerr << "mag_radius: "   << mag_radius << std::endl;
    std::cerr << "\n" << std::endl; 
}

void BNO055I2CCalibNode::writeCalibration()
{
    // Wrtie calibration data
    imu->writeCalib(calib_data);

    // Save calibration data into file
    FILE * fp = fopen(param_calib_file_name.c_str(), "wb");
    fwrite(calib_data, sizeof(uint8_t), CALIB_DATA_SIZE, fp);
    fclose(fp);
}


int main(int argc, char *argv[]) {
    BNO055I2CCalibNode node(argc, argv);
    node.run();

    ros::shutdown();

    return 0;
}
