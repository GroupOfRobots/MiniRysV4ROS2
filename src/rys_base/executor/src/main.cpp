#include <iostream>
#include <thread>
#include <chrono>
#include <sched.h>
#include <sys/mman.h>
#include <cstring>
#include <mutex>
#include <csignal>
#include <fstream>
#include "MyExecutor.hpp"
#include "IMU.hpp"
#include "helper_3dmath.hpp"
#include "MotorsController.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/steering.hpp"
#include "rys_interfaces/msg/temperature_status.hpp"
#include "rys_interfaces/msg/battery_status.hpp"
#include "sensor_msgs/msg/imu.hpp"

bool destruct = false;

struct SteeringData{
    float throttle;
    float rotation;
    int precision;
    SteeringData() : throttle(0), rotation(0), precision(32){}
};

void sigintHandler(int signum) {
    if (signum == SIGINT) {
        destruct = true;
    }
}

void setRTPriority() {
    struct sched_param schedulerParams;
    schedulerParams.sched_priority = sched_get_priority_max(SCHED_FIFO)-1;
    std::cout << "Setting RT scheduling, priority " << schedulerParams.sched_priority << std::endl;
    if (sched_setscheduler(0, SCHED_FIFO, &schedulerParams) == -1) {
        std::cout << "WARNING: Setting RT scheduling failed: " << std::strerror(errno) << std::endl;
        return;
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        std::cout << "WARNING: Failed to lock memory: " << std::strerror(errno) << std::endl;
    }
}

void IMUreader(bool& activate, std::mutex& m, bool& destroy, IMU::ImuData& extData, std::mutex& dm){
    std::string name = "IMUreader";
    pthread_setname_np(pthread_self(), name.c_str());
    int numOfRuns = 0;
    float frequency = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();
    IMU * imu = new IMU();

    while(!destroy){
        m.lock();
        if(activate){
            activate = false;
            m.unlock();

            IMU::ImuData data;
            int result = -1;
            while(result < 0 && !destroy){
                result = imu->getData(&data);
            }

            dm.lock();
            extData = data;
            dm.unlock();

            numOfRuns++;
            if (numOfRuns > 5999) {
                previous = timeNow;
                timeNow = std::chrono::high_resolution_clock::now();
                auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(timeNow - previous);
                float loopTime = loopTimeSpan.count();
                frequency = numOfRuns/loopTime;
                std::cout << name << " Frequency " << frequency << "Hz after " << numOfRuns << " messages." << std::endl;
                numOfRuns = 0;
            }
        } else {
            m.unlock();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    delete imu;
    std::cout << name << ": I'm dying.." << std::endl;
}

void BATreader(bool& activate, std::mutex& m, bool& destroy, VectorFloat& v, std::mutex& vm){
    std::string name = "BATreader";
    pthread_setname_np(pthread_self(), name.c_str());
    int numOfRuns = 0;
    float frequency = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();

    const float coefficients[3] = { 734.4895, 340.7509, 214.1773 };
    const uint8_t inputNumbers[3] = { 3, 1, 6 };
    std::string filenames[3];
    for (int i = 0; i < 3; ++i) {
        filenames[i] = std::string("/sys/devices/platform/ocp/44e0d000.tscadc/TI-am335x-adc/iio:device0/in_voltage") + std::to_string(inputNumbers[i]) + std::string("_raw");
    }
    std::ifstream file;
    float voltages[3];
    int rawValue = 0;
    VectorFloat localVoltage(0, 0, 0);

    while(!destroy){
        m.lock();
        if(activate){
            activate = false;
            m.unlock();

            for (int i = 0; i < 3; ++i) {
                file.open(filenames[i], std::ios::in);
                file >> rawValue;
                file.close();
                voltages[i] = static_cast<float>(rawValue) / coefficients[i];
            }

            localVoltage = VectorFloat(voltages[0], voltages[1]-voltages[0], voltages[2]-voltages[1]);
            vm.lock();
            v = localVoltage;
            vm.unlock();
            if (localVoltage.x < 3.3 || localVoltage.y < 3.3 || localVoltage.z < 3.3){
                std::cout << name << ": Low Voltage Warning: " << localVoltage.x << " " << localVoltage.y << " " << localVoltage.z << std::endl;
                destroy = true;
                continue;
            }

            numOfRuns++;
            if (numOfRuns > 599) {
                previous = timeNow;
                timeNow = std::chrono::high_resolution_clock::now();
                auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(timeNow - previous);
                float loopTime = loopTimeSpan.count();
                frequency = numOfRuns/loopTime;
                std::cout << name << " Frequency " << frequency << "Hz after " << numOfRuns << " messages." << std::endl;
                numOfRuns = 0;
            }
        } else {
            m.unlock();
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << name << ": I'm dying.." << std::endl;
}

void TEMPreader(bool& activate, std::mutex& m, bool& destroy, float& f, std::mutex& fm){
    std::string name = "TEMPreader";
    pthread_setname_np(pthread_self(), name.c_str());
    int numOfRuns = 0;
    float frequency = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();

    const float coefficient = 564.7637;
    const uint8_t inputNumber = 5;
    std::string filename = std::string("/sys/devices/platform/ocp/44e0d000.tscadc/TI-am335x-adc/iio:device0/in_voltage") + std::to_string(inputNumber) + std::string("_raw");
    std::ifstream file;
    float voltageSum = 0;
    float voltage = 0;
    int rawValue = 0;
    int currentReadings = 0;

    while(!destroy){
        m.lock();
        if(activate){
            activate = false;
            m.unlock();

            file.open(filename, std::ios::in);
            file >> rawValue;
            file.close();
            voltageSum += static_cast<float>(rawValue) / coefficient;
            currentReadings++;
            if (currentReadings == 5){
                voltage = (voltageSum/5)*100;
                // std::cout << f << std::endl;
                if (voltage > 60){
                    std::cout << name << ": Critical Temperature Warning: " << voltage << std::endl;
                    destroy = true;
                    continue;
                }
                fm.lock();
                f = voltage;
                fm.unlock();
                currentReadings = 0;
                voltageSum = 0;
            }

            numOfRuns++;
            if (numOfRuns > 599) {
                previous = timeNow;
                timeNow = std::chrono::high_resolution_clock::now();
                auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(timeNow - previous);
                float loopTime = loopTimeSpan.count();
                frequency = numOfRuns/loopTime;
                std::cout << name << " Frequency " << frequency << "Hz after " << numOfRuns << " messages." << std::endl;
                numOfRuns = 0;
            }
        } else {
            m.unlock();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << name << ": I'm dying.." << std::endl;
}

void motorsController(bool& activate, std::mutex& m, bool& destroy, float PIDparams[6], IMU::ImuData& imu, std::mutex& imu_m, SteeringData& ster, std::mutex& ster_m){
    std::string name = "motors";
    pthread_setname_np(pthread_self(), name.c_str());
    int numOfRuns = 0;
    float frequency = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();

    std::chrono::time_point<std::chrono::high_resolution_clock> previousRun = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> timeNowRun = std::chrono::high_resolution_clock::now();
    float loopTimeRun;
    float roll = 0;
    float previousRoll = 0;
    float rotationX = 0;
    bool layingDown = false;
    bool standingUp = false;
    int standUpMultiplier;
    bool standUpPhase = false;
    std::chrono::milliseconds standUpTimer;
    std::chrono::milliseconds rate = std::chrono::milliseconds(10);

    float rotation = 0;
    float throttle = 0;
    int precision = 32;

    float balancing = true;

    MotorsController * controller = new MotorsController();
    controller->enableMotors();
    controller->setInvertSpeed(true, false);
    controller->setMotorsSwapped(true);
    controller->setLQREnabled(false);
    controller->setBalancing(balancing);
    controller->setSpeedFilterFactor(1);
    controller->setAngleFilterFactor(1);
    controller->setPIDSpeedRegulatorEnabled(true);
    // Ku = 10, T = 60ms = 0.06s ===> K = 0.6*10 = 6, InvTi = 2/0.06 = 33.(3), Td  = 0.06/8 = 0.0075
    // initial
    // controller->newSetPIDParameters(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    // working angle PID
    // controller->newSetPIDParameters(0.0, 0.0, 0.0, 2.0, 20.0, 0);
    // poorly working speed over angle PID
    // controller->newSetPIDParameters(0.05, 0.0, 0.00, 2.0, 20.0, 0);
    // sth maybe working
    // controller->newSetPIDParameters(0.1, 0.05, 0.00001, 2.0, 20.0, 0.01);
    controller->newSetPIDParameters(PIDparams[0], PIDparams[1], PIDparams[2], PIDparams[3], PIDparams[4], PIDparams[5]);

    for (int i = 1; i<21 && !destroy; i++){
        std::cout << name << ": " << i << std::endl; 
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    IMU::ImuData localData;

    while(!destroy){
        m.lock();
        if(activate){
            activate = false;
            m.unlock();

            previousRun = timeNowRun;
            timeNowRun = std::chrono::high_resolution_clock::now();
            auto loopTimeSpanRun = std::chrono::duration_cast<std::chrono::duration<float>>(timeNowRun - previousRun);
            loopTimeRun = loopTimeSpanRun.count();

            previousRoll = roll;
            imu_m.lock();
            localData = imu;
            imu_m.unlock();
            roll = atan2(2.0 * (localData.orientationQuaternion[0] * localData.orientationQuaternion[1] + localData.orientationQuaternion[2] * localData.orientationQuaternion[3]),
                1.0 - 2.0 * (localData.orientationQuaternion[1] * localData.orientationQuaternion[1] + localData.orientationQuaternion[2] * localData.orientationQuaternion[2]));
            rotationX = localData.angularVelocity[0];
            // std::cout << roll << " : " << rotationX << " : " << imu.angularVelocity[1] << std::endl;

            layingDown = (roll > 1.0 && previousRoll > 1.0) || (roll < -1.0 && previousRoll < -1.0);
            if(layingDown && !standingUp && balancing){
                standingUp = true;
                standUpMultiplier = (roll > 1.0 ? 1 : -1);
                standUpPhase = false;
                standUpTimer = std::chrono::milliseconds(0);
            }

            if (standingUp){
                if (!standUpPhase) {
                    if (standUpTimer < std::chrono::milliseconds(1000)){
                        controller->setMotorSpeeds(0,0,32,true);
                    } else {
                        if (controller->getMotorSpeedLeftRaw() == standUpMultiplier * 1.0 &&
                                controller->getMotorSpeedRightRaw() == standUpMultiplier * 1.0){
                            standUpPhase = true;
                        } else {
                            controller->setMotorSpeeds(standUpMultiplier * 1.0, standUpMultiplier * 1.0, 32, false);
                        }
                    }
                } else {
                    if((standUpMultiplier * roll) <= 0){
                        standingUp = false;
                        controller->setMotorSpeeds(0, 0, 32, true);
                        controller->newZeroPIDRegulator();
                    } else if(standUpTimer >= std::chrono::milliseconds(2000) && standUpMultiplier * roll > 1.0) {
                        standUpPhase = false;
                        standUpTimer = std::chrono::milliseconds(0);
                        controller->setMotorSpeeds(0, 0, 32, true);
                    } else {
                        controller->setMotorSpeeds(standUpMultiplier * (-1.0), standUpMultiplier * (-1.0), 32, false);
                    }
                }
                standUpTimer += rate;
            }

            ster_m.lock();
            throttle = ster.throttle;
            rotation = ster.rotation;
            precision = ster.precision;
            // std::cout << throttle << " : " << rotation << " : " << precision << std::endl;
            ster_m.unlock();

            if (!standingUp) {
                float leftSpeed = controller->getMotorSpeedLeftRaw();
                float rightSpeed = controller->getMotorSpeedRightRaw();
                float linearSpeed = (leftSpeed + rightSpeed) / 2;
                float finalLeftSpeed = 0;
                float finalRightSpeed = 0;
                controller->calculateSpeeds(roll, rotationX, linearSpeed, throttle, rotation, finalLeftSpeed, finalRightSpeed, loopTimeRun);
                // std::cout << "L: " << finalLeftSpeed << "\t R: " << finalRightSpeed << std::endl;
                if (balancing)
                    controller->setMotorSpeeds(finalLeftSpeed, finalRightSpeed, 32, false);
                else
                    controller->setMotorSpeeds(finalLeftSpeed, finalRightSpeed, precision, false);
            }
            
            numOfRuns++;
            if (numOfRuns > 5999) {
                previous = timeNow;
                timeNow = std::chrono::high_resolution_clock::now();
                auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(timeNow - previous);
                float loopTime = loopTimeSpan.count();
                frequency = numOfRuns/loopTime;
                std::cout << name << " Frequency " << frequency << "Hz after " << numOfRuns << " messages." << std::endl;
                numOfRuns = 0;
            }
        } else {
            m.unlock();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    controller->disableMotors();
    delete controller;
    std::cout << name << ": I'm dying.." << std::endl;
}

void steeringReceiver(bool& activate, std::mutex& m, bool& destroy, SteeringData& s, std::mutex& sm){
    std::string name = "SteeringRec";
    pthread_setname_np(pthread_self(), name.c_str());
    int numOfRuns = 0;
    float frequency = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();

    float throttle = 0;
    float rotation = 0;
    int precision = 32;

    float newSteering = false;
    sm.lock();
    s.throttle = throttle;
    s.rotation = rotation;
    s.precision = precision;
    // std::cout << throttle << rotation << precision << std::endl;
    sm.unlock();
    const std::string robotName = "rys";
    const std::string nodeName = "receiver";
    const rmw_qos_profile_t qos = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        false
    };

    auto node = rclcpp::Node::make_shared(nodeName, robotName, false);
    auto steeringCallback = 
        [&](const rys_interfaces::msg::Steering::SharedPtr message){
            throttle = message->throttle;
            rotation = message->rotation;
            precision = message->precision;
            newSteering = true;
        };
    auto sub = node->create_subscription<rys_interfaces::msg::Steering>("/" + robotName + "/control/steering", steeringCallback, qos);

    while(!destroy){
        m.lock();
        if(activate){
            activate = false;
            m.unlock();

            rclcpp::spin_some(node);

            if(newSteering){
                sm.lock();
                s.throttle = throttle;
                s.rotation = rotation;
                s.precision = precision;
                // std::cout << throttle << rotation << precision << std::endl;
                sm.unlock();
                newSteering = false;
            }

            numOfRuns++;
            if (numOfRuns > 1199) {
                previous = timeNow;
                timeNow = std::chrono::high_resolution_clock::now();
                auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(timeNow - previous);
                float loopTime = loopTimeSpan.count();
                frequency = numOfRuns/loopTime;
                std::cout << name << " Frequency " << frequency << "Hz after " << numOfRuns << " messages." << std::endl;
                numOfRuns = 0;
            }

        } else {
            m.unlock();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    rclcpp::shutdown();
    std::cout << name << ": I'm dying.." << std::endl;
}

void rollSender(bool& activate, std::mutex& m, bool& destroy, IMU::ImuData& extData, std::mutex& dm){
    std::string name = "rollSender";
    pthread_setname_np(pthread_self(), name.c_str());
    int numOfRuns = 0;
    float frequency = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();

    const std::string robotName = "rys";
    const std::string nodeName = "rollSender";
    const rmw_qos_profile_t qos = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        false
    };

    auto node = rclcpp::Node::make_shared(nodeName, robotName, false);
    auto pub = node->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imuInfrequent", qos);
    IMU::ImuData localData;
    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.frame_id = "MPU6050";
    for (int i = 0; i < 9; ++i) {
        message->orientation_covariance[i] = 0;
        message->angular_velocity_covariance[i] = 0;
        message->linear_acceleration_covariance[i] = 0;
    }
    

    while(!destroy){
        m.lock();
        if(activate){
            activate = false;
            m.unlock();

            dm.lock();
            localData = extData;
            dm.unlock();

            // std::cout << localData.orientationQuaternion[1] << std::endl;
            message->header.stamp = node->now();
            message->orientation.x = localData.orientationQuaternion[1];
            message->orientation.y = localData.orientationQuaternion[2];
            message->orientation.z = localData.orientationQuaternion[3];
            message->orientation.w = localData.orientationQuaternion[0];
            message->angular_velocity.x = localData.angularVelocity[0];
            message->angular_velocity.y = localData.angularVelocity[1];
            message->angular_velocity.z = localData.angularVelocity[2];
            message->linear_acceleration.x = localData.linearAcceleration[0];
            message->linear_acceleration.y = localData.linearAcceleration[1];
            message->linear_acceleration.z = localData.linearAcceleration[2];

            pub->publish(message);

            numOfRuns++;
            if (numOfRuns > 1199) {
                previous = timeNow;
                timeNow = std::chrono::high_resolution_clock::now();
                auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(timeNow - previous);
                float loopTime = loopTimeSpan.count();
                frequency = numOfRuns/loopTime;
                std::cout << name << " Frequency " << frequency << "Hz after " << numOfRuns << " messages." << std::endl;
                numOfRuns = 0;
            }

        } else {
            m.unlock();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    rclcpp::shutdown();
    std::cout << name << ": I'm dying.." << std::endl;
}

void remoteComm(bool& activate, std::mutex& m, bool& destroy, IMU::ImuData& extData, std::mutex& dm, SteeringData& s, std::mutex& sm, float& temperature, std::mutex& tm, VectorFloat& battery, std::mutex& bm){
    std::string name = "remoteComm";
    pthread_setname_np(pthread_self(), name.c_str());
    int numOfRuns = 0;
    float frequency = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> previous = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> timeNow = std::chrono::high_resolution_clock::now();

    float throttle = 0;
    float rotation = 0;
    int precision = 32;

    float newSteering = false;
    sm.lock();
    s.throttle = throttle;
    s.rotation = rotation;
    s.precision = precision;
    // std::cout << throttle << rotation << precision << std::endl;
    sm.unlock();
    const std::string robotName = "rys";
    const std::string nodeName = "remoteComm";
    const rmw_qos_profile_t qos = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        false
    };

    auto node = rclcpp::Node::make_shared(nodeName, robotName, false);
    auto steeringCallback = 
        [&](const rys_interfaces::msg::Steering::SharedPtr message){
            throttle = message->throttle;
            rotation = message->rotation;
            precision = message->precision;
            newSteering = true;
        };
    auto sub = node->create_subscription<rys_interfaces::msg::Steering>("/" + robotName + "/control/steering", steeringCallback, qos);
    auto pub = node->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imuInfrequent", qos);
    auto pubTemp = node->create_publisher<rys_interfaces::msg::TemperatureStatus>("/" + robotName + "/sensor/temperature", qos);
    auto pubBat = node->create_publisher<rys_interfaces::msg::BatteryStatus>("/" + robotName + "/sensor/battery", qos);

    IMU::ImuData localData;
    auto message = std::make_shared<sensor_msgs::msg::Imu>();
    message->header.frame_id = "MPU6050";
    for (int i = 0; i < 9; ++i) {
        message->orientation_covariance[i] = 0;
        message->angular_velocity_covariance[i] = 0;
        message->linear_acceleration_covariance[i] = 0;
    }

    float localTemperature = 0;
    auto messageTemp = std::make_shared<rys_interfaces::msg::TemperatureStatus>();
    messageTemp->header.frame_id = "LM35";
    messageTemp->temperature_critical = false;

    VectorFloat localBattery(0, 0, 0);
    auto messageBat = std::make_shared<rys_interfaces::msg::BatteryStatus>();
    messageBat->header.frame_id = "ADC";
    messageBat->voltage_low = false;

    while(!destroy){
        m.lock();
        if(activate){
            activate = false;
            m.unlock();

            dm.lock();
            localData = extData;
            dm.unlock();

            message->header.stamp = node->now();
            message->orientation.x = localData.orientationQuaternion[1];
            message->orientation.y = localData.orientationQuaternion[2];
            message->orientation.z = localData.orientationQuaternion[3];
            message->orientation.w = localData.orientationQuaternion[0];
            message->angular_velocity.x = localData.angularVelocity[0];
            message->angular_velocity.y = localData.angularVelocity[1];
            message->angular_velocity.z = localData.angularVelocity[2];
            message->linear_acceleration.x = localData.linearAcceleration[0];
            message->linear_acceleration.y = localData.linearAcceleration[1];
            message->linear_acceleration.z = localData.linearAcceleration[2];

            pub->publish(message);

            tm.lock();
            localTemperature = temperature;
            tm.unlock();

            messageTemp->header.stamp = node->now();
            messageTemp->temperature = localTemperature;

            pubTemp->publish(messageTemp);

            bm.lock();
            localBattery = battery;
            bm.unlock();

            messageBat->header.stamp = node->now();
            messageBat->voltage_cell1 = localBattery.x;
            messageBat->voltage_cell2 = localBattery.y;
            messageBat->voltage_cell3 = localBattery.z;

            pubBat->publish(messageBat);

            rclcpp::spin_some(node);

            if(newSteering){
                sm.lock();
                s.throttle = throttle;
                s.rotation = rotation;
                s.precision = precision;
                sm.unlock();
                newSteering = false;
            }

            numOfRuns++;
            if (numOfRuns > 1199) {
                previous = timeNow;
                timeNow = std::chrono::high_resolution_clock::now();
                auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(timeNow - previous);
                float loopTime = loopTimeSpan.count();
                frequency = numOfRuns/loopTime;
                std::cout << name << " Frequency " << frequency << "Hz after " << numOfRuns << " messages." << std::endl;
                numOfRuns = 0;
            }

        } else {
            m.unlock();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    rclcpp::shutdown();
    std::cout << name << ": I'm dying.." << std::endl;
}

int main(int argc, char * argv[]){
    std::string name = "main";
    pthread_setname_np(pthread_self(), name.c_str());
    signal(SIGINT, sigintHandler);
    std::cout << "Initializing ROS...\n";
    rclcpp::init(argc, argv);
    std::cout << "ROS initialized.\n";
    setRTPriority();

    MyExecutor *exec = new MyExecutor(std::ref(destruct));

    VectorFloat batteryStatus(0, 0, 0);
    std::mutex b_mutex;
    bool BATreader_bool = false;
    std::mutex BATreader_mutex;
    std::thread t1(BATreader, std::ref(BATreader_bool), std::ref(BATreader_mutex), std::ref(destruct),
                    std::ref(batteryStatus), std::ref(b_mutex));

    exec->addExec(std::ref(BATreader_mutex), std::ref(BATreader_bool), std::chrono::milliseconds(1000));

    float temperature = 0;
    std::mutex t_mutex;
    bool TEMPreader_bool = false;
    std::mutex TEMPreader_mutex;
    std::thread t2(TEMPreader, std::ref(TEMPreader_bool), std::ref(TEMPreader_mutex), std::ref(destruct),
                    std::ref(temperature), std::ref(t_mutex));

    exec->addExec(std::ref(TEMPreader_mutex), std::ref(TEMPreader_bool), std::chrono::milliseconds(200));

    IMU::ImuData imuData;
    std::mutex imuData_mutex;
    bool IMUreader_bool = false;
    std::mutex IMUreader_mutex;
    std::thread t3(IMUreader, std::ref(IMUreader_bool), std::ref(IMUreader_mutex), std::ref(destruct),
                    std::ref(imuData), std::ref(imuData_mutex));

    exec->addExec(std::ref(IMUreader_mutex), std::ref(IMUreader_bool), std::chrono::milliseconds(10));

    float PIDparams[6] = {0.05, 0.05, 0.0001, 2.0, 10.0, 0};
    if (argc == 8){
        if (!std::strcmp(argv[1], "-p")) {
            std::cout << "Reading custom PID parameters..." << std::endl;
            PIDparams[0] = atof(argv[2]);
            PIDparams[1] = atof(argv[3]);
            PIDparams[2] = atof(argv[4]);
            PIDparams[3] = atof(argv[5]);
            PIDparams[4] = atof(argv[6]);
            PIDparams[5] = atof(argv[7]);
        }
    }
    bool motors_bool = false;
    std::mutex motors_mutex;
    SteeringData steering_data;
    std::mutex steering_data_mutex;
    std::thread t4(motorsController, std::ref(motors_bool), std::ref(motors_mutex), std::ref(destruct), PIDparams,
                    std::ref(imuData), std::ref(imuData_mutex),
                    std::ref(steering_data), std::ref(steering_data_mutex));

    exec->addExec(std::ref(motors_mutex), std::ref(motors_bool), std::chrono::milliseconds(10));

    // bool steering_bool = false;
    // std::mutex steering_mutex;
    // std::thread t5(steeringReceiver, std::ref(steering_bool), std::ref(steering_mutex), std::ref(destruct),
    //                 std::ref(steering_data), std::ref(steering_data_mutex));

    // exec->addExec(std::ref(steering_mutex), std::ref(steering_bool), std::chrono::milliseconds(50));

    // bool roll_sender_bool = false;
    // std::mutex sender_mutex;
    // std::thread t6(rollSender, std::ref(roll_sender_bool), std::ref(sender_mutex), std::ref(destruct),
    //                 std::ref(imuData), std::ref(imuData_mutex));

    // exec->addExec(std::ref(sender_mutex), std::ref(roll_sender_bool), std::chrono::milliseconds(50));

    bool remoteComm_bool = false;
    std::mutex remoteComm_mutex;
    std::thread t7(remoteComm, std::ref(remoteComm_bool), std::ref(remoteComm_mutex), std::ref(destruct),
                    std::ref(imuData), std::ref(imuData_mutex),
                    std::ref(steering_data), std::ref(steering_data_mutex),
                    std::ref(temperature), std::ref(t_mutex),
                    std::ref(batteryStatus), std::ref(b_mutex));

    exec->addExec(std::ref(remoteComm_mutex), std::ref(remoteComm_bool), std::chrono::milliseconds(50));

    exec->list();
    exec->spin();
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    // t5.join();
    // t6.join();
    t7.join();
    delete exec;
}