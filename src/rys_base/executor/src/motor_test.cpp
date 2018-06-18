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
#include "sensor_msgs/msg/imu.hpp"

bool destruct = false;

struct SteeringData{
    float throttle;
    float rotation;
    int precision;
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
            while(result < 0){
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

            vm.lock();
            v = VectorFloat(voltages[0], voltages[1]-voltages[0], voltages[2]-voltages[1]);
            if (v.x < 3.3 || v.y < 3.3 || v.z < 3.3){
                std::cout << name << ": Low Voltage Warning: " << v.x << " " << v.y << " " << v.z << std::endl;
                vm.unlock();
                destroy = true;
                continue;
            }
            vm.unlock();

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
                fm.lock();
                f = (voltageSum/5)*100;
                // std::cout << f << std::endl;
                if (f > 60){
                    std::cout << name << ": Critical Temperature Warning: " << f << std::endl;
                    fm.unlock();
                    destroy = true;
                    continue;
                }
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

void motorsController(bool& activate, std::mutex& m, bool& destroy, IMU::ImuData& imu, std::mutex& imu_m, SteeringData& ster, std::mutex& ster_m){
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
    controller->newSetPIDParameters(0.1, 0.05, 0.00001, 2.0, 20.0, 0.01);

    for (int i = 1;i<21;i++){
        std::cout << name << ": " << i << std::endl; 
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

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
            roll = atan2(2.0 * (imu.orientationQuaternion[0] * imu.orientationQuaternion[1] + imu.orientationQuaternion[2] * imu.orientationQuaternion[3]),
                1.0 - 2.0 * (imu.orientationQuaternion[1] * imu.orientationQuaternion[1] + imu.orientationQuaternion[2] * imu.orientationQuaternion[2]));
            rotationX = imu.angularVelocity[0];
            // std::cout << roll << " : " << rotationX << " : " << imu.angularVelocity[1] << std::endl;
            imu_m.unlock();

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
    std::string name = "SteeringReceiver";
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

    auto node = rclcpp::Node::make_shared(nodeName, robotName, true);
    auto steeringCallback = 
        [&](const rys_interfaces::msg::Steering::SharedPtr message){
            throttle = message->throttle;
            rotation = message->rotation;
            precision = message->precision;
            newSteering = true;
        };
    auto sub = node->create_subscription<rys_interfaces::msg::Steering>("/" + robotName + "/control/steering", steeringCallback);

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
            if (numOfRuns > 11999) {
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

    auto node = rclcpp::Node::make_shared(nodeName, robotName, true);
    auto pub = node->create_publisher<sensor_msgs::msg::Imu>("/" + robotName + "/sensor/imuInfrequent");
    IMU::ImuData localData;
    auto message = std::make_shared<sensor_msgs::msg::Imu>();

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
            message->header.frame_id = "MPU6050";
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
            for (int i = 0; i < 9; ++i) {
                message->orientation_covariance[i] = 0;
                message->angular_velocity_covariance[i] = 0;
                message->linear_acceleration_covariance[i] = 0;
            }

            pub->publish(message);

            numOfRuns++;
            if (numOfRuns > 2999) {
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

    VectorFloat batteryStatus;
    std::mutex b_mutex;
    bool BATreader_bool = false;
    std::mutex BATreader_mutex;
    std::thread t1(BATreader, std::ref(BATreader_bool), std::ref(BATreader_mutex), std::ref(destruct),
                    std::ref(batteryStatus), std::ref(b_mutex));

    exec->addExec(std::ref(BATreader_mutex), std::ref(BATreader_bool), std::chrono::milliseconds(1000));

    float temperature;
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

    bool motors_bool = false;
    std::mutex motors_mutex;
    SteeringData steering_data;
    std::mutex steering_data_mutex;
    std::thread t4(motorsController, std::ref(motors_bool), std::ref(motors_mutex), std::ref(destruct),
                    std::ref(imuData), std::ref(imuData_mutex),
                    std::ref(steering_data), std::ref(steering_data_mutex));

    exec->addExec(std::ref(motors_mutex), std::ref(motors_bool), std::chrono::milliseconds(10));

    bool steering_bool = false;
    std::mutex steering_mutex;
    std::thread t5(steeringReceiver, std::ref(steering_bool), std::ref(steering_mutex), std::ref(destruct),
                    std::ref(steering_data), std::ref(steering_data_mutex));

    exec->addExec(std::ref(steering_mutex), std::ref(steering_bool), std::chrono::milliseconds(5));

    bool roll_sender_bool = false;
    std::mutex sender_mutex;
    std::thread t6(rollSender, std::ref(roll_sender_bool), std::ref(sender_mutex), std::ref(destruct),
                    std::ref(imuData), std::ref(imuData_mutex));

    exec->addExec(std::ref(sender_mutex), std::ref(roll_sender_bool), std::chrono::milliseconds(20));
 
    exec->list();
    exec->spin();
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
    delete exec;
}