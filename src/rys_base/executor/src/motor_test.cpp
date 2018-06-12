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

bool destruct = false;

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
            if (numOfRuns > 9999) {
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

void motorsController(bool& activate, std::mutex& m, bool& destroy,
                    IMU::ImuData& imu, std::mutex& imu_m){
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
    float rotation = 0;
    bool layingDown = false;
    bool standingUp = false;
    int standUpMultiplier;
    bool standUpPhase = false;
    std::chrono::milliseconds standUpTimer;
    std::chrono::milliseconds rate = std::chrono::milliseconds(10);

    MotorsController * controller = new MotorsController();
    controller->enableMotors();
    controller->setInvertSpeed(true, false);
    controller->setMotorsSwapped(true);
    controller->setLQREnabled(false);
    controller->setBalancing(true);
    controller->setSpeedFilterFactor(1);
    controller->setAngleFilterFactor(1);
    controller->setPIDSpeedRegulatorEnabled(true);
    // Ku = 10, T = 60ms = 0.06s ===> K = 0.6*10 = 6, InvTi = 2/0.06 = 33.(3), Td  = 0.06/8 = 0.0075
    // initial
    // controller->newSetPIDParameters(0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    // working angle PID
    // controller->newSetPIDParameters(0.0, 0.0, 0.0, 2.0, 20.0, 0);
    controller->newSetPIDParameters(0.0, 0.0, 0.0, 2.0, 20.0, 0);

    std::this_thread::sleep_for(std::chrono::seconds(20));

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
            std::cout << roll << " : " << rotationX << " : " << imu.angularVelocity[1] << std::endl;
            imu_m.unlock();

            layingDown = (roll > 1.0 && previousRoll > 1.0) || (roll < -1.0 && previousRoll < -1.0);
            if(layingDown && !standingUp){
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

            if (!standingUp) {
                float leftSpeed = controller->getMotorSpeedLeftRaw();
                float rightSpeed = controller->getMotorSpeedRightRaw();
                float linearSpeed = (leftSpeed + rightSpeed) / 2;
                float finalLeftSpeed = 0;
                float finalRightSpeed = 0;
                controller->calculateSpeeds(roll, rotationX, linearSpeed, 0, rotation, finalLeftSpeed, finalRightSpeed, loopTimeRun);
                std::cout << "L: " << finalLeftSpeed << "\t R: " << finalRightSpeed << std::endl;
                controller->setMotorSpeeds(finalLeftSpeed, finalRightSpeed, 32, false);
            }
            
            numOfRuns++;
            if (numOfRuns > 9999) {
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

int main()
{
    setRTPriority();
    std::string name = "main";
    pthread_setname_np(pthread_self(), name.c_str());
    signal(SIGINT, sigintHandler);

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
    std::thread t4(motorsController, std::ref(motors_bool), std::ref(motors_mutex), std::ref(destruct),
                    std::ref(imuData), std::ref(imuData_mutex));

    exec->addExec(std::ref(motors_mutex), std::ref(motors_bool), std::chrono::milliseconds(10));
 
    exec->list();
    exec->spin();
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    delete exec;
}