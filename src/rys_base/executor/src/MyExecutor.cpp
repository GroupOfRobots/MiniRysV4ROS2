#include <iostream>
#include <thread>
#include "MyExecutor.hpp"

Exec::Exec() {
	next = 0;
}

MyExecutor::MyExecutor(bool& destructionBool) {
	firstExec = 0;
	this->destroy = &destructionBool;
	std::cout << "Executor initialized." << std::endl;
}

MyExecutor::~MyExecutor() {
	std::cout << "Deleting executor." << std::endl;
	// *this->destroy = true;
	Exec *temp = this->firstExec;
	Exec *temp2 = temp->next;
	if (temp2){
		delete temp;
		temp = temp2;
		temp2 = temp->next;
	}
}

void MyExecutor::addExec(std::mutex& mut, bool& activationBool, std::chrono::milliseconds threadDelay) {
	std::cout << "Adding new exec" << std::endl;
	Exec *newExec = new Exec;
	newExec->m = &mut;
	newExec->activate = &activationBool;
	newExec->delay = threadDelay;
	newExec->nextActivationTime = std::chrono::high_resolution_clock::now();

	if (!this->firstExec) {
		this->firstExec = newExec;
	} else {
		Exec *temp = this->firstExec;
		while (temp->next){
			temp = temp->next;
		}
		temp->next = newExec;
	}
	std::cout << "New exec added" << std::endl;
}

void MyExecutor::spin(){
	if (!this->firstExec){
		std::cout << "Nothing to spin.." << std::endl;
		return;
	}
	Exec *temp = this->firstExec;
	while(temp){
		temp->nextActivationTime = std::chrono::high_resolution_clock::now() + temp->delay;
		temp = temp->next;
	}
	while(!*this->destroy){
		if (!temp) {
			temp = this->firstExec;
	        std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
		if (temp->nextActivationTime < std::chrono::high_resolution_clock::now()) {
			temp->m->lock();
			*temp->activate = true;
			temp->m->unlock();
			temp->nextActivationTime = temp->nextActivationTime + temp->delay;
		}
		temp = temp->next;
	}
}

void MyExecutor::list() {
	if (this->firstExec){
		Exec *temp = this->firstExec;
		std::cout << *temp->activate << " ; " << temp->delay.count() << std::endl;
		while(temp->next){
			temp = temp->next;
			std::cout << *temp->activate << " ; " << temp->delay.count() << std::endl;
		}
	} else std::cout << "Executor is empty..." << std::endl;
}