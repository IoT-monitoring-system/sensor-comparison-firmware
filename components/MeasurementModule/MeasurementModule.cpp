#include "MeasurementModule.h"

MeasurementModule::MeasurementModule(){

};

void MeasurementModule::configure(){

};

void MeasurementModule::start(){

    // for (MeasurementTask *measurementTask : this->tasks) {
    //   measurement_task_config_t taskConfig =
    //   measurementTask->getConfiguration();

    //   BaseType_t result=
    //   xTaskCreatePinnedToCore(&measurementTask->measurementTaskWrapper,
    //   taskConfig.pcName, taskConfig.usStackDepth, taskConfig.pvParameters,
    //   taskConfig.uxPriority, measurementTask.);

    //   if (result == pdFALSE) {
    //     //error
    //     return;
    //   } else {

    //   }
    // };
};

void MeasurementModule::stop(){

};

void MeasurementModule::terminate(){

};

void MeasurementModule::addMeasurementTask(MeasurementTask *measurementTask) {
  this->tasks.push_back(measurementTask);
};

void MeasurementModule::removeMeasurementTask(
    MeasurementTask *measurementTask) {

  this->tasks.erase(
      std::remove(this->tasks.begin(), this->tasks.end(), measurementTask),
      this->tasks.end());
};
