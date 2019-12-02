#include <SNPE/SNPE.hpp>
#include <SNPE/SNPEBuilder.hpp>
#include <SNPE/SNPEFactory.hpp>
#include <DlContainer/IDlContainer.hpp>
#include <DlSystem/DlError.hpp>
#include <DlSystem/ITensor.hpp>
#include <DlSystem/ITensorFactory.hpp>
#include <DlSystem/IUserBuffer.hpp>
#include <DlSystem/IUserBufferFactory.hpp>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <array>

extern "C"{
  float run_model(float shitty_angle, float t_output);
  void init_model();
  float run_model_time_series(float inputData[120]);
}

struct testvar_t {
  float testvar;
};
