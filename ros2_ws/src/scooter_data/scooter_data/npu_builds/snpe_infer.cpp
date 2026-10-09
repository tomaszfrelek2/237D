// snpe_infer.cpp - a minimal C interface around the SNPE C API, so Python (ctypes)
// can load a DLC once and run it on the NPU/CPU/GPU from inside a ROS node.
//
// Build on the device:  ./build.sh   (see that file)
//
// Exposed functions (all return 0 / non-null on success; si_last_error() explains failures):
//   void*       si_open(const char* dlc, int runtime, int perf, const char* output_names_csv)
//                 runtime: 0 = CPU, 1 = GPU, 2 = DSP/NPU
//                 perf:    0 = default, 1 = balanced, 2 = high performance,
//                          3 = sustained high performance, 4 = burst
//   long        si_input_size(void*)                    elements in the (single) input tensor
//   int         si_input_dims(void*, long* dims, int max) rank, dims written to `dims`
//   int         si_run(void*, const float* data, long n)
//   int         si_num_outputs(void*)
//   const char* si_output_name(void*, int i)
//   long        si_output_size(void*, int i)
//   int         si_get_output(void*, int i, float* dst, long n)
//   void        si_close(void*)
//   const char* si_last_error()

#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "DlContainer/DlContainer.h"
#include "DlSystem/DlEnums.h"
#include "DlSystem/DlError.h"
#include "DlSystem/ITensor.h"
#include "DlSystem/RuntimeList.h"
#include "DlSystem/StringList.h"
#include "DlSystem/TensorMap.h"
#include "DlSystem/TensorShape.h"
#include "SNPE/SNPE.h"
#include "SNPE/SNPEBuilder.h"
#include "SNPE/SNPEUtil.h"

namespace {

struct Model {
  Snpe_SNPE_Handle_t snpe = nullptr;
  Snpe_ITensor_Handle_t in_tensor = nullptr;
  Snpe_TensorMap_Handle_t in_map = nullptr;
  Snpe_TensorMap_Handle_t out_map = nullptr;
  std::string in_name;
  std::vector<long> in_dims;
  long in_size = 0;
  std::vector<std::string> out_names;
  std::vector<long> out_sizes;
};

thread_local std::string g_err;

void set_err(const std::string& where) {
  const char* e = Snpe_ErrorCode_GetLastErrorString();
  g_err = where + (e ? std::string(": ") + e : std::string());
}

Snpe_Runtime_t to_runtime(int r) {
  switch (r) {
    case 1: return SNPE_RUNTIME_GPU;
    case 2: return SNPE_RUNTIME_DSP;
    default: return SNPE_RUNTIME_CPU;
  }
}

Snpe_PerformanceProfile_t to_perf(int p) {
  switch (p) {
    case 1: return SNPE_PERFORMANCE_PROFILE_BALANCED;
    case 2: return SNPE_PERFORMANCE_PROFILE_HIGH_PERFORMANCE;
    case 3: return SNPE_PERFORMANCE_PROFILE_SUSTAINED_HIGH_PERFORMANCE;
    case 4: return SNPE_PERFORMANCE_PROFILE_BURST;
    default: return SNPE_PERFORMANCE_PROFILE_DEFAULT;
  }
}

// Run once and record the output names/sizes (also warms up the model).
bool refresh_outputs(Model* m) {
  m->out_names.clear();
  m->out_sizes.clear();
  Snpe_StringList_Handle_t names = Snpe_TensorMap_GetTensorNames(m->out_map);
  size_t n = Snpe_StringList_Size(names);
  for (size_t i = 0; i < n; ++i) {
    std::string name = Snpe_StringList_At(names, i);
    Snpe_ITensor_Handle_t t = Snpe_TensorMap_GetTensor_Ref(m->out_map, name.c_str());
    m->out_names.push_back(name);
    m->out_sizes.push_back(static_cast<long>(Snpe_ITensor_GetSize(t)));
  }
  Snpe_StringList_Delete(names);
  return n > 0;
}

}  // namespace

extern "C" {

const char* si_last_error() { return g_err.c_str(); }

int si_run(void* h, const float* data, long n);
void si_close(void* h);

void* si_open(const char* dlc, int runtime, int perf, const char* output_names_csv) {
  Snpe_Runtime_t rt = to_runtime(runtime);
  if (!Snpe_Util_IsRuntimeAvailable(rt)) {
    g_err = "requested runtime is not available on this device (check libcdsprpc, "
            "ADSP_LIBRARY_PATH and /dev/fastrpc-* permissions for the NPU)";
    return nullptr;
  }
  Snpe_DlContainer_Handle_t container = Snpe_DlContainer_Open(dlc);
  if (!container) { set_err(std::string("cannot open DLC ") + dlc); return nullptr; }

  Snpe_SNPEBuilder_Handle_t builder = Snpe_SNPEBuilder_Create(container);
  Snpe_RuntimeList_Handle_t rtl = Snpe_RuntimeList_Create();
  Snpe_RuntimeList_Add(rtl, rt);
  Snpe_SNPEBuilder_SetRuntimeProcessorOrder(builder, rtl);
  Snpe_SNPEBuilder_SetPerformanceProfile(builder, to_perf(perf));

  Snpe_StringList_Handle_t outs = nullptr;
  if (output_names_csv && *output_names_csv) {
    outs = Snpe_StringList_Create();
    std::stringstream ss(output_names_csv);
    std::string item;
    while (std::getline(ss, item, ',')) {
      if (!item.empty()) Snpe_StringList_Append(outs, item.c_str());
    }
    Snpe_SNPEBuilder_SetOutputTensors(builder, outs);
  }

  Snpe_SNPE_Handle_t snpe = Snpe_SNPEBuilder_Build(builder);
  if (outs) Snpe_StringList_Delete(outs);
  Snpe_RuntimeList_Delete(rtl);
  Snpe_SNPEBuilder_Delete(builder);
  Snpe_DlContainer_Delete(container);
  if (!snpe) { set_err("building the SNPE network failed"); return nullptr; }

  Model* m = new Model();
  m->snpe = snpe;

  Snpe_StringList_Handle_t in_names = Snpe_SNPE_GetInputTensorNames(snpe);
  if (!in_names || Snpe_StringList_Size(in_names) != 1) {
    g_err = "this helper supports models with exactly one input";
    if (in_names) Snpe_StringList_Delete(in_names);
    Snpe_SNPE_Delete(snpe);
    delete m;
    return nullptr;
  }
  m->in_name = Snpe_StringList_At(in_names, 0);
  Snpe_StringList_Delete(in_names);

  Snpe_TensorShape_Handle_t shape = Snpe_SNPE_GetInputDimensions(snpe, m->in_name.c_str());
  size_t rank = Snpe_TensorShape_Rank(shape);
  for (size_t i = 0; i < rank; ++i) m->in_dims.push_back(static_cast<long>(Snpe_TensorShape_At(shape, i)));
  m->in_tensor = Snpe_Util_CreateITensor(shape);
  Snpe_TensorShape_Delete(shape);
  m->in_size = static_cast<long>(Snpe_ITensor_GetSize(m->in_tensor));

  m->in_map = Snpe_TensorMap_Create();
  Snpe_TensorMap_Add(m->in_map, m->in_name.c_str(), m->in_tensor);

  // Warm-up run with zeros: learns the output names/sizes.
  std::vector<float> zeros(m->in_size, 0.0f);
  if (si_run(m, zeros.data(), m->in_size) != 0) {
    std::string e = g_err;
    si_close(m);
    g_err = "warm-up inference failed: " + e;
    return nullptr;
  }
  if (!refresh_outputs(m)) {
    g_err = "model produced no outputs";
    si_close(m);
    return nullptr;
  }
  return m;
}

long si_input_size(void* h) { return static_cast<Model*>(h)->in_size; }

int si_input_dims(void* h, long* dims, int max) {
  Model* m = static_cast<Model*>(h);
  int n = static_cast<int>(m->in_dims.size());
  for (int i = 0; i < n && i < max; ++i) dims[i] = m->in_dims[i];
  return n;
}

int si_run(void* h, const float* data, long n) {
  Model* m = static_cast<Model*>(h);
  if (n != m->in_size) {
    std::ostringstream os;
    os << "input has " << n << " values, model expects " << m->in_size;
    g_err = os.str();
    return 1;
  }
  std::memcpy(Snpe_ITensor_GetData(m->in_tensor), data, sizeof(float) * n);
  if (m->out_map) Snpe_TensorMap_Delete(m->out_map);
  m->out_map = Snpe_TensorMap_Create();
  if (Snpe_SNPE_ExecuteITensors(m->snpe, m->in_map, m->out_map) != SNPE_SUCCESS) {
    set_err("inference failed");
    return 2;
  }
  return 0;
}

int si_num_outputs(void* h) { return static_cast<int>(static_cast<Model*>(h)->out_names.size()); }

const char* si_output_name(void* h, int i) { return static_cast<Model*>(h)->out_names.at(i).c_str(); }

long si_output_size(void* h, int i) { return static_cast<Model*>(h)->out_sizes.at(i); }

int si_get_output(void* h, int i, float* dst, long n) {
  Model* m = static_cast<Model*>(h);
  if (i < 0 || i >= static_cast<int>(m->out_names.size())) { g_err = "bad output index"; return 1; }
  Snpe_ITensor_Handle_t t = Snpe_TensorMap_GetTensor_Ref(m->out_map, m->out_names[i].c_str());
  if (!t) { g_err = "output missing after inference"; return 2; }
  long size = static_cast<long>(Snpe_ITensor_GetSize(t));
  if (size != n) { g_err = "output buffer size mismatch"; return 3; }
  std::memcpy(dst, Snpe_ITensor_GetData(t), sizeof(float) * n);
  return 0;
}

void si_close(void* h) {
  Model* m = static_cast<Model*>(h);
  if (!m) return;
  if (m->out_map) Snpe_TensorMap_Delete(m->out_map);
  if (m->in_map) Snpe_TensorMap_Delete(m->in_map);
  if (m->in_tensor) Snpe_ITensor_Delete(m->in_tensor);
  if (m->snpe) Snpe_SNPE_Delete(m->snpe);
  delete m;
}

}  // extern "C"