// 只供自动化测试使用的 SDK 替身；不代表真实相机参数或海康转换算法。
#include <MvCameraControl.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

namespace
{
std::map<std::string, int> counts;
std::map<std::string, float> floats{{"ExposureTime", 1000}, {"Gain", 4}, {"AcquisitionFrameRate", 30}};
std::map<std::string, int64_t> integers{{"Width", 2}, {"Height", 2}, {"OffsetX", 0}, {"OffsetY", 0}};
std::map<std::string, unsigned int> enums{
  {"PixelFormat", PixelType_Gvsp_RGB8_Packed}, {"ExposureAuto", 2}, {"GainAuto", 2},
  {"BalanceWhiteAuto", 2}, {"TriggerMode", 1}, {"AcquisitionMode", 2}};
int devices = 1;
bool enabled = false;
unsigned char pixels[12]{};
bool borrowed = false;
MV_CC_DEVICE_INFO descriptions[2]{};

bool failing(const char * stage)
{
  const char * setting = std::getenv("SRM_TEST_MVS_FAIL");
  return setting && std::string(setting) == stage;
}

std::string symbolic(const std::string & key, unsigned int value)
{
  if (key == "PixelFormat") {
    if (value == PixelType_Gvsp_RGB8_Packed) return "RGB8Packed";
    if (value == PixelType_Gvsp_BGR8_Packed) return "BGR8Packed";
    if (value == PixelType_Gvsp_Mono8) return "Mono8";
  } else if (key == "AcquisitionMode") { if (value == 2) return "Continuous"; }
  else { if (value == 0) return "Off"; if (value == 1) return "On"; if (value == 2) return "Continuous"; }
  return {};
}
}  // namespace

extern "C" {
void srm_mvs_mock_reset(int count)
{
  counts.clear(); devices = count; borrowed = false; enabled = false;
  floats = {{"ExposureTime", 1000}, {"Gain", 4}, {"AcquisitionFrameRate", 30}};
  integers = {{"Width", 2}, {"Height", 2}, {"OffsetX", 0}, {"OffsetY", 0}};
  enums = {{"PixelFormat", PixelType_Gvsp_RGB8_Packed}, {"ExposureAuto", 2}, {"GainAuto", 2},
           {"BalanceWhiteAuto", 2}, {"TriggerMode", 1}, {"AcquisitionMode", 2}};
}
int srm_mvs_mock_stat(const char * key) { return counts[key]; }
void srm_mvs_mock_pixel(unsigned int type) { enums["PixelFormat"] = type; }

unsigned int MV_CC_GetSDKVersion() { return failing("version") ? 0x04010202 : 0x04070003; }
int MV_CC_Initialize() { ++counts["initialize"]; return MV_OK; }
int MV_CC_Finalize() { ++counts["finalize"]; return MV_OK; }
int MV_CC_EnumDevices(unsigned int type, MV_CC_DEVICE_INFO_LIST * result)
{
  ++counts["enumerate"];
  if (type != MV_USB_DEVICE || failing("enumerate")) return MV_E_PARAMETER;
  *result = {};
  result->nDeviceNum = devices;
  for (int i = 0; i < devices; ++i) {
    descriptions[i] = {};
    descriptions[i].nTLayerType = MV_USB_DEVICE;
    auto & usb = descriptions[i].SpecialInfo.stUsb3VInfo;
    std::strcpy(reinterpret_cast<char *>(usb.chModelName), "MOCK-CS016");
    std::strcpy(reinterpret_cast<char *>(usb.chSerialNumber), i == 0 ? "MOCK-A" : "MOCK-B");
    usb.idVendor = 0x2bdf; usb.idProduct = 1;
    result->pDeviceInfo[i] = &descriptions[i];
  }
  return MV_OK;
}
int MV_CC_CreateHandle(void ** handle, const MV_CC_DEVICE_INFO * info)
{
  ++counts["create"];
  *handle = new MV_CC_DEVICE_INFO(*info);
  return MV_OK;
}
int MV_CC_OpenDevice(void *, unsigned int access, unsigned short)
{
  ++counts["open"];
  return failing("open") || access != MV_ACCESS_Exclusive ? MV_E_USB_READ : MV_OK;
}
int MV_CC_CloseDevice(void *) { ++counts["close"]; return MV_OK; }
int MV_CC_DestroyHandle(void * handle)
{
  ++counts["destroy"];
  delete static_cast<MV_CC_DEVICE_INFO *>(handle);
  return MV_OK;
}
int MV_CC_StartGrabbing(void *) { ++counts["start"]; return failing("start") ? MV_E_USB_READ : MV_OK; }
int MV_CC_StopGrabbing(void *) { ++counts["stop"]; return MV_OK; }
int MV_CC_SetImageNodeNum(void *, unsigned int number) { counts["nodes"] = number; return MV_OK; }
int MV_XML_GetNodeAccessMode(void *, const char * key, MV_XML_AccessMode * mode)
{
  *mode = failing("gain_ro") && std::string(key) == "Gain" ? AM_RO : AM_RW;
  return MV_OK;
}
int MV_CC_GetIntValueEx(void *, const char * key, MVCC_INTVALUE_EX * value)
{
  if (!integers.count(key)) return MV_E_GC_ARGUMENT;
  *value = {};
  value->nCurValue = integers[key]; value->nMin = 0; value->nMax = 2048; value->nInc = 2;
  return MV_OK;
}
int MV_CC_SetIntValueEx(void *, const char * key, int64_t value)
{
  ++counts["set_integer"];
  integers[key] = value;
  return MV_OK;
}
int MV_CC_GetFloatValue(void *, const char * key, MVCC_FLOATVALUE * value)
{
  if (!floats.count(key)) return MV_E_GC_ARGUMENT;
  *value = {};
  value->fCurValue = floats[key];
  value->fMin = std::string(key) == "Gain" ? 0 : 1;
  value->fMax = std::string(key) == "ExposureTime" ? 10000 : 100;
  return MV_OK;
}
int MV_CC_SetFloatValue(void *, const char * key, float value)
{
  ++counts["set_float"];
  if (!failing("readback")) floats[key] = value;
  return MV_OK;
}
int MV_CC_GetEnumValue(void *, const char * key, MVCC_ENUMVALUE * value)
{
  if (!enums.count(key)) return MV_E_GC_ARGUMENT;
  *value = {};
  value->nCurValue = enums[key]; value->nSupportedNum = 3;
  if (std::string(key) == "PixelFormat") {
    value->nSupportValue[0] = PixelType_Gvsp_RGB8_Packed;
    value->nSupportValue[1] = PixelType_Gvsp_BGR8_Packed;
    value->nSupportValue[2] = PixelType_Gvsp_Mono8;
  } else { value->nSupportValue[0] = 0; value->nSupportValue[1] = 1; value->nSupportValue[2] = 2; }
  return MV_OK;
}
int MV_CC_GetEnumEntrySymbolic(void *, const char * key, MVCC_ENUMENTRY * entry)
{
  const auto name = symbolic(key, entry->nValue);
  if (name.empty()) return MV_E_GC_ARGUMENT;
  std::strcpy(entry->chSymbolic, name.c_str());
  return MV_OK;
}
int MV_CC_SetEnumValueByString(void *, const char * key, const char * value)
{
  ++counts["set_enum"];
  for (unsigned int candidate : {0U, 1U, 2U, static_cast<unsigned int>(PixelType_Gvsp_RGB8_Packed),
       static_cast<unsigned int>(PixelType_Gvsp_BGR8_Packed), static_cast<unsigned int>(PixelType_Gvsp_Mono8)}) {
    if (symbolic(key, candidate) == value) { enums[key] = candidate; return MV_OK; }
  }
  return MV_E_GC_ARGUMENT;
}
int MV_CC_GetBoolValue(void *, const char *, bool * value) { *value = enabled; return MV_OK; }
int MV_CC_SetBoolValue(void *, const char *, bool value) { ++counts["set_bool"]; enabled = value; return MV_OK; }

int MV_CC_GetImageBuffer(void *, MV_FRAME_OUT * frame, unsigned int timeout)
{
  ++counts["read"]; counts["timeout_ms"] = timeout;
  if (failing("read") || (failing("read_after_two") && counts["read"] > 2)) return MV_E_NODATA;
  if (borrowed) return MV_E_CALLORDER;
  borrowed = true;
  *frame = {};
  for (int i = 0; i < 12; ++i) pixels[i] = static_cast<unsigned char>((i % 3 + 1) * 10);
  frame->pBufAddr = pixels;
  frame->stFrameInfo.nWidth = 2; frame->stFrameInfo.nHeight = 2;
  frame->stFrameInfo.enPixelType = static_cast<MvGvspPixelType>(enums["PixelFormat"]);
  frame->stFrameInfo.nFrameLen = frame->stFrameInfo.enPixelType == PixelType_Gvsp_Mono8 ? 4 : 12;
  if (failing("metadata")) frame->stFrameInfo.nWidth = 0;
  if (failing("truncated")) frame->stFrameInfo.nFrameLen = 1;
  if (failing("oversize")) frame->stFrameInfo.nExtendWidth = 0xffffffff;
  if (failing("lost_packet")) frame->stFrameInfo.nLostPacket = 1;
  return MV_OK;
}
int MV_CC_FreeImageBuffer(void *, MV_FRAME_OUT *)
{
  ++counts["free"];
  if (!borrowed) return MV_E_CALLORDER;
  borrowed = false;
  std::memset(pixels, 0xdd, sizeof(pixels));  // 检查 Mat 没有借用 SDK 的生命周期。
  return failing("free") ? MV_E_USB_READ : MV_OK;
}
int MV_CC_ConvertPixelTypeEx(void *, MV_CC_PIXEL_CONVERT_PARAM_EX * value)
{
  ++counts["convert"];
  if (failing("convert")) return MV_E_SUPPORT;
  const auto pixels_count = value->nWidth * value->nHeight;
  const bool mono = value->enSrcPixelType == PixelType_Gvsp_Mono8;
  if (value->enDstPixelType != PixelType_Gvsp_BGR8_Packed || value->nDstBufferSize < pixels_count * 3 ||
      value->nSrcDataLen < pixels_count * (mono ? 1 : 3)) return MV_E_PARAMETER;
  for (unsigned int i = 0; i < pixels_count; ++i) {
    for (unsigned int channel = 0; channel < 3; ++channel) {
      value->pDstBuffer[i * 3 + channel] = value->pSrcData[mono ? i : i * 3 + 2 - channel];
    }
  }
  value->nDstLen = failing("convert_length") ? 1 : pixels_count * 3;
  return MV_OK;
}
}  // extern "C"
