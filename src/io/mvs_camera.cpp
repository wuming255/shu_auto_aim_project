#include "io/mvs_camera.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <vector>

#ifdef SRM_HAVE_MVS
#include <dlfcn.h>
#include <MvCameraControl.h>
#endif

namespace srm::io
{

void validate_mvs_config(const MvsCameraConfig & config)
{
  if (config.device_index < -1 || (!config.serial_number.empty() && config.device_index != -1)) {
    throw std::invalid_argument("MVS selection requires either a serial number or an index >= 0");
  }
  if (config.timeout_ms == 0 || config.timeout_ms > 10000) {
    throw std::invalid_argument("MVS timeout must be within [1, 10000] ms");
  }
  if (config.exposure_us && (!std::isfinite(*config.exposure_us) || *config.exposure_us <= 0.0)) {
    throw std::invalid_argument("MVS exposure must be finite and positive (microseconds)");
  }
  if (config.gain && (!std::isfinite(*config.gain) || *config.gain < 0.0)) {
    throw std::invalid_argument("MVS gain must be finite and non-negative");
  }
}

#ifdef SRM_HAVE_MVS
namespace
{
std::string hex_code(unsigned int code)
{
  std::ostringstream out;
  out << "0x" << std::hex << std::setfill('0') << std::setw(8) << code;
  return out.str();
}

void check(int code, const std::string & operation)
{
  if (code != MV_OK) {
    throw std::runtime_error("MVS " + operation + " failed: " + hex_code(code));
  }
}

template <typename Character, std::size_t N>
std::string text(const Character (&buffer)[N])
{
  const auto * begin = reinterpret_cast<const char *>(buffer);
  return std::string(begin, ::strnlen(begin, N));
}

// 参考项目头文件声明的 ABI；初始化/结束接口在提供的 4.7 运行库中存在，
// 但参考头文件未声明。版本门限制在 4.7.x；本地实测版本为 4.7.0.3。
struct Sdk
{
  void * library = nullptr;
  std::string requested_path;
  std::string loaded_path;
  std::string version;
  bool initialized = false;
  std::mutex enumeration_mutex;
  int (*initialize)() = nullptr;
  int (*finalize)() = nullptr;
#define SRM_MVS_FUNCTION(name) decltype(&::name) name = nullptr
  SRM_MVS_FUNCTION(MV_CC_GetSDKVersion);
  SRM_MVS_FUNCTION(MV_CC_EnumDevices);
  SRM_MVS_FUNCTION(MV_CC_CreateHandle);
  SRM_MVS_FUNCTION(MV_CC_OpenDevice);
  SRM_MVS_FUNCTION(MV_CC_CloseDevice);
  SRM_MVS_FUNCTION(MV_CC_DestroyHandle);
  SRM_MVS_FUNCTION(MV_CC_StartGrabbing);
  SRM_MVS_FUNCTION(MV_CC_StopGrabbing);
  SRM_MVS_FUNCTION(MV_CC_GetImageBuffer);
  SRM_MVS_FUNCTION(MV_CC_FreeImageBuffer);
  SRM_MVS_FUNCTION(MV_CC_ConvertPixelTypeEx);
  SRM_MVS_FUNCTION(MV_CC_GetFloatValue);
  SRM_MVS_FUNCTION(MV_CC_SetFloatValue);
  SRM_MVS_FUNCTION(MV_CC_GetIntValueEx);
  SRM_MVS_FUNCTION(MV_CC_SetIntValueEx);
  SRM_MVS_FUNCTION(MV_CC_GetEnumValue);
  SRM_MVS_FUNCTION(MV_CC_GetEnumEntrySymbolic);
  SRM_MVS_FUNCTION(MV_CC_SetEnumValueByString);
  SRM_MVS_FUNCTION(MV_CC_GetBoolValue);
  SRM_MVS_FUNCTION(MV_CC_SetBoolValue);
  SRM_MVS_FUNCTION(MV_XML_GetNodeAccessMode);
  SRM_MVS_FUNCTION(MV_CC_SetImageNodeNum);
#undef SRM_MVS_FUNCTION

  template <typename Function>
  void bind(Function & function, const char * name)
  {
    ::dlerror();
    function = reinterpret_cast<Function>(::dlsym(library, name));
    const char * error = ::dlerror();
    if (error || !function) {
      throw std::runtime_error(std::string("MVS missing SDK symbol ") + name +
                               ": " + (error ? error : "null symbol"));
    }
  }

  explicit Sdk(const std::string & path) : requested_path(path)
  {
    const std::vector<std::string> candidates = path.empty()
      ? std::vector<std::string>{"libMvCameraControl.so", "/opt/MVS/lib/64/libMvCameraControl.so",
                                "/opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3"}
      : std::vector<std::string>{path};
    std::string errors;
    for (const auto & candidate : candidates) {
      library = ::dlopen(candidate.c_str(), RTLD_NOW | RTLD_LOCAL);
      if (library) { loaded_path = candidate; break; }
      errors += "\n  " + candidate + ": " + ::dlerror();
    }
    if (!library) {
      throw std::runtime_error("MVS runtime unavailable; set --mvs-library and the SDK "
                               "lib/64 directory in LD_LIBRARY_PATH:" + errors);
    }
    try {
      bind(MV_CC_GetSDKVersion, "MV_CC_GetSDKVersion");
      const auto value = MV_CC_GetSDKVersion();
      version = std::to_string(value >> 24) + "." + std::to_string((value >> 16) & 255) +
                "." + std::to_string((value >> 8) & 255) + "." + std::to_string(value & 255);
      if ((value >> 16) != 0x0407) {
        throw std::runtime_error("MVS runtime " + version + " is unsupported; use SDK 4.7.x");
      }
      bind(initialize, "MV_CC_Initialize");
      bind(finalize, "MV_CC_Finalize");
#define SRM_MVS_BIND(name) bind(name, #name)
      SRM_MVS_BIND(MV_CC_EnumDevices);
      SRM_MVS_BIND(MV_CC_CreateHandle);
      SRM_MVS_BIND(MV_CC_OpenDevice);
      SRM_MVS_BIND(MV_CC_CloseDevice);
      SRM_MVS_BIND(MV_CC_DestroyHandle);
      SRM_MVS_BIND(MV_CC_StartGrabbing);
      SRM_MVS_BIND(MV_CC_StopGrabbing);
      SRM_MVS_BIND(MV_CC_GetImageBuffer);
      SRM_MVS_BIND(MV_CC_FreeImageBuffer);
      SRM_MVS_BIND(MV_CC_ConvertPixelTypeEx);
      SRM_MVS_BIND(MV_CC_GetFloatValue);
      SRM_MVS_BIND(MV_CC_SetFloatValue);
      SRM_MVS_BIND(MV_CC_GetIntValueEx);
      SRM_MVS_BIND(MV_CC_SetIntValueEx);
      SRM_MVS_BIND(MV_CC_GetEnumValue);
      SRM_MVS_BIND(MV_CC_GetEnumEntrySymbolic);
      SRM_MVS_BIND(MV_CC_SetEnumValueByString);
      SRM_MVS_BIND(MV_CC_GetBoolValue);
      SRM_MVS_BIND(MV_CC_SetBoolValue);
      SRM_MVS_BIND(MV_XML_GetNodeAccessMode);
      SRM_MVS_BIND(MV_CC_SetImageNodeNum);
#undef SRM_MVS_BIND
      check(initialize(), "MV_CC_Initialize");
      initialized = true;
    } catch (...) {
      ::dlclose(library);
      library = nullptr;
      throw;
    }
  }

  ~Sdk()
  {
    if (initialized) (void)finalize();
    if (library) ::dlclose(library);
  }

  std::vector<MV_CC_DEVICE_INFO> enumerate()
  {
    // SDK 枚举指针在下一次枚举时失效，锁内拷贝设备描述。
    std::lock_guard<std::mutex> lock(enumeration_mutex);
    MV_CC_DEVICE_INFO_LIST list{};
    check(MV_CC_EnumDevices(MV_USB_DEVICE, &list), "MV_CC_EnumDevices(USB)");
    if (list.nDeviceNum > MV_MAX_DEVICE_NUM) throw std::runtime_error("MVS invalid device count");
    std::vector<MV_CC_DEVICE_INFO> result;
    for (unsigned int index = 0; index < list.nDeviceNum; ++index) {
      if (list.pDeviceInfo[index] && list.pDeviceInfo[index]->nTLayerType == MV_USB_DEVICE) {
        result.push_back(*list.pDeviceInfo[index]);
      }
    }
    return result;
  }
};

std::shared_ptr<Sdk> load_sdk(const std::string & path)
{
  static std::mutex mutex;
  static std::weak_ptr<Sdk> cached;
  std::lock_guard<std::mutex> lock(mutex);
  auto sdk = cached.lock();
  if (sdk && sdk->requested_path != path) {
    throw std::runtime_error("MVS simultaneous cameras must use the same SDK library path");
  }
  if (!sdk) { sdk = std::make_shared<Sdk>(path); cached = sdk; }
  return sdk;
}

std::string device_description(const MV_CC_DEVICE_INFO & device, std::size_t index)
{
  const auto & usb = device.SpecialInfo.stUsb3VInfo;
  std::ostringstream out;
  out << "index=" << index << " model=" << text(usb.chModelName)
      << " serial=" << text(usb.chSerialNumber) << " vid:pid=" << std::hex
      << std::setfill('0') << std::setw(4) << usb.idVendor << ':' << std::setw(4) << usb.idProduct;
  return out.str();
}
}  // namespace

struct MvsCamera::Impl
{
  std::shared_ptr<Sdk> sdk;
  void * handle = nullptr;
  bool opened = false;
  bool grabbing = false;
  unsigned int timeout_ms;
  std::string report;
  std::string error;
  double fps = 0.0;

  explicit Impl(const MvsCameraConfig & config)
    : sdk(load_sdk(config.sdk_library)), timeout_ms(config.timeout_ms) {}

  ~Impl()
  {
    if (grabbing) (void)sdk->MV_CC_StopGrabbing(handle);
    if (opened) (void)sdk->MV_CC_CloseDevice(handle);
    if (handle) (void)sdk->MV_CC_DestroyHandle(handle);
  }

  std::string access(const char * key)
  {
    MV_XML_AccessMode mode{};
    const auto code = sdk->MV_XML_GetNodeAccessMode(handle, key, &mode);
    if (code != MV_OK) return "access_error=" + hex_code(code);
    switch (mode) {
      case AM_RO: return "RO";
      case AM_RW: return "RW";
      case AM_WO: return "WO";
      default: return "unavailable";
    }
  }

  void set_enum(const char * key, const std::string & value)
  {
    MVCC_ENUMVALUE current{};
    check(sdk->MV_CC_GetEnumValue(handle, key, &current), std::string("query ") + key);
    MVCC_ENUMENTRY entry{};
    entry.nValue = current.nCurValue;
    check(sdk->MV_CC_GetEnumEntrySymbolic(handle, key, &entry), std::string("symbolic ") + key);
    if (text(entry.chSymbolic) == value) return;
    if (access(key) != "RW") throw std::runtime_error(std::string("MVS node is not writable: ") + key);
    check(sdk->MV_CC_SetEnumValueByString(handle, key, value.c_str()),
          std::string("set ") + key + "=" + value);
    check(sdk->MV_CC_GetEnumValue(handle, key, &current), std::string("read back ") + key);
    entry = {};
    entry.nValue = current.nCurValue;
    check(sdk->MV_CC_GetEnumEntrySymbolic(handle, key, &entry), std::string("symbolic ") + key);
    if (text(entry.chSymbolic) != value) throw std::runtime_error(std::string("MVS readback mismatch: ") + key);
  }

  void set_float(const char * key, double value)
  {
    MVCC_FLOATVALUE range{};
    check(sdk->MV_CC_GetFloatValue(handle, key, &range), std::string("query ") + key);
    if (!std::isfinite(range.fMin) || !std::isfinite(range.fMax) || range.fMin > range.fMax ||
        access(key) != "RW" || value < range.fMin || value > range.fMax) {
      throw std::runtime_error(std::string("MVS ") + key + "=" + std::to_string(value) +
        " is not writable or outside [" + std::to_string(range.fMin) + ", " + std::to_string(range.fMax) + "]");
    }
    check(sdk->MV_CC_SetFloatValue(handle, key, static_cast<float>(value)), std::string("set ") + key);
    check(sdk->MV_CC_GetFloatValue(handle, key, &range), std::string("read back ") + key);
    if (!std::isfinite(range.fCurValue) ||
        std::abs(range.fCurValue - value) > std::max(0.01, std::abs(value) * 0.001)) {
      throw std::runtime_error(std::string("MVS readback mismatch: ") + key + " actual=" + std::to_string(range.fCurValue));
    }
  }

  void set_integer(const char * key, int value)
  {
    if (value == 0) return;
    MVCC_INTVALUE_EX range{};
    check(sdk->MV_CC_GetIntValueEx(handle, key, &range), std::string("query ") + key);
    if (access(key) != "RW" || value < range.nMin || value > range.nMax ||
        (range.nInc > 0 && (value - range.nMin) % range.nInc != 0)) {
      throw std::runtime_error(std::string("MVS ") + key + "=" + std::to_string(value) +
        " is not writable or violates min=" + std::to_string(range.nMin) +
        " max=" + std::to_string(range.nMax) + " inc=" + std::to_string(range.nInc));
    }
    check(sdk->MV_CC_SetIntValueEx(handle, key, value), std::string("set ") + key);
    check(sdk->MV_CC_GetIntValueEx(handle, key, &range), std::string("read back ") + key);
    if (range.nCurValue != value) throw std::runtime_error(std::string("MVS readback mismatch: ") + key);
  }

  void describe_parameters()
  {
    std::ostringstream out;
    for (const auto * key : {"Width", "Height", "OffsetX", "OffsetY"}) {
      MVCC_INTVALUE_EX v{};
      const auto code = sdk->MV_CC_GetIntValueEx(handle, key, &v);
      out << key << " " << access(key);
      if (code == MV_OK) out << " current=" << v.nCurValue << " min=" << v.nMin << " max=" << v.nMax << " inc=" << v.nInc;
      else out << " query_error=" << hex_code(code);
      out << '\n';
    }
    for (const auto * key : {"ExposureTime", "Gain", "AcquisitionFrameRate", "ResultingFrameRate"}) {
      MVCC_FLOATVALUE v{};
      const auto code = sdk->MV_CC_GetFloatValue(handle, key, &v);
      out << key << " " << access(key);
      if (code == MV_OK) {
        out << " current=" << v.fCurValue << " min=" << v.fMin << " max=" << v.fMax;
        if (std::isfinite(v.fCurValue) && v.fCurValue > 0 &&
            (std::string(key) == "AcquisitionFrameRate" || std::string(key) == "ResultingFrameRate")) fps = v.fCurValue;
      } else out << " query_error=" << hex_code(code);
      out << '\n';
    }
    for (const auto * key : {"PixelFormat", "ExposureAuto", "GainAuto", "BalanceWhiteAuto", "TriggerMode", "AcquisitionMode"}) {
      MVCC_ENUMVALUE v{};
      const auto code = sdk->MV_CC_GetEnumValue(handle, key, &v);
      out << key << " " << access(key);
      if (code == MV_OK) {
        MVCC_ENUMENTRY current{};
        current.nValue = v.nCurValue;
        out << " current=";
        if (sdk->MV_CC_GetEnumEntrySymbolic(handle, key, &current) == MV_OK) out << text(current.chSymbolic);
        else out << hex_code(v.nCurValue);
        out << " supported=";
        if (v.nSupportedNum > MV_MAX_XML_SYMBOLIC_NUM) throw std::runtime_error("MVS invalid enum count");
        for (unsigned int i = 0; i < v.nSupportedNum; ++i) {
          MVCC_ENUMENTRY entry{};
          entry.nValue = v.nSupportValue[i];
          if (i) out << ',';
          if (sdk->MV_CC_GetEnumEntrySymbolic(handle, key, &entry) == MV_OK) out << text(entry.chSymbolic);
          else out << hex_code(entry.nValue);
        }
      } else out << " query_error=" << hex_code(code);
      out << '\n';
    }
    bool enabled = false;
    const auto code = sdk->MV_CC_GetBoolValue(handle, "AcquisitionFrameRateEnable", &enabled);
    out << "AcquisitionFrameRateEnable " << access("AcquisitionFrameRateEnable");
    if (code == MV_OK) out << " current=" << enabled;
    else out << " query_error=" << hex_code(code);
    out << '\n';
    report += out.str();
  }

  bool read(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp)
  {
    if (!grabbing) { error = "MVS acquisition is not running"; return false; }
    MV_FRAME_OUT raw{};
    const auto code = sdk->MV_CC_GetImageBuffer(handle, &raw, timeout_ms);
    if (code != MV_OK) {
      error = "MVS MV_CC_GetImageBuffer failed: " + hex_code(code) +
              " timeout_ms=" + std::to_string(timeout_ms) +
              "; check camera connection, USB permissions, trigger and exposure";
      return false;
    }
    timestamp = std::chrono::steady_clock::now();
    // 无论分配/转换是否抛异常，已借出的 SDK 缓冲区必须归还。
    struct Buffer
    {
      Impl & owner;
      MV_FRAME_OUT & frame;
      bool released = false;
      ~Buffer() { if (!released) (void)owner.sdk->MV_CC_FreeImageBuffer(owner.handle, &frame); }
      void release() {
        const int result = owner.sdk->MV_CC_FreeImageBuffer(owner.handle, &frame);
        released = true;
        check(result, "MV_CC_FreeImageBuffer");
      }
    } buffer{*this, raw};
    try {
      const auto & info = raw.stFrameInfo;
      const unsigned int width = info.nExtendWidth ? info.nExtendWidth : info.nWidth;
      const unsigned int height = info.nExtendHeight ? info.nExtendHeight : info.nHeight;
      const auto pixels = static_cast<std::uint64_t>(width) * height;
      if (!raw.pBufAddr || width == 0 || height == 0 || info.nFrameLen == 0 ||
          info.nLostPacket != 0 || pixels > (256 * 1024 * 1024ULL) / 3) {
        throw std::runtime_error("MVS invalid/incomplete frame metadata or image exceeds 256 MiB");
      }
      const auto bytes = pixels * 3;
      cv::Mat next(static_cast<int>(height), static_cast<int>(width), CV_8UC3);
      if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
        if (info.nFrameLen < bytes) throw std::runtime_error("MVS truncated BGR frame");
        std::memcpy(next.data, raw.pBufAddr, static_cast<std::size_t>(bytes));
      } else {
        MV_CC_PIXEL_CONVERT_PARAM_EX conversion{};
        conversion.nWidth = width;
        conversion.nHeight = height;
        conversion.enSrcPixelType = info.enPixelType;
        conversion.pSrcData = raw.pBufAddr;
        conversion.nSrcDataLen = info.nFrameLen;
        conversion.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        conversion.pDstBuffer = next.data;
        conversion.nDstBufferSize = static_cast<unsigned int>(bytes);
        check(sdk->MV_CC_ConvertPixelTypeEx(handle, &conversion),
              "MV_CC_ConvertPixelTypeEx source=" + hex_code(info.enPixelType));
        if (conversion.nDstLen != bytes) throw std::runtime_error("MVS unexpected converted image length");
      }
      buffer.release();
      image = std::move(next);
      error.clear();
      return true;
    } catch (const std::exception & failure) {
      error = failure.what();
      if (!buffer.released) {
        try { buffer.release(); }
        catch (const std::exception & cleanup) { error += "; " + std::string(cleanup.what()); }
      }
      return false;
    }
  }
};

bool mvs_compiled() noexcept { return true; }

std::string list_mvs_cameras(const std::string & path)
{
  const auto sdk = load_sdk(path);
  const auto devices = sdk->enumerate();
  std::string report = "MVS sdk=" + sdk->version + " library=" + sdk->loaded_path +
                       " usb_device_count=" + std::to_string(devices.size()) + "\n";
  for (std::size_t i = 0; i < devices.size(); ++i) report += device_description(devices[i], i) + "\n";
  return report;
}

MvsCamera::MvsCamera(const MvsCameraConfig & config, int width, int height, double fps, bool start)
{
  validate_mvs_config(config);
  if (width < 0 || height < 0 || fps < 0 || !std::isfinite(fps)) throw std::invalid_argument("invalid MVS dimensions/fps");
  auto impl = std::make_unique<Impl>(config);
  auto devices = impl->sdk->enumerate();
  int selected = config.device_index;
  if (!config.serial_number.empty()) {
    for (std::size_t i = 0; i < devices.size(); ++i) {
      if (text(devices[i].SpecialInfo.stUsb3VInfo.chSerialNumber) == config.serial_number) {
        if (selected != -1) throw std::runtime_error("MVS serial number is ambiguous");
        selected = static_cast<int>(i);
      }
    }
  } else if (selected == -1 && devices.size() == 1) selected = 0;
  if (selected < 0 || static_cast<std::size_t>(selected) >= devices.size()) {
    throw std::runtime_error("MVS USB camera selection failed (count=" + std::to_string(devices.size()) +
      "); use --list-cameras, then --mvs-serial or --mvs-index; check USB permissions if lsusb sees the camera");
  }
  impl->report = "MVS sdk=" + impl->sdk->version + " library=" + impl->sdk->loaded_path + "\n" +
                 device_description(devices[selected], selected) + "\n";
  check(impl->sdk->MV_CC_CreateHandle(&impl->handle, &devices[selected]), "MV_CC_CreateHandle");
  check(impl->sdk->MV_CC_OpenDevice(impl->handle, MV_ACCESS_Exclusive, 0),
        "MV_CC_OpenDevice (check USB permissions or another process owning the camera)");
  impl->opened = true;
  // --camera-info 只查询，不修改节点，不启动取流。
  if (start) {
    impl->set_enum("AcquisitionMode", "Continuous");
    impl->set_enum("TriggerMode", "Off");
    impl->set_integer("Width", width);
    impl->set_integer("Height", height);
    if (!config.pixel_format.empty()) impl->set_enum("PixelFormat", config.pixel_format);
    if (config.exposure_us) { impl->set_enum("ExposureAuto", "Off"); impl->set_float("ExposureTime", *config.exposure_us); }
    if (config.gain) { impl->set_enum("GainAuto", "Off"); impl->set_float("Gain", *config.gain); }
    if (fps > 0) {
      if (impl->access("AcquisitionFrameRateEnable") != "RW") throw std::runtime_error("MVS AcquisitionFrameRateEnable is not writable");
      check(impl->sdk->MV_CC_SetBoolValue(impl->handle, "AcquisitionFrameRateEnable", true), "enable frame rate");
      bool enabled = false;
      check(impl->sdk->MV_CC_GetBoolValue(impl->handle, "AcquisitionFrameRateEnable", &enabled), "read back frame rate enable");
      if (!enabled) throw std::runtime_error("MVS frame rate enable readback mismatch");
      impl->set_float("AcquisitionFrameRate", fps);
    }
  }
  impl->describe_parameters();
  if (start) {
    // 参考头文件将 SetGrabStrategy 标注为仅 Windows；Linux 使用一个缓存节点，
    // 不把未知的平台行为作为启动前提。不在本轮承诺实时最新帧/自动重连。
    check(impl->sdk->MV_CC_SetImageNodeNum(impl->handle, 1), "MV_CC_SetImageNodeNum");
    check(impl->sdk->MV_CC_StartGrabbing(impl->handle), "MV_CC_StartGrabbing");
    impl->grabbing = true;
  }
  impl_ = std::move(impl);
}

#else
struct MvsCamera::Impl { std::string report; std::string error; double fps = 0; };
namespace
{
[[noreturn]] void unavailable()
{
  throw std::runtime_error("MVS support is unavailable in this build; configure SRM_ENABLE_MVS=ON "
                           "with SRM_MVS_INCLUDE_DIR containing SDK headers, or use image/video input");
}
}
bool mvs_compiled() noexcept { return false; }
std::string list_mvs_cameras(const std::string &) { unavailable(); }
MvsCamera::MvsCamera(const MvsCameraConfig &, int, int, double, bool) { unavailable(); }
#endif

MvsCamera::~MvsCamera() = default;
bool MvsCamera::read(cv::Mat & image, std::chrono::steady_clock::time_point & timestamp)
{
#ifdef SRM_HAVE_MVS
  return impl_->read(image, timestamp);
#else
  (void)image; (void)timestamp;
  return false;
#endif
}
const std::string & MvsCamera::report() const noexcept { return impl_->report; }
const std::string & MvsCamera::last_error() const noexcept { return impl_->error; }
double MvsCamera::nominal_fps() const noexcept { return impl_->fps; }

}  // namespace srm::io
