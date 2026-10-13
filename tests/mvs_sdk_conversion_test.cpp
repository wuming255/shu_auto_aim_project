// 显式传入真实 SDK 运行库；使用合成像素和未打开设备的句柄，不连接相机。
#include <MvCameraControl.h>
#include <dlfcn.h>
#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
template <typename Function>
Function bind(void * library, const char * name)
{
  auto function = reinterpret_cast<Function>(::dlsym(library, name));
  if (!function) throw std::runtime_error(std::string("missing SDK symbol: ") + name);
  return function;
}

void check(int code, const char * operation)
{
  if (code != MV_OK) {
    std::cerr << operation << " failed: 0x" << std::hex << static_cast<unsigned int>(code) << std::dec << '\n';
    throw std::runtime_error(operation);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) { std::cerr << "Usage: srm_mvs_sdk_conversion_test /path/to/libMvCameraControl.so\n"; return 2; }
  try {
    const auto close_library = [](void * value) { (void)::dlclose(value); };
    std::unique_ptr<void, decltype(close_library)> library(
      ::dlopen(argv[1], RTLD_NOW | RTLD_LOCAL), close_library);
    if (!library) throw std::runtime_error(::dlerror());
    const auto version = bind<decltype(&MV_CC_GetSDKVersion)>(library.get(), "MV_CC_GetSDKVersion")();
    if ((version >> 16) != 0x0407) throw std::runtime_error("use MVS SDK 4.7.x");
    const auto initialize = bind<int (*)()>(library.get(), "MV_CC_Initialize");
    const auto finalize = bind<int (*)()>(library.get(), "MV_CC_Finalize");
    const auto create = bind<decltype(&MV_CC_CreateHandle)>(library.get(), "MV_CC_CreateHandle");
    const auto destroy = bind<decltype(&MV_CC_DestroyHandle)>(library.get(), "MV_CC_DestroyHandle");
    const auto convert = bind<decltype(&MV_CC_ConvertPixelTypeEx)>(library.get(), "MV_CC_ConvertPixelTypeEx");
    check(initialize(), "MV_CC_Initialize");
    struct Session {
      int (*finalize)();
      bool initialized = true;
      ~Session() { if (initialized) (void)finalize(); }
    } session{finalize};
    struct Handle {
      decltype(&MV_CC_DestroyHandle) destroy;
      void * value = nullptr;
      ~Handle() { if (value) (void)destroy(value); }
    } handle{destroy};
    MV_CC_DEVICE_INFO description{};
    description.nTLayerType = MV_USB_DEVICE;
    description.SpecialInfo.stUsb3VInfo.idVendor = 0x2bdf;
    description.SpecialInfo.stUsb3VInfo.idProduct = 1;
    // ConvertPixelTypeEx 需要有效句柄；这里只建立本地对象，不调用 OpenDevice。
    check(create(&handle.value, &description), "MV_CC_CreateHandle(synthetic description)");
    for (const auto type : {PixelType_Gvsp_RGB8_Packed, PixelType_Gvsp_Mono8,
         PixelType_Gvsp_BayerRG8, PixelType_Gvsp_BayerGR8, PixelType_Gvsp_BayerGB8, PixelType_Gvsp_BayerBG8}) {
      constexpr unsigned int width = 8, height = 8;
      std::vector<unsigned char> source(width * height * (type == PixelType_Gvsp_RGB8_Packed ? 3 : 1));
      for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
          if (type == PixelType_Gvsp_RGB8_Packed) {
            source[(y * width + x) * 3] = 10;
            source[(y * width + x) * 3 + 1] = 20;
            source[(y * width + x) * 3 + 2] = 30;
          } else if (type == PixelType_Gvsp_Mono8) source[y * width + x] = 20;
          else {
            const std::array<unsigned char, 4> pattern = type == PixelType_Gvsp_BayerRG8
              ? std::array<unsigned char, 4>{10, 20, 20, 30} : type == PixelType_Gvsp_BayerGR8
              ? std::array<unsigned char, 4>{20, 10, 30, 20} : type == PixelType_Gvsp_BayerGB8
              ? std::array<unsigned char, 4>{20, 30, 10, 20} : std::array<unsigned char, 4>{30, 20, 20, 10};
            source[y * width + x] = pattern[(y % 2) * 2 + x % 2];
          }
        }
      }
      std::vector<unsigned char> output(width * height * 3);
      MV_CC_PIXEL_CONVERT_PARAM_EX parameters{};
      parameters.nWidth = width; parameters.nHeight = height;
      parameters.enSrcPixelType = type; parameters.pSrcData = source.data();
      parameters.nSrcDataLen = static_cast<unsigned int>(source.size());
      parameters.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
      parameters.pDstBuffer = output.data(); parameters.nDstBufferSize = static_cast<unsigned int>(output.size());
      check(convert(handle.value, &parameters), "MV_CC_ConvertPixelTypeEx");
      const auto offset = (3 * width + 3) * 3;
      const unsigned char red = type == PixelType_Gvsp_Mono8 ? 20 : 10;
      const unsigned char blue = type == PixelType_Gvsp_Mono8 ? 20 : 30;
      if (parameters.nDstLen != output.size() || output[offset] != blue ||
          output[offset + 1] != 20 || output[offset + 2] != red) {
        throw std::runtime_error("converted BGR length or color mismatch");
      }
      std::cout << "pixel=0x" << std::hex << type << std::dec << " bytes=" << parameters.nDstLen
                << " BGR=" << int(output[offset]) << ',' << int(output[offset + 1]) << ','
                << int(output[offset + 2]) << " pass=1\n";
    }
    const int destroyed = destroy(handle.value);
    handle.value = nullptr;
    check(destroyed, "MV_CC_DestroyHandle");
    session.initialized = false;
    check(finalize(), "MV_CC_Finalize");
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "srm_mvs_sdk_conversion_test: " << error.what() << '\n';
    return 1;
  }
}
