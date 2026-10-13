// 故意只有版本接口，验证 loader 在进入 SDK 生命周期前拒绝缺失符号。
extern "C" unsigned int MV_CC_GetSDKVersion() { return 0x04070003; }
