# 正式自瞄协议实现约束

## 帧格式

每帧为：

```text
body_len: uint16 little-endian
records: body_len bytes
```

没有帧头、帧尾、CRC、时间戳和序列号。每条记录为 `int16 id` 加固定长度数据体，字段均为小端编码。

## 上位机到下位机

固定 18 字节：`body_len=16`，记录顺序必须为 ID 1、ID 2。

```text
ID 1 (int16=1) + yaw(float32) + pitch(float32)
ID 2 (int16=2) + fire_flag(int32)
```

## 下位机到上位机

固定 30 字节：`body_len=28`，记录顺序必须为 ID 1、ID 2。

```text
ID 1 (int16=1) + yaw(float32) + pitch(float32) + roll(float32)
                    + mode(int32) + color(int32)
ID 2 (int16=2) + bullet_speed(float32)
```

## 解析规则

接收端必须维护持久缓存，允许一次读取半帧、整帧或多帧。缓存不足一帧时保留数据；完整帧处理后保留后续数据。长度非法、ID 未知、记录长度不匹配或字段顺序不正确时，由于没有可靠重同步信息，应丢弃当前缓存并等待下一次完整帧。

协议层不得依赖 USB 包边界，也不得添加参考项目旧协议中的 `SP` 帧头或 CRC16。
