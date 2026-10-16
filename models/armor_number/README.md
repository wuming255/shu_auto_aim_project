# 装甲内容分类资源

模型和标签直接复用本仓库参考项目 `chenjun_rm_auto_aim-main/armor_detector/model/`，
上游 https://github.com/chenjunnn/rm_auto_aim ，MIT 许可见同目录 `LICENSE`。

`mlp.onnx` SHA256：`b3250176ea8fdbe6d1585be954363757ff6a8245a81e8797f16cea601976d47a`。
输入为透视矫正后 20×28 的 Otsu 二值灰度图，float32、0..1。
只用数字/negative 内容证据排除空隙误配；不继承参考项目数字与装甲大小、目标优先级的对应规则。
模型置信度不是现场识别准确率，也不作为物理装甲唯一身份。
