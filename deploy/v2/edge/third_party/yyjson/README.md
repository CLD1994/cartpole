# yyjson 来源

本目录固定使用 yyjson 0.13.0，MIT 许可证见 `LICENSE`。

- 上游：https://github.com/ibireme/yyjson
- 固定提交：`6447536015f3d600f3d65323b10976103b337ca7`
- 导入文件：上游 `src/yyjson.c`、`src/yyjson.h`、`LICENSE`，未修改内容。
- 用途：启动时解析小型模型清单，静态编入应用，不要求开发板安装 JSON 动态库。
- 清单的字段、类型及取值限制由 `package.c` 检查，JSON 语法由 yyjson 解析。

## SHA-256

| 文件 | SHA-256 |
|---|---|
| yyjson.c | d2d58ef0a3b2267862dc363832c4f3185fd375aea933d5d3419d1a940d6ece2e |
| yyjson.h | ef803cda5c06b8962face6dfa39c3284b3bcf3e73f4d7317664690cc779679ca |
| LICENSE | 45e384d3d52c73cba3a64d6e6c25d47cd738cd8a55c30629e3201046eda62947 |

升级时从官方固定版本重新获取上述三个文件，更新提交和散列，并运行清单解析集成测试及两种 ARM 构建。
