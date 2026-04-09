# 海康相机ROS2服务接口使用说明

## 概述

本项目为海康相机提供了ROS2服务接口，允许用户通过服务调用来读取和设置相机参数，包括浮点型、整型、字符串型和枚举型参数。

## 支持的服务接口

### 浮点型参数
- `get_float_value`: 读取浮点型参数
- `set_float_value`: 设置浮点型参数

### 整型参数
- `get_integer_value`: 读取整型参数
- `set_integer_value`: 设置整型参数

### 字符串型参数
- `get_string_value`: 读取字符串型参数
- `set_string_value`: 设置字符串型参数

### 枚举型参数
- `get_enum_value`: 读取枚举型参数
- `set_enum_value`: 设置枚举型参数

### 布尔型参数
- `get_bool_value`: 读取布尔型参数
- `set_bool_value`: 设置布尔型参数

### command型参数
- `execute_command`: 执行command类型参数

## 支持的参数

以下是一些常用的相机参数名称：

- `ExposureTime`: 曝光时间（浮点型）
- `Gain`: 增益（浮点型）
- `Width`: 图像宽度（整型）
- `Height`: 图像高度（整型）
- `PixelFormat`: 像素格式（枚举型）
- `TriggerMode`: 触发模式（枚举型）

## 启动节点

### 启动相机发布节点和服务端

```bash
ros2 run mvcc_camera_ros2 hik_camera_image_pub
```

### 启动测试节点

```bash
ros2 launch mvcc_camera_ros2 camera_params_test.xml
```

## 注意事项

1. 在使用服务之前，请确保相机发布节点已经启动并正常运行。
2. 参数名称必须与相机支持的参数名称完全匹配。
3. 在设置参数时，请确保参数值在相机支持的范围内，否则设置会失败。
4. 某些参数可能需要在特定模式下才能设置，例如在触发模式下才能设置曝光时间。

## 故障排除

如果服务调用失败，请检查以下几点：

1. 相机是否已正确连接并初始化
2. 服务端节点是否正在运行
3. 参数名称是否正确
4. 参数值是否在有效范围内
</content>
</write_to_file>
