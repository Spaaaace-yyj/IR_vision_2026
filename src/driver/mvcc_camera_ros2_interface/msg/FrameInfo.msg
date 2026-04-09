# MV_FRAME_OUT Message
sensor_msgs/Image image       #标准图像
std_msgs/Header header        #标准图像附加信息


uint32 n_extend_width          # 图像宽（扩展）
uint32 n_extend_height         # 图像高（扩展）
uint64 n_frame_len_ex          # 帧长度
uint32 en_pixel_type           # 像素格式（用uint32表示枚举，如MvGvspPixelType）
uint32 n_frame_num             # 帧号
uint32 n_dev_timestamp_high    # 设备时间戳高32位
uint32 n_dev_timestamp_low     # 设备时间戳低32位
int64 n_host_timestamp         # 主机时间戳
uint32 n_second_count          # 设备水印时标： 秒数
uint32 n_cycle_count           # 设备水印时标：周期数
uint32 n_cycle_offset          # 设备水印时标：周期偏移量
float32 f_gain                 # 设备水印时标：增益
float32 f_exposure_time        # 设备水印时标：曝光时间
uint32 n_average_brightness    # 设备水印时标：平均亮度
uint32 n_red                   # 设备水印时标：红色白平衡值
uint32 n_green                 # 设备水印时标：绿色白平衡值
uint32 n_blue                  # 设备水印时标：蓝色白平衡值
uint32 n_frame_counter         # 设备水印时标：总帧数
uint32 n_trigger_index         # 设备水印时标：触发计数
uint32 n_input                 # 设备水印时标：输入
uint32 n_output                # 设备水印时标：输出
uint16 n_offset_x              # 设备水印时标：ROI区域 水平偏移量
uint16 n_offset_y              # 设备水印时标：ROI区域 垂直偏移量

uint16 n_chunk_width           # 设备水印时标：ROI区域  Chunk宽
uint16 n_chunk_height          # 设备水印时标：ROI区域  Chunk高
uint32 n_lost_packet           # 本帧丢包数

uint32 n_unparsed_chunk_num    # 未解析的Chunk数量
ChunkDataContent[] unparsed_chunk_content  # 未解析的Chunk数据数组（简化联合体，只保留数据部分）
