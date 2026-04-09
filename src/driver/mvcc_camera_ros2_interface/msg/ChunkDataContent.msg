# ChunkDataContent.msg
uint8[] p_chunk_data          # 对应 pChunkData，使用动态数组存储字节数据
uint32 n_chunk_id             # Chunk ID
uint32 n_chunk_len            # Chunk长度（可能冗余，因为数组长度隐含，但保留以兼容原结构）
