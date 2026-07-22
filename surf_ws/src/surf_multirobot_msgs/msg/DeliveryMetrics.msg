# Receiver-side measurements, correlated with PipelineMetrics by source/epoch/version/class.
std_msgs/Header header
string source_id
uint64 map_epoch
uint64 version
uint8 traffic_class
uint32 wire_bytes
uint32 voxel_count
float32 transport_latency_ms
float32 decode_latency_ms
float32 end_to_end_latency_ms
bool accepted
string rejection_reason
