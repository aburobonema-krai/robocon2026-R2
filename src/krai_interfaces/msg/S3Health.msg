# Health/status packet from ESP32-S3.

builtin_interfaces/Time stamp

uint32 s3_time_ms

uint8 mode
uint8 heading_ready
uint8 base_connected
uint8 primitive_state

uint32 bad_bno_frames
uint32 yaw_jump_reject_count
uint32 base_crc_error
uint32 nuc_rx_count
uint32 nuc_crc_error

uint16 last_nuc_cmd_age_ms
uint16 last_base_age_ms

uint16 fault_flags
