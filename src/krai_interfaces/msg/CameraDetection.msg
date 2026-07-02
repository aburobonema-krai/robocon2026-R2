# Hasil deteksi per-frame kamera.

builtin_interfaces/Time stamp

DetectedObject[] objects          # semua objek terdeteksi

int32[] occupied_cells            # cell grid yang terisi (1-9)
int32[9] grid_count               # jumlah objek per cell

bool bounding_detected            # apakah rak (Bounding) terdeteksi
int32 bounding_x1
int32 bounding_y1
int32 bounding_x2
int32 bounding_y2

int32 frame_width
int32 frame_height

float32 fps
