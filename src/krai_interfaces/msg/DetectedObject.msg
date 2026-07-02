# Satu objek terdeteksi oleh kamera.

string class_name          # "Kotak" atau "Bounding"

int32 x1                   # bounding box top-left x
int32 y1                   # bounding box top-left y
int32 x2                   # bounding box bottom-right x
int32 y2                   # bounding box bottom-right y

int32 center_x             # titik tengah objek x
int32 center_y             # titik tengah objek y

float32 confidence         # confidence score

int32 dx                   # offset X relatif terhadap pusat frame
int32 dy                   # offset Y relatif terhadap pusat frame
float32 distance_cm        # estimasi jarak dalam cm

int32 grid_cell            # posisi grid 1-9 (0 jika tidak ada bounding)
