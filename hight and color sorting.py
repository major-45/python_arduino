import cv2
import numpy as np
import math
import serial
import serial.tools.list_ports
import time
 
# ==============================================================================
# USER INPUTS
# ==============================================================================
print("="*70)
print("TOP-VIEW CAMERA SETUP")
print("="*70)
CAMERA_HEIGHT   = float(input("  Camera height above ground (metres)          : "))
BOX_HEIGHT      = float(input("  Object / box height above ground (metres)    : "))
PIXELS_PER_CM   = float(input("  Top-view pixels per cm at image centre       : "))
FOCAL_LENGTH_MM = float(input("  Top-view camera focal length (mm)            : "))
 
print()
print("="*70)
print("FRONT-VIEW CAMERA SETUP")
print("="*70)
FRONT_PIXELS_PER_CM  = float(input("  Front-view pixels per cm                     : "))
FRONT_AVG_DISTANCE_M = float(input("  Average distance from camera to objects (m)  : "))
 
print()
print("="*70)
print("  TOP-VIEW")
print(f"    Camera height      : {CAMERA_HEIGHT} m")
print(f"    Box height         : {BOX_HEIGHT} m")
print(f"    Pixels/cm          : {PIXELS_PER_CM} px/cm")
print(f"    Focal length       : {FOCAL_LENGTH_MM} mm")
print("  FRONT-VIEW")
print(f"    Pixels/cm          : {FRONT_PIXELS_PER_CM} px/cm")
print(f"    Avg object distance: {FRONT_AVG_DISTANCE_M} m")
print("="*70 + "\n")
 
# ==============================================================================
# IMAGE PATHS
# ==============================================================================
TOP_IMAGE_PATH   = r"C:\Users\USER\Downloads\20260403233342.jpg"
FRONT_IMAGE_PATH = r"C:\Users\USER\Downloads\20260403233802.jpg"

 
# ==============================================================================
# SHARED CONSTANTS
# ==============================================================================
SENSOR_WIDTH_MM  = 3.6
SENSOR_HEIGHT_MM = 2.7
 
DISTORTION_COEFFS         = np.array([-0.3, 0.1, 0.0, 0.0, 0.0])
USE_DISTORTION_CORRECTION = True
 
# ==============================================================================
# FIXED PICK Z — claw lowers to this Z to grip top of box
# ==============================================================================
PICK_Z_FIXED = 45.0
 
# ==============================================================================
# HSV COLOR RANGES
# ==============================================================================
color_ranges = {
    'Red': [
        (np.array([0,   25,  80]),  np.array([15,  200, 240])),
        (np.array([155, 25,  80]),  np.array([180, 200, 240]))
    ],
    'Green': [
        (np.array([55,  40,  60]),  np.array([85,  130, 180]))
    ],
    'Blue': [
        (np.array([100, 60,  80]),  np.array([125, 150, 160]))
    ]
}

# ==============================================================================
# MINIMUM OBJECT SIZE
# ==============================================================================
MIN_OBJECT_AREA = 1200
MIN_OBJECT_W    = 28
MIN_OBJECT_H    = 28
MIN_ASPECT      = 0.25
MAX_ASPECT      = 4.0
 
box_colors = {'Red': (0, 0, 255), 'Green': (0, 255, 0), 'Blue': (255, 0, 0)}
TEXT_COLOR  = (0, 0, 0)
 
# ==============================================================================
# SERIAL CONFIGURATION
# ==============================================================================
BAUD_RATE = 115200
 
# ==============================================================================
# GANTRY WORKING AREA (mm)
# ==============================================================================
GANTRY_X_MAX          =  210.0
GANTRY_X_MIN          =    0.0
GANTRY_Y_MAX          =  250.0
GANTRY_Y_MIN          =    0.0
GANTRY_Z_TOP          =   50.0
GANTRY_Z_BOTTOM       =    0.0
CAMERA_VIEW_WIDTH_MM  =  210.0   # maps exactly to GANTRY_X_MAX
CAMERA_VIEW_HEIGHT_MM =  250.0   # maps exactly to GANTRY_Y_MAX
 
# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================
def build_camera_matrix(focal_px, img_width, img_height):
    cx = img_width  / 2.0
    cy = img_height / 2.0
    K  = np.array([
        [focal_px, 0,        cx],
        [0,        focal_px, cy],
        [0,        0,         1]
    ], dtype=np.float32)
    return K, cx, cy
 
def apply_distortion_correction(image, camera_matrix, dist_coeffs):
    return cv2.undistort(image, camera_matrix, dist_coeffs)
 
# ==============================================================================
# DETECT COLOR OBJECTS
# ==============================================================================
def detect_color_objects(image):
    hsv    = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    kernel = np.ones((7, 7), np.uint8)
    results = {}
 
    for color_name, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
 
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.GaussianBlur(mask, (7, 7), 0)
 
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        valid = []
        for c in contours:
            area       = cv2.contourArea(c)
            x, y, w, h = cv2.boundingRect(c)
 
            if area < MIN_OBJECT_AREA:                     continue
            if w    < MIN_OBJECT_W:                        continue
            if h    < MIN_OBJECT_H:                        continue
            aspect = w / float(h)
            if aspect < MIN_ASPECT or aspect > MAX_ASPECT: continue
 
            valid.append({'bbox': (x, y, w, h), 'area': area})
 
        if valid:
            valid.sort(key=lambda d: d['area'], reverse=True)
            valid = valid[:3]
            results[color_name] = valid
 
    return results
 
# ==============================================================================
# PIXEL TO WORLD
# ==============================================================================
def pixel_to_world_perspective(pixel_u, pixel_v,
                               focal_px, origin_u, origin_v,
                               camera_height, object_height):
    x_norm  = (pixel_u - origin_u) / focal_px
    y_norm  = (origin_v - pixel_v) / focal_px
    t       = camera_height - object_height
    world_x =  x_norm * t
    world_y =  y_norm * t
    return world_x, world_y
 
def pixel_height_to_world(pixel_height, pixels_per_cm):
    return (pixel_height / pixels_per_cm) / 100.0
 
# ==============================================================================
# COORDINATE MAPPING
# ==============================================================================
def opencv_to_gantry_mm(world_x_m, world_y_m):
    gantry_x = world_x_m * 1000.0
    gantry_y = world_y_m * 1000.0
    gantry_x = max(GANTRY_X_MIN, min(gantry_x, GANTRY_X_MAX))
    gantry_y = max(GANTRY_Y_MIN, min(gantry_y, GANTRY_Y_MAX))
    return round(gantry_x, 2), round(gantry_y, 2)
 
# ==============================================================================
# COORDINATE MAPPING VERIFICATION
# Confirms that the camera field of view maps 1:1 to the gantry working area.
# Bottom-left of image = gantry home (0,0), top-right = (210,250) mm.
# ==============================================================================
def verify_mapping():
    print("\n" + "="*70)
    print("COORDINATE MAPPING VERIFICATION")
    print("="*70)
    print("Expected results:")
    print("  Camera bottom-left  → Gantry (  0,   0)  [you standing here]")
    print("  Camera bottom-right → Gantry (210,   0)")
    print("  Camera top-left     → Gantry (  0, 250)  [far end]")
    print("  Camera top-right    → Gantry (210, 250)")
    print("  Camera centre       → Gantry (105, 125)")
    print()
 
    img_w = CAMERA_VIEW_WIDTH_MM  / 1000.0
    img_h = CAMERA_VIEW_HEIGHT_MM / 1000.0
 
    test_points = [
        ("Camera bottom-left  → Gantry (  0,   0)",  0.0,     0.0    ),
        ("Camera bottom-right → Gantry (210,   0)",  img_w,   0.0    ),
        ("Camera top-left     → Gantry (  0, 250)",  0.0,     img_h  ),
        ("Camera top-right    → Gantry (210, 250)",  img_w,   img_h  ),
        ("Camera centre       → Gantry (105, 125)",  img_w/2, img_h/2),
    ]
 
    print(f"{'Point':<45} {'OpenCV(m)':>20} {'Gantry(mm)':>15}")
    print("-"*82)
    for label, ox, oy in test_points:
        gx, gy = opencv_to_gantry_mm(ox, oy)
        print(f"{label:<45} ({ox:+.4f},{oy:+.4f})m   ({gx:>6.1f},{gy:>6.1f})mm")
    print("="*70)
 
# ==============================================================================
# BUILD OBJECT LIST
# Sends actual object_height_mm in the Z field so Arduino can sort
# same-colour objects by height ascending within each colour group.
# ==============================================================================
def build_object_list(top_results, front_results):
    all_colors  = sorted(set(list(top_results.keys()) + list(front_results.keys())))
    object_list = []
 
    print("\n" + "="*75)
    print("GANTRY COORDINATE TABLE")
    print("="*75)
    print(f"{'#':<4} {'Color':<8} {'X(mm)':>8} {'Y(mm)':>8} "
          f"{'ObjH(mm)':>10} {'PickZ(mm)':>10}")
    print("-"*55)
 
    for color_name in all_colors:
        top_objs   = top_results.get(color_name,   [])
        front_objs = front_results.get(color_name, [])
        max_count  = max(len(top_objs), len(front_objs))
 
        for i in range(max_count):
            top_obj   = top_objs[i]   if i < len(top_objs)   else None
            front_obj = front_objs[i] if i < len(front_objs) else None
 
            if top_obj is None or front_obj is None:
                print(f"  WARNING: {color_name} #{i+1} missing in one view - skipping")
                continue
 
            gantry_x, gantry_y = opencv_to_gantry_mm(
                top_obj['world_x'],
                top_obj['world_y']
            )
 
            object_height_mm = round(front_obj['real_height_m'] * 1000.0, 2)

            object_list.append({
                "x"        : gantry_x,
                "y"        : gantry_y,
                "z"        : object_height_mm,   # Arduino reads into height_mm for sorting
                "height_mm": object_height_mm,
                "color"    : color_name.strip().upper()
            })
 
            print(f"{len(object_list):<4} {color_name.upper():<8} "
                  f"{gantry_x:>8} {gantry_y:>8} "
                  f"{object_height_mm:>10} {PICK_Z_FIXED:>10}")
 
    print("="*75)
    return object_list
 
# ==============================================================================
# AUTO DETECT ARDUINO PORT
# ==============================================================================
def find_arduino_port():
    print("\nScanning for Arduino...")
    ports = serial.tools.list_ports.comports()
    arduino_port = None
    for port in ports:
        print(f"  Found: {port.device} - {port.description}")
        if any(keyword in port.description
               for keyword in ['Arduino', 'CH340', 'CH341',
                                'FTDI', 'USB Serial', 'Mega']):
            arduino_port = port.device
            print(f"  Arduino detected on {arduino_port}!")
            break
    if arduino_port is None:
        print("\nCould not auto-detect Arduino.")
        for port in ports:
            print(f"  {port.device} - {port.description}")
        arduino_port = input("\nEnter COM port manually (e.g. COM13): ").strip()
    return arduino_port
 
# ==============================================================================
# SEND TO ARDUINO
# ==============================================================================
def send_to_arduino(object_list, port):
    if not object_list:
        print("ERROR: No valid objects to send!")
        return
 
    print("\n" + "="*60)
    print("DATA TO BE SENT TO ARDUINO:")
    print("="*60)
    for i, obj in enumerate(object_list):
        x_str = str(round(float(obj['x']),  2))
        y_str = str(round(float(obj['y']),  2))
        z_str = str(round(float(obj['z']),  2))
        c_str = str(obj['color']).strip().upper()
        print(f"  [{i+1}] {x_str},{y_str},{z_str},{c_str}")
    print("="*60)
 
    print(f"\nConnecting to Arduino on {port} at {BAUD_RATE} baud...")
 
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
 
        ser.dtr = False
        time.sleep(0.1)
        ser.dtr = True
        time.sleep(0.1)
 
        ser.reset_input_buffer()
        ser.reset_output_buffer()
 
        print("Waiting for READY signal (up to 30 seconds)...")
        ready_received = False
        start_time = time.time()
 
        while time.time() - start_time < 30:
            if ser.in_waiting > 0:
                raw  = ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
                line = line.replace('\r', '').replace('\n', '').strip()
                print(f"  Arduino: [{line}]")
                if line == "READY":
                    ready_received = True
                    break
            time.sleep(0.05)
 
        if not ready_received:
            print("READY not received. Sending newline to wake Arduino...")
            ser.write(b"\n")
            time.sleep(2)
            ser.reset_input_buffer()
            for _ in range(10):
                if ser.in_waiting > 0:
                    raw  = ser.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()
                    line = line.replace('\r', '').replace('\n', '').strip()
                    print(f"  Arduino: [{line}]")
                    if line == "READY":
                        ready_received = True
                        break
                time.sleep(0.5)
 
        if not ready_received:
            print("\nERROR: Never received READY!")
            print("  1. Close Arduino IDE Serial Monitor")
            print("  2. Re-upload Arduino code")
            print("  3. Check baud rate is 115200")
            ser.close()
            return
 
        print("\nREADY received! Sending objects...\n")
        time.sleep(0.2)
 
        error_occurred = False
        for i, obj in enumerate(object_list):
            x_str = str(round(float(obj['x']), 2))
            y_str = str(round(float(obj['y']), 2))
            z_str = str(round(float(obj['z']), 2))
            c_str = str(obj['color']).strip().upper()
 
            data  = f"{x_str},{y_str},{z_str},{c_str}\n"
            data  = data.replace('\r', '').replace(' ', '')
 
            ser.write(data.encode('utf-8'))
            print(f"  Sent [{i+1}/{len(object_list)}]: {data.strip()}")
 
            ack   = ""
            start = time.time()
            while time.time() - start < 5:
                if ser.in_waiting > 0:
                    raw = ser.readline()
                    ack = raw.decode('utf-8', errors='ignore').strip()
                    ack = ack.replace('\r', '').replace('\n', '').strip()
                    if "Stored" in ack or "ERROR" in ack:
                        break
                time.sleep(0.05)
 
            print(f"  Arduino: [{ack}]")
 
            if "ERROR" in ack:
                print(f"\nERROR from Arduino on object {i+1}!")
                print(f"  Data sent: {data.strip()}")
                print("  Stopping.")
                error_occurred = True
                break
 
            time.sleep(0.15)
 
        if error_occurred:
            ser.close()
            return
 
        print("\n" + "="*50)
        print("All objects sent. Sending START...")
        print("="*50)
        ser.write(b"START\n")
 
        print("\n--- Arduino pick and place running ---")
        done_timeout = time.time()
        while True:
            if time.time() - done_timeout > 300:
                print("ERROR: Timeout waiting for DONE!")
                break
            if ser.in_waiting > 0:
                raw  = ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
                line = line.replace('\r', '').replace('\n', '').strip()
                if line:
                    print(f"  Arduino: {line}")
                if line == "DONE":
                    print("\n" + "="*50)
                    print("All objects picked and placed!")
                    print("="*50)
                    break
            time.sleep(0.05)
 
        ser.close()
        print("Serial connection closed.")
 
    except serial.SerialException as e:
        print(f"\nSerial error: {e}")
        print("  1. Close Arduino IDE Serial Monitor")
        print("  2. Check COM port in Device Manager")
        print("  3. Unplug and replug USB cable")
 
# ==============================================================================
# TOP-VIEW PROCESSING
# ==============================================================================
print("="*70)
print("TOP-VIEW PROCESSING")
print("="*70)
 
top_image = cv2.imread(TOP_IMAGE_PATH)
if top_image is None:
    print(f"ERROR: Could not load top-view image: {TOP_IMAGE_PATH}")
    exit()
 
top_img_h, top_img_w = top_image.shape[:2]
print(f"Top-view image: {top_img_w} x {top_img_h} px")
 
focal_sensor_x      = (FOCAL_LENGTH_MM * top_img_w)  / SENSOR_WIDTH_MM
focal_sensor_y      = (FOCAL_LENGTH_MM * top_img_h)  / SENSOR_HEIGHT_MM
focal_sensor        = (focal_sensor_x + focal_sensor_y) / 2.0
EFFECTIVE_HEIGHT_CM = (CAMERA_HEIGHT - BOX_HEIGHT) * 100.0
FOCAL_PX            = PIXELS_PER_CM * EFFECTIVE_HEIGHT_CM
 
print(f"Focal length (sensor-derived)  : {focal_sensor:.2f} px")
print(f"Focal length (px/cm empirical) : {FOCAL_PX:.2f} px")
diff_pct = abs(FOCAL_PX - focal_sensor) / focal_sensor * 100
print(f"Difference                     : {diff_pct:.1f}%")
 
top_cam_matrix, top_cx, top_cy = build_camera_matrix(FOCAL_PX, top_img_w, top_img_h)
 
if USE_DISTORTION_CORRECTION:
    print("Applying distortion correction...")
    top_working = apply_distortion_correction(top_image, top_cam_matrix, DISTORTION_COEFFS)
else:
    top_working = top_image.copy()
 
top_annotated = top_working.copy()
 
origin_u = 0.0
origin_v = float(top_img_h)
 
ox = 30
oy = int(origin_v) - 30
cv2.circle(top_annotated, (ox, oy), 10, (255, 255, 255), -1)
cv2.circle(top_annotated, (ox, oy), 12, TEXT_COLOR, 2)
cv2.putText(top_annotated, "Origin (0,0) YOU",
            (ox+15, oy-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 2)
axis_len = 60
cv2.arrowedLine(top_annotated, (ox, oy), (ox+axis_len, oy),
                TEXT_COLOR, 2, tipLength=0.3)
cv2.arrowedLine(top_annotated, (ox, oy), (ox, oy-axis_len),
                TEXT_COLOR, 2, tipLength=0.3)
cv2.putText(top_annotated, "+X right",
            (ox+axis_len+5, oy+5),  cv2.FONT_HERSHEY_SIMPLEX, 0.4, TEXT_COLOR, 2)
cv2.putText(top_annotated, "+Y away",
            (ox+5, oy-axis_len-5),  cv2.FONT_HERSHEY_SIMPLEX, 0.4, TEXT_COLOR, 2)
 
print("\nDetecting objects in top-view...")
top_detections_raw = detect_color_objects(top_working)
top_results        = {}
 
print(f"\n{'─'*70}")
print("TOP-VIEW DETECTIONS")
print(f"{'─'*70}")
 
for color_name, objs in top_detections_raw.items():
    top_results[color_name] = []
    print(f"\n{color_name}: {len(objs)} object(s)")
    for idx, obj in enumerate(objs):
        x, y, w, h = obj['bbox']
        center_u   = x + w / 2.0
        center_v   = y + h / 2.0
        world_x, world_y = pixel_to_world_perspective(
            center_u, center_v,
            FOCAL_PX, origin_u, origin_v,
            CAMERA_HEIGHT, BOX_HEIGHT
        )
        top_results[color_name].append({
            'idx'         : idx + 1,
            'pixel_center': (int(center_u), int(center_v)),
            'world_x'     : world_x,
            'world_y'     : world_y,
            'bbox'        : (x, y, w, h)
        })
        gx, gy = opencv_to_gantry_mm(world_x, world_y)
        cv2.rectangle(top_annotated, (x, y), (x+w, y+h), box_colors[color_name], 2)
        cv2.circle(top_annotated, (int(center_u), int(center_v)),
                   5, box_colors[color_name], -1)
        cv2.circle(top_annotated, (int(center_u), int(center_v)),
                   7, (255, 255, 255), 2)
        y_off = y - 10
        cv2.putText(top_annotated, f"{color_name} #{idx+1}  Area:{obj['area']:.0f}px",
                    (x, y_off),    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_COLOR, 2)
        cv2.putText(top_annotated, f"Gantry:({gx},{gy})mm",
                    (x, y_off-18), cv2.FONT_HERSHEY_SIMPLEX, 0.4,  TEXT_COLOR, 2)
        cv2.putText(top_annotated, f"World:({world_x:.3f},{world_y:.3f})m",
                    (x, y_off-33), cv2.FONT_HERSHEY_SIMPLEX, 0.4,  TEXT_COLOR, 2)
        dist = math.sqrt(world_x**2 + world_y**2)
        print(f"  #{idx+1}  Pixel:({int(center_u)},{int(center_v)})  "
              f"Area:{obj['area']:.0f}px  "
              f"World:({world_x:.4f}m,{world_y:.4f}m)  "
              f"Gantry:({gx}mm,{gy}mm)")
 
for i, txt in enumerate([
    f"TOP-VIEW | Calib:{PIXELS_PER_CM}px/cm H={CAMERA_HEIGHT}m F={FOCAL_LENGTH_MM}mm",
    f"Distortion: {'Corrected' if USE_DISTORTION_CORRECTION else 'NOT corrected'}",
    f"Min area:{MIN_OBJECT_AREA}px Min size:{MIN_OBJECT_W}x{MIN_OBJECT_H}px "
    f"Aspect:{MIN_ASPECT}-{MAX_ASPECT}"
]):
    cv2.putText(top_annotated, txt, (10, 25+i*18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, TEXT_COLOR, 1)
 
cv2.imwrite("output_topview.jpg", top_annotated)
cv2.imshow("Top-View -- X,Y Position", top_annotated)
print("\nTop-view output saved: output_topview.jpg")
cv2.waitKey(0)
cv2.destroyAllWindows()
 
# ==============================================================================
# FRONT-VIEW PROCESSING
# ==============================================================================
print("\n" + "="*70)
print("FRONT-VIEW PROCESSING")
print("="*70)
 
front_image = cv2.imread(FRONT_IMAGE_PATH)
if front_image is None:
    print(f"ERROR: Could not load front-view image: {FRONT_IMAGE_PATH}")
    exit()
 
front_img_h, front_img_w = front_image.shape[:2]
print(f"Front-view image: {front_img_w} x {front_img_h} px")
 
FRONT_FOCAL_PX = FRONT_PIXELS_PER_CM * (FRONT_AVG_DISTANCE_M * 100.0)
front_cam_matrix, _, _ = build_camera_matrix(
    FRONT_FOCAL_PX, front_img_w, front_img_h)
 
if USE_DISTORTION_CORRECTION:
    print("Applying distortion correction...")
    front_working = apply_distortion_correction(
        front_image, front_cam_matrix, DISTORTION_COEFFS)
else:
    front_working = front_image.copy()
 
front_annotated      = front_working.copy()
front_detections_raw = detect_color_objects(front_working)
front_results        = {}
 
print(f"\n{'─'*70}")
print("FRONT-VIEW HEIGHT DETECTIONS")
print(f"{'─'*70}")
 
for color_name, objs in front_detections_raw.items():
    front_results[color_name] = []
    print(f"\n{color_name}: {len(objs)} object(s)")
    for idx, obj in enumerate(objs):
        x, y, w, h   = obj['bbox']
        pixel_h       = h
        real_height_m = pixel_height_to_world(pixel_h, FRONT_PIXELS_PER_CM)
        front_results[color_name].append({
            'idx'          : idx + 1,
            'pixel_height' : pixel_h,
            'real_height_m': real_height_m,
            'bbox'         : (x, y, w, h)
        })
        cv2.rectangle(front_annotated, (x, y), (x+w, y+h),
                      box_colors[color_name], 2)
        arrow_x = x + w + 10
        cv2.arrowedLine(front_annotated, (arrow_x, y),   (arrow_x, y+h),
                        box_colors[color_name], 2, tipLength=0.05)
        cv2.arrowedLine(front_annotated, (arrow_x, y+h), (arrow_x, y),
                        box_colors[color_name], 2, tipLength=0.05)
        mid_y  = y + h // 2
        pick_z = PICK_Z_FIXED
        cv2.putText(front_annotated, f"{color_name} #{idx+1}  Area:{obj['area']:.0f}px",
                    (x, y-22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_COLOR, 2)
        cv2.putText(front_annotated,
                    f"H={real_height_m*100:.1f}cm pickZ={pick_z}mm",
                    (x, y-8),  cv2.FONT_HERSHEY_SIMPLEX, 0.4, TEXT_COLOR, 2)
        cv2.putText(front_annotated, f"{real_height_m:.3f}m",
                    (arrow_x+5, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_colors[color_name], 2)
        print(f"  #{idx+1}  Area:{obj['area']:.0f}px  "
              f"h:{pixel_h}px -> {real_height_m*100:.2f}cm  "
              f"pickZ={pick_z}mm")
 
for i, txt in enumerate([
    f"FRONT-VIEW | Calib:{FRONT_PIXELS_PER_CM}px/cm Dist:{FRONT_AVG_DISTANCE_M}m",
    f"Distortion: {'Corrected' if USE_DISTORTION_CORRECTION else 'NOT corrected'}",
    f"Min area:{MIN_OBJECT_AREA}px | pickZ = {PICK_Z_FIXED}mm (fixed)"
]):
    cv2.putText(front_annotated, txt, (10, 25+i*18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, TEXT_COLOR, 1)
 
cv2.imwrite("output_frontview.jpg", front_annotated)
cv2.imshow("Front-View -- Height", front_annotated)
print("\nFront-view output saved: output_frontview.jpg")
cv2.waitKey(0)
cv2.destroyAllWindows()
 
# ==============================================================================
# COMBINED DETECTION TABLE
# ==============================================================================
print("\n" + "="*80)
print("COMBINED DETECTION TABLE")
print("="*80)
print(f"{'Color':<8} {'#':>3}  {'World_X(m)':>12} {'World_Y(m)':>12} "
      f"{'Gantry_X':>10} {'Gantry_Y':>10} {'H(cm)':>8} {'PickZ':>8}")
print("-"*80)
 
all_colors = sorted(set(list(top_results.keys()) + list(front_results.keys())))
for color_name in all_colors:
    top_objs   = top_results.get(color_name,   [])
    front_objs = front_results.get(color_name, [])
    max_count  = max(len(top_objs), len(front_objs))
    for i in range(max_count):
        top_obj   = top_objs[i]   if i < len(top_objs)   else None
        front_obj = front_objs[i] if i < len(front_objs) else None
        wx   = top_obj['world_x']         if top_obj   else float('nan')
        wy   = top_obj['world_y']         if top_obj   else float('nan')
        h_m  = front_obj['real_height_m'] if front_obj else float('nan')
        h_cm = h_m * 100                  if front_obj else float('nan')
        pz   = PICK_Z_FIXED               if front_obj else float('nan')
        gx, gy = opencv_to_gantry_mm(wx, wy) if top_obj else (float('nan'), float('nan'))
        print(f"{color_name:<8} {i+1:>3}  "
              f"{wx:>12.4f} {wy:>12.4f} "
              f"{gx:>10} {gy:>10} {h_cm:>8.2f} {pz:>8}")
 
print("="*80)
 
# ==============================================================================
# MAPPING VERIFICATION
# ==============================================================================
verify_mapping()
 
# ==============================================================================
# SEND TO ARDUINO
# ==============================================================================
print("\n" + "="*70)
print("ARDUINO COMMUNICATION")
print("="*70)
 
confirm = input("\nSend data to Arduino? (yes/no): ").strip().lower()
 
if confirm == "yes":
    arduino_port = find_arduino_port()
    print(f"\nUsing port: {arduino_port}")
    object_list = build_object_list(top_results, front_results)
    if object_list:
        send_to_arduino(object_list, arduino_port)
    else:
        print("No valid objects found to send.")
else:
    print("Skipped. Re-run when ready.")