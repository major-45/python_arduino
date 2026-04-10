🤖 Computer Vision Based Color & Height Object Sorter
Hi!! Everyone. This is Wassie. This is my lab group's third year second term project on control system engineering course. The project was on design and implementation of a computer vision based color and height object sorter.
I along with my labmates Sanjay, Nafi, Alauddin and Emon have worked really hard on this project. I hope you guys like it and anyone is welcome to work on this project for any future modification. We have made this gantry system for this traditional project for the first time.

📌 What This Project Does
This system automatically detects objects placed in front of a gantry robot, identifies their color (Red, Green, Blue) and height (taller or shorter), and physically sorts and places them into their correct designated positions — fully automatically, without any human intervention after the initial setup.
A Python script handles all the computer vision work, and an Arduino Mega controls the gantry hardware. They talk to each other over a USB serial connection.

🛠️ Hardware Used

Gantry Robot — 210mm × 250mm working area, built on a RAMPS 1.4 board
Arduino Mega 2560
Stepper Motors — X, Y (dual motor), Z axes
Servo Motor — gripper claw (pin 4)
Two Cameras — one top-view (for X, Y position) and one front-view (for height measurement)
PC — runs the Python vision script


💻 Software & Libraries
Python side:

opencv-python — image capture, HSV color segmentation, contour detection
numpy — array operations
pyserial — serial communication with Arduino

Arduino side:

AccelStepper — trapezoidal motion profiles for all stepper axes
Servo — gripper claw control


⚙️ How It Works
1. Camera Calibration
Before running, you enter your camera parameters (height above ground, pixels per cm, focal length). The system uses these to convert pixel coordinates from the top-view camera into real-world gantry millimetre coordinates. A built-in mapping verification test confirms that the four corners and centre of the camera view map correctly to the gantry working area corners (0,0) to (210,250) mm.
2. Object Detection
Two images are taken — one from the top-view camera and one from the front-view camera. OpenCV converts each frame to HSV colour space and applies colour-specific threshold masks to detect Red, Green, and Blue objects. Morphological filtering and contour analysis clean up the results.
3. Height Measurement
The front-view camera measures each object's pixel height in its bounding box. This is converted to real-world millimetres using the front camera's calibration factor. Each object is classified as either shorter or taller within its colour group.
4. Serial Communication
Python sends each detected object to the Arduino in the format:
X,Y,HEIGHT_MM,COLOR
After all objects are sent, a START command triggers the sorting sequence. The Arduino acknowledges each object and sends DONE when finished.
5. Sorting Algorithm
The Arduino firmware sorts all received objects using a two-key bubble sort:

Primary key: Color priority — RED → GREEN → BLUE
Secondary key: Height ascending within the same color (shorter object placed first)

This guarantees the following drop order and positions every time:
OrderColorHeightDrop Position (mm)1REDShorter(230, 10)2REDTaller(240, 10)3GREENShorter(10, 230)4GREENTaller(10, 240)5BLUEShorter(240, 230)6BLUETaller(240, 240)
6. Collision Avoidance
Since the objects are tall enough that the claw cannot safely travel over them, a path-checking algorithm runs before every move. It tests the L-shaped travel path (Y-move then X-move) against all unplaced object positions using a point-to-segment distance formula. If any object is within a 25 mm safe radius, the claw automatically detours around it via a waypoint inserted at ±25 mm in the X direction, choosing whichever detour is shorter.
7. Pick and Place
For each object the claw follows this sequence:

Raise Z to safe travel height (43 mm)
Navigate to pick position (collision-aware, Y-axis first then X)
Open claw
Lower Z to pick height (5 mm)
Close claw and grip
Raise Z back to travel height
Move to drop zone (Y first, then X)
Lower Z to drop height (12 mm)
Open claw and release
Raise Z and return claw to closed position



🚀 How to Run

Upload gantry_sort.ino to the Arduino Mega using Arduino IDE
Close the Arduino IDE Serial Monitor (or the Python script cannot connect)
Install Python dependencies:


👥 Team
Name            Role      
Wassie           Vision system, Arduino firmware, system integration of arduino and python, CAD design(Outsourced) 
Sanjay           Hardware build, mechanical setup, System intigration of python and srduino code.
Nafi             Pixel calibration, camera view and gantry axis co0rodinate mapping and initial stepper and servo motorrunning test and claw assemble.
Alauddin         Hardware assembling and testing and debugging
Emon             Documentation testing and hardware setup.

📬 Contributions & Future Work
Anyone is welcome to fork this repository and build on it. Some ideas for future improvements:

(i) Live video feed instead of static images.
(ii) Support for more than two height classes per color. For lack of our gantry space, we actually have made a small protype of our main idea. If the gantry space can be made broader, the project can work for multiple heights. Just the the drop zones has to be newly assigned in the source code.
(iii) Automatic camera calibration using a calibration target
(iv)Web dashboard to monitor gantry status in real time.

📄 License
This project is open for academic and personal use. If you use or build upon it, a credit to the original team would be appreciated. 🙂

Bangladesh University of Engineering and Technology — Department of Electrical and Electronic Engineering
EEE 318 — Control System Engineering Lab, 3rd Year 2nd Term
