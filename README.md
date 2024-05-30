<!--# Kogrob HW Object Tracking-->
<!--Homework for Cognitive Robotics (BMEGEMINMKR) subject of BME/BUTE-->
# Kognitív robotika 
## Object Tracking házi feladat - 2024 tavasz

## A feladat leírása
A választott feladatunk témája emberek vagy tárgyak felismerése és követése klasszikus
vagy neurális hálós algoritmusos képfeldolgozás segítségével TurtleBot3 Burger roboton, 
vagy szimulált ROS/Gazebo környezetben annak virtuális másán ROS segítségéve. 
Mi az utóbbit választottuk. 
Ennek megfelelően a konkrét feladat 
- a megfelelő Gazebo világ elkészítése, illetve
- azon belül a TurtleBot3 Burger robot irányítására olyan program vagy programok írása,
- amely a környezetben felismer egy adott típusú objektumot, és
- kifejezetten a felismert objektumot képes követni, tehát távolságot tartani, 
és a kamerát az objektumon tartani. 

## Megvalósítás
A program két fő osztályt használ fel, a 'Controller' és az 'ImageProcessor' osztályokat; ezek
definíciója a megfelelő 'controller.py' és 'image_processor.py' fájlokban található, a 
'../kogrob_tracking/src' mappában. 



## Telepítés

### Környezet létrehozása
A telepítés feltétele természetesen a ROS és Gazebo programok használata, amelyek valamilyen
Linux operációs rendszerhez kötöttek. Mi a programot WSL segítségével Ubuntu 20.04 alatt futtattuk,
tehát ezt javasoljuk használatra. 
ROS NOETIC, GAZEBO INSTALL PARANCSOK BECOPYZVA IDE

Továbbá szükségünk lesz az ultralytics csomagra a YOLO algoritmushoz, amit felhasználtunk. Ezt az 
alábbi paranccsal tudjuk telepíteni:
```bash
$ !pip install ultralytics
```

### A repo klónozása
Lépjünk a tetszőleges Catkin Workspace-ünkbe, majd hozzunk létre egy kogrob_tracking rospy package-et.
```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg kogrob_tracking rospy
```
Klónozzuk a jelenlegi Github repository-t!
```bash
git clone https://github.com/HorvathBenedek/Kogrob_HW_Tracking.git
```
Majd helyezzük a tartalmát a létrehozott package-be úgy, hogy a package-en belül az
alábbi mappaszerkezetet kapjuk:
```bash
$ tree
├── CMakeLists.txt
├── README.md
├── launch
│   └── kogrob_tracking.launch
├── meshes
│   └── Human.001.stl
├── package.xml
├── src
│   ├── controller.py
│   └── image_processor.py
├── srv
│   └── Detection.srv
├── urdf
│   ├── turtlebot3_burger_for_autorace.gazebo.xacro
│   ├── turtlebot3_burger_for_autorace.urdf.xacro
│   └── turtlebot3_human.urdf.xacro
├── worlds
│   ├── Small_City.world
│   └── polyline.world
└── yolo
    └── yolov5nu.pt
```
A biztonság kedvéért futtasuk a catkin_make-et. 

Ezzel tulajdonképpen a program futtatásra kész. Futtatni az alábbi paranccsal tudjuk:
```bash
$ roslaunch kogrob_tracking kogrob_tracking.launch
```
Amennyiben hibába futunk bele, tegyük futtathatóvá a .py file-okat:
```bash
$ cd ~/catkin_ws/src/kogrob_tracking/src
$ chmod +x controller.py
$ chmod +x image_processor.py
```
<!--Illetve amennyiben szerkesztettük a .py file-okat, érdemes lefuttatni az alábbi parancsokat. 
Ezzek a .py file-okba kerülő esetleges 
```bash
$ 
$ dos2unix controller.py
$ dos2unix image_processor.py
```-->
