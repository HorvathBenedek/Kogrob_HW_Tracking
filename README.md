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

A programot a `kogrob_tracking.launch` file futtatásával indíthatjuk, ez tartalmazza a szükséges komponenseket. 

### A Gazebo szimuláció felépítése

A szimuláció a `Small_City.world` Gazebo világban fut.
```xml
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find kogrob_tracking)/launch/Small_City.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
```
Tartalmaz a szimuláció továbbá két TurtleBot3 irányítású modellt. Az egyik értelemszerűen a robotunk; a 
másik egy kézzel irányítható emberfigura. Ezzel demonstrálható, hogy a program sikeresen végzi a követési
feladatot. 

```xml
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find kogrob_tracking)/urdf/turtlebot3_human.urdf.xacro" />
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_human -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
  
  <group ns = "follower">
    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find kogrob_tracking)/urdf/turtlebot3_burger_for_autorace.urdf.xacro" />
    <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model follower_turtlebot3_burger -x $(arg follower_x_pos) -y $(arg follower_y_pos) -z $(arg  follower_z_pos) -param /follower/robot_description" />
  </group>
```
A robot viselkedését a `controller.py` és `image_processor.py` kódok határozzák meg, melyik
a `../kogrob_tracking/src` mappában találhatók. 


Ezek a programok párhuzamosan futnak; a `Controller` felelős a robot mozgatásáért, míg az `ImageProcessor` a
YOLOv5 algoritmus segítségével dolgozza fel a kamerából bejövő jelet. A két szálat a 
`../kogrob_tracking/srv/Detection.srv` ROS szerver kapcsolja össze, ez tartalmazza a ROS
node-ok közötti üzenetek formátumát. 

<!--A robot viselkedését két osztály határozza meg, a `Controller` illetve az `ImageProcessor`; ezek
definíciója a megfelelő `controller.py` és `image_processor.py` fájlokban található, a 
`../kogrob_tracking/src` mappában. 
Ezek a programok párhuzamosan futnak; a `Controller` felelős a robot mozgatásáért, míg az `ImageProcessor` a
YOLOv5 algoritmus segítségével dolgozza fel a kamerából bejövő jelet. A két szálat a 
`../kogrob_tracking/srv/Detection.srv` ROS szerver kapcsolja össze, ez tartalmazza a ROS
node-ok közötti üzenetek formátumát. -->

### A robot viselkedésének áttekintése

A robot viselkedését az alábbi gráffal lehet szemléltetni

GRÁF!

Lényegében a robot viselkedése két szálon fut - a fent említett `controller.py` és `image_processor.py`
fájlokban definiált `Controller` és `ImageProcessor` osztályok. 
- Az `ImageProcessor` a Gazebo szimuláció kamera objektumának képét veszi, ezt eltárolja a `self.image_np`-ben
- Eközben a `Controller` érzékelési request-et küld az `ImageProcessor`-nak.
- Az `ImageProcessor` a requestet feldolgozandó az eltárolt kameraképet beküldi a YOLOv5 neurális hálós
szegmentálási és osztályozási algoritmusba. 
- Az `ImageProcessor` visszaküldi a beazonosított objektumok közül a legvalószínűbb helyét és méretét.
- A `Controller` ennek megfelelően igazítja a sebességét és szögsebességét
- Végül pedig a `Controller` a Gazebo szimulációnak `Twist` üzenet formájában elküldi az elvárt mozgási és 
forgási sebességeket. 

### A kód magyarázata

_Megjegyzés: A robot működését vezérlő kód bemutatásánál a lényegretörőség érdekében csak a fontosabb részleteket emelnénk 
ki. A kivágott kódrészletek helyét `##[...]` komment jelöli._

A rospy node-ok és az osztályok inicializálása után a osztályt a `Controller.run()` függvénnyel indíthatjuk el 
az osztályok együttes működését. Az  `ImageProcessor` osztálynak nincs `run()` függvénye, annak vezérlését a 
`Controller` végzi. 
```python
##controller.py
if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    controller = Controller()
    controller.run()

##image_processor.py
if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)
    rospy.on_shutdown(cv2.destroyAllWindows)
    image_processor = ImageProcessor()
    rospy.spin()
```

A `Controller()` osztály működése:
Inicializáló függvény:
```python
class Controller:
    def __init__(self) -> None:
        self.move = Twist()
        self.freeze = Twist()
        self.cmd_publisher = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=100)
        ##[...]
        rospy.wait_for_service('detection')
        self.detection = rospy.ServiceProxy('detection', Detection)
```
Itt 
- létrehozunk egy publisher-t `cmd_vel` topichoz, ahova `Twist` üzeneteket fogunk küldeni, ezzel 
irányítjuk a robotot.
- Várunk an `ImageProcessor` osztályra, hogy létrehozzon egy `rospy.Service` kiszolgálót. Err 
- Továbbá létrehozunk egy `ServiceProxy`-t.

A  `rospy.Service` és  `rospy.ServiceProxy` használatáról lejjebb lesz szó. 

Az `ImageProcessor` osztály működése: 

Inicializáló függvény:
```python
class ImageProcessor:
    def __init__(self) -> None:
        self.image_msg = Image() # Image message
        ##[...]
        self.camera_subscriber = rospy.Subscriber("/follower/camera/image", Image, callback=self.camera_listener)
        self.model: YOLO = YOLO('../yolo/yolov5nu.pt')
        self.results: Results = self.model(self.image_np)
        ##[...]
        self.human_detection_server = rospy.Service('detection', Detection, self.human_detection)
        ##[...]
        self.update_view()
```
Itt 
- létrehozunk egy subscriber-t `follower/camera/image` topichoz, ez veszi a robot kamerájának
képét.
- A kép feldolgozását a YOLO modell fogja végezni, amit a `self.model()` függvénnyel hívhatunk meg.
- A `Controller` felé történő kommunikációra létrehozunk egy ropsy `Service`-t; ez fogja kiszolgálni a
`Controller` kérésseit. 
- végül pedig frissítjük a képet a `self.update_view()` függvénnyel; itt egy `rospy.Sunscriber`
segítésével vesszük a kamera adatait. 

ez tulajdonképpen
hasonló szerepet tölt be, mint egy publisher, annyi külöbséggel, hogy nem folyamatosan küld adatot,
hanem hívásra (request) válaszol. A `Publisher/Subscriber` paradigma alkalmasabb folytonos adatfolyam
közvetítésére, míg a `Service/ServiceProxy` paradigma ideális alkalmankénti, egyszeri üzenetek
közvetítésére, mivel blokkoló módban működik. 
A `rospy.Service` üzenet `Detection` formátumát a fent említett `Detection.srv` ROS szerver határozza meg:
```python
string label
---
float64 box_x
float64 box_y
float64 box_width
float64 box_height
float64 image_width
float64 image_height
bool in_sight_of_robot
```
Itt a vonal feletti rész a `ServiceProxy` irányából a `Service` felé menő request formátuma, míg a
vonal alatti rész a `Service` által a `ServiceProxy` felé visszaküldött válaszé. 
Az üzenet callback függvényeét a `human detection` függvény végzi:

```python
class ImageProcessor:
    ##[...]
    def human_detection(self, req):
        self.bounding_boxes = []
        res = DetectionResponse()
        person_detected = False

        for result in self.results:
            boxes = result.boxes
            for box in boxes:
                cls = box.cls.item()
                x1, y1, x2, y2 = box.xyxy[0]
                self.bounding_boxes.append([cls, x1, y1, x2, y2])

                # Check if the detected object is a person
                label_box = self.model.names[cls]
                if label_box == "person":
                    person_detected = True
                    res.box_x = x1
                    res.box_y = y1
                    res.box_width = x2 - x1
                    res.box_height = y2 - y1
                    res.image_width = self.image_res[1]
                    res.image_height = self.image_res[0]
                    res.in_sight_of_robot = True

        # If a person is detected, return the detection response
        if person_detected:
            return res
        else:
            # If no person is detected, return response indicating not in sight
            return DetectionResponse(in_sight_of_robot=False)
```

A program ebben a formában kifejezetten emberi alakok detektálására van kiélezve, ezt természetesen 
át lehet írni bármely a YOLOv5 által detektálni képes kategóriára minimális erőbefektetéssel.


továbbá létrehozunk egy `ServiceProxy`-t - ez hasonló a subscriberhez, annyi különbséggel, hogy 



## Telepítés

### Környezet létrehozása
A telepítés feltétele természetesen a ROS és Gazebo programok használata, amelyek valamilyen
Linux operációs rendszerhez kötöttek. Mi a programot WSL segítségével Ubuntu 20.04 alatt futtattuk,
tehát ezt javasoljuk használatra. 
ROS NOETIC, GAZEBO INSTALL PARANCSOK BECOPYZVA IDE

Továbbá szükségünk lesz az ultralytics csomagra a YOLO algoritmushoz, amit felhasználtunk 
(egész pontosan YOLOv5). Ezt az alábbi paranccsal tudjuk telepíteni:
```bash
$ pip install ultralytics
```
További szükséges Python package-ek: 
- numpy: legalább 1.20.3-as verzió
- opencv legalább 4.2.0
Tehát:
```bash
$ pip install numpy==1.20.3
$ pip install opencv-python==4.2.0.34
```
Itt értelemszerűen magasabb verziószámmot választani lehet. 

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
