# CartesianServo + Haptic Bridge — Instrukcja wdrozenia

Cartesian streaming servo dla robota ES5 z teleoperacja przez Phantom Omni/Touch.

## Architektura

```
Phantom Omni         haptic_bridge.py        CartesianServo (OROCOS 500Hz)       Robot
  ~1 kHz                100 Hz                  exp. smoothing + vel limit

/phantom/pose ---> [clutch + scale + R] ---> /es_cartesian_servo/command
PoseStamped            delta * scale           [IIR filter + SLERP]
                                                      |
/phantom/button --> engage/disengage                   v
                    (anchor poses)               IK -> LimitDetector -> Motors
                                                      ^
/es_arm/cartesian_pose <----- FK ---------------------+
```

## Pliki do skopiowania

Caly katalog `cartesian_servo/` kopiujemy do workspace robota:

```bash
scp -r cartesian_servo/  USER@ROBOT:~/ws_easy/underlay/src/orocos_controllers/
```

Plik wiring `.ops` kopiujemy do bringup:

```bash
scp ops/cartesian-servo.ops  USER@ROBOT:~/ws_easy/underlay/src/master_es/es5_bringup/
```

## Zmiany w istniejacych plikach (na robocie)

### 1. `master_es/es_bringup/common-imports.ops`

Dodaj na koncu pliku:

```
ros.import("cartesian_servo")
```

### 2. `master_es/es5_bringup/es5-hardware.ops`

**Linia 257** — zakomentuj polaczenie ESPoseInt -> IK:

```ops
// BYLO:
connect("ESPoseInt.CartesianPositionCommand", "IK.InputEndEffectorPose", ConnPolicy())

// ZMIEN NA:
//connect("ESPoseInt.CartesianPositionCommand", "IK.InputEndEffectorPose", ConnPolicy())
```

**Przed linia `ESHandGuiding.start()`** (okolo linii 952) — dodaj:

```ops
runScript( ros.find("es5_bringup") + "/cartesian-servo.ops")
```

Sekcja koncowa powinna wygladac tak:

```ops
stream("EC.joint1.Diagnostic", ros.comm.topic("/es_arm/diag1test"))

runScript( ros.find("es5_bringup") + "/cartesian-servo.ops")

ESHandGuiding.start()
//ESImpedance.start()
EC.start()
```

## Budowanie

```bash
ssh USER@ROBOT
cd ~/ws_easy/underlay
catkin_make
source devel/setup.bash
```

## Uruchomienie

**Terminal 1** — System robota (standardowe uruchomienie OROCOS z es5-hardware.ops)

**Terminal 2** — Haptic bridge:

```bash
source ~/ws_easy/underlay/devel/setup.bash
roslaunch cartesian_servo haptic_teleop.launch
```

Jesli driver haptic uzywa innych topicow:

```bash
roslaunch cartesian_servo haptic_teleop.launch \
  haptic_pose_topic:=/twoj/topic/pose \
  haptic_button_topic:=/twoj/topic/button
```

## Pierwszy test (ostroznie!)

### Bez robota — sprawdz bridge

```bash
# monitoruj output:
rostopic echo /es_cartesian_servo/command

# symuluj przycisk:
rostopic pub /phantom/button std_msgs/Bool "data: true"

# symuluj pozycje haptic:
rostopic pub -r 50 /phantom/pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'base'},
    pose: {position: {x: 0.01, y: 0.0, z: 0.0},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

### Na robocie — bezpieczne parametry startowe

```bash
# niski scaling (1x zamiast 3x)
roslaunch cartesian_servo haptic_teleop.launch scale_position:=1.0

# niska predkosc CartesianServo (5 cm/s)
rostopic pub /es_cartesian_servo/max_linear_speed \
  std_msgs/Float32MultiArray "data: [0.05]"
```

### Kalibracja frame alignment

1. Ustaw `frame_rotation_rpy: [0, 0, 0]` w launch file
2. Engage (wcisnij przycisk), rusz joystick w prawo
3. Obserwuj w ktorym kierunku jedzie robot
4. Koryguj RPY az ruchy beda intuicyjne (prawo=prawo, przod=przod, gora=gora)
5. Typowe ustawienie (Phantom przed robotem): `rpy: [0, 0, -1.5708]`

## Topici ROS

| Topic | Typ | Opis |
|-------|-----|------|
| `/es_cartesian_servo/command` | Pose | Komenda z bridge -> servo |
| `/es_cartesian_servo/velocity` | Float32MultiArray | Predkosc kartezjanska [vx,vy,vz,wx,wy,wz] |
| `/es_cartesian_servo/active` | Bool | Czy servo aktywnie sledzi |
| `/es_cartesian_servo/status` | Int32 | 0=IDLE, 1=TRACKING, 2=DECELERATING |
| `/es_cartesian_servo/smoothing_alpha` | Float32MultiArray | Online tuning gladkosci |
| `/es_cartesian_servo/max_linear_speed` | Float32MultiArray | Online tuning predkosci [m/s] |
| `/es_cartesian_servo/debug` | Bool | Wlacz debug logi |

## Powrot do trybu bez CartesianServo

1. Odkomentuj linie 257 w `es5-hardware.ops`
2. Zakomentuj/usun `runScript(..."cartesian-servo.ops")`
3. Przebuduj (`catkin_make`)

## Tryb momentowy (CST z ESImpedance)

Odkomentuj sekcje TORQUE MODE w `cartesian-servo.ops` i aktywuj ESImpedance
w `es5-hardware.ops` (linie 863-871). Szczegoly w komentarzach w plikach.
