- Linux geupdatet
- Colcon installiert
- Ros2 installiert (Jazzy Jalisco Version, da Ubuntu 24.04.)
- joint_state_broadcaster und gz_ros2_control installiert
um Roboter mit MoveIt zu kontrollieren oder mit Gazebo zu simulieren

igus_rebel starten mit Ros2 -> cd in ros2_ben -> source install/setup.bash
 -> ros2 launch igus_rebel rebel.launch.py
-> ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py
use_gui:=true
-> Um den Roboterarm zu erreichen muss eine statische Ip vergeben werden : 192.168.3.1 mit Subnetz 255.255.255.0

https://bitbucket.org/truphysics/igus_rebel_ros2/src/main/ Repository Igus
https://docs.ros.org/en/jazzy/index.html Dokumentation Jazzy ROS2

09.09.25 - 3 Stunden

///////////////////////////////////////////////////////////////////////////////////////

in ~ Ordner -> .bashrc -> source ros2_ben/install/setup.bash ->
jede neue Shell sourced automatisch unseren ROS2 Ordner

ROS2 Bedienung:
	- "ros2 node list" -> Zeigt aktive Nodes
	- "ros2 node info <nodename>" -> infos

mit OrbbecViewer Orbbec Astra Mini getestet

in ros2_ben/src OrbbecSDK für ROS2 gecloned mit git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

-> Versucht zu starten mit ros2 launch orbbec_camera astra.launch.py
- Fehler wegen falscher Depth Auflösung von 640x480 statt verfügbar 640x400
- in ros2_ben/src/OrbbecSDK_ROS2/orbbec_camera/launch/astra.launch.py die Default Height auf 400 abgeändert
- Build Prozess wiederholt mit  "colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release"
- Camera Topic gestartet mit "ros2 launch orbbec_camera astra.launch.py" -> dann hinzufügbar in RVIZ

Problem "unable to transform object from frame "camera_depth_frame" to planning frame "world"
(could not find a connection between "world" and "camera_depth_frame" because they are not part of the same tree.
 Tf has two or more unconnected trees.)

Hilfe von Niklas :
	- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 link6 camera_link
-> statische Beziehung zwischen link6 (letztes Roboterglied) und camera_link
-> Ausrichtung X Y Z QX QY QZ (Quarternion)
-> wird hier in Radient angegeben -> wir brauchen 3.14 0 0 , da unsere Kamera in Rviz um 180° auf der X Achse gedreht
war


To-Do :
	- Kamera am Roboterarm montieren
	- genaue Lage von camera_link zu link6 herausfinden

	- RGB Bild publishen, eine Yolo Node subscribed und erstellt aus den Bilddaten Boundingbox o.Ä.?, RGB Pixel
	des gefundenen Werkzeugs transformieren/umrechnen zu 3D Punkt der Punktwolke (Beziehung RGB zu Depth)
	- Pose ermitteln -> Greifer Pose ermitteln (oder Magnet o.Ä.) -> IK zum ermittelten Punkt

	- eine URDF Datei schreiben oder mehrere launch.py in einer großen launch verbinden

10.09.25 - ~5 Stunden

///////////////////////////////////////////////////////////////////////////////////////

Ziel -> Startvorgang einfacher machen, alle wichtigen Sachen in einem File
-> Überlegung URDF oder launch file

Suche nach urdf Dateien der bereits vorhandenen Igus Nodes
/home/amrl-linux24-04/ros2_ben/src/igus_rebel_ros2/igus_rebel_description/urdf/
	-> hier wird nur das 3D Modell für Rviz beschrieben
	- stl Dateien für alle Links und Motoren mit den jeweiligen Positionen

Wie ist es bisher?
->
gestartet wird Rviz mit dem Roboter und Motionplanner über move_group.launch.py
welche in igus_rebel_motion_planner.launch.py referenziert wird


Eigene Launch File schreiben ->
- in ros2_ben/src/ ein neues Verzeichnis mit "mkdir igusgus/src" erstellt
- mit cd in src Ordner gewechselt
- mit
"ros2 pkg create --build-type ament_python igusgus_launch"
ein neues Paket erstellt welches die richtige Struktur für Launch files beinhaltet

- eine Launch file in
"~/ros2_ben/src/igusgus/src/igusgus_launch/launch/igusgus_launch.py
igusgus hier der Paketname

vom root Ordner ros2_ben kann man nun mit "ros2 launch igusgus_launch igusgus_launch.py" diese Launch File starten

igusgus_launch.py ist die Toplevel Launch File und sie ruft die anderen Launch files für Igus, moveit_config und orbbec_camera auf

aufgerufen über
"ros2 launch igusgus_launch igusgus_launch.py"

-> RViz Konfigurieren
- RVIZ starten mit igusgus_launch -> Oberfläche einstellen
-> Einstellungen speichern als igusgus.rviz in igus_rebel_moveit_config
-> move_group.launch.py ruft dies als RVIZ Config auf

Roboter Definitionen für das Modell in "igus_rebel2.urdf"

11.09.25 - ~5 Stunden

///////////////////////////////////////////////////////////////////////////////////////

Orbbec Kamera durch Intel realsense ersetzt

Build von Source da sonst Kernel Fehler bei librealsense
- Launch Datei abgeändert für Realsense Paket
- nicht sicher ob Kamera wirklich besser für Usecase, aber wesentlich mehr Parameter änderbar

- Cad Datei angefangen für Kamera Halter

12.09.25 - ~ 7 Stunden
