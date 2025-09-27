# Projet : Plateforme logicielle pour la Robotique Collaborative

## Description
Ce dépot contient le travail réalisé pour l'équipe IRA2 du laboratoire IBISC, visant à développer une plateforme logicielle intégrant ROS2, Unity 3D, un robot collaboratif Universal Robots UR3e, un bras haptique 3D Systems Touch et un casque Meta Quest 2. Le projet permet une interaction immersive et en temps réel entre l'utilisateur et le robot, en traduisant les mouvements haptiques en commande robotiques, avec synchronisation visuelle dans Unity.

Le package ROS2 `unity_robotics_ira2` gère la communication et le contrôle, tandis que le projet Unity `project_ira2_ur`fournit l'interface immersive et haptique. Un projet Unity `Project_RosUnity` est également disponible pour tester la communication entre ROS2 et la scène Unity projeté dans le casque.

### Prérequis
- **Système d'exploitation** : Ubuntu 24.04 (recommandé pour ROS2) ou Windows 10/11 avec WSL2.
- **ROS2** : Version Jazzy Jalisco (installée via [guide officiel](https://docs.ros.org/en/jazzy/Installation.html)).
- **Unity** : Version 2022.3 LTS (pour compatibilité XR).
- **Hardware** :
  - Robot UR3e avec contrôleur e-Series. Installation du driver Universal Robots pour ROS2 (vers [ici](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy)) et de l'interface RTDE (vers [ici](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html)).
  - Bras haptique 3D Systems Touch avec pilotes installés.
  - Meta Quest 2 avec Oculus app.
- **Réseau** : Tous les composants sur le même sous-réseau

### Installation

#### 1. Installez les dépendances ROS2 du projet :
  ```
  cd ~/ros2_ws/src
  git clone https://github.com/MaksNemanja/Project-Unity-Robotics-IRA2.git
  cd ~/ros2_ws
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --packages-select unity_robotics_ira2
  source install/setup.bash
  ```

#### 2. Installation Unity
- Téléchargez Unity Hub depuis [unity.com](https://unity.com/download).
- Installez Unity 2022.3 (ou version plus récente) LTS via Unity Hub.
- Dans le projet `project_ira2_ur` :
  - Importez le package ROS TCP Connector via Package Manager (Add package from git URL : `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`).
  - Importez URDF Importer : `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`.
  - Ajoutez OpenXR pour Quest 2 : Window > Package Manager > XR Plugin Management > Install OpenXR.
  - Importez HapticsDirectforUnityv1 : Téléchargez le package depuis le Unity Asset store (lien [ici](https://assetstore.unity.com/packages/tools/integration/haptics-direct-for-unity-v1-197034)), puis importez-le dans le projet via Assets > Import Package > Custom Package.
 
 #### 3. Configuration Hardware
- **UR3e** : Connectez via Ethernet. Activez RTDE dans Polyscope (Installation > RTDE).
- **Bras haptique** : Installez les pilotes 3D Systems et configurez via Touch Smart Setup.
- **Meta Quest 2** : Installez Oculus app et activez Link/Air Link.

  ### Utilisation
  
  #### 1. Lancer le driver UR
  - Fenêtre 1 (modifier l'adresse IP du robot si besoin):
  ```
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.1.101 use_fake_hardware:=false launch_rviz:=true

#### 2. Lancer le Nœud ROS2
- Dans une nouvelle fenêtre :
  ```
  source ~/ros2_ws/install/setup.bash
  ros2 run unity_robotics_ira2 haptic_control
  ```
- Cela lance le nœud principal pour le contrôle haptique et robotique.

#### 3. Lancer Unity
- Ouvrez le projet `project_ira2_ur` dans Unity 2022.3 LTS.
- Configurez ROS Settings (Edit > Project Settings > ROS Settings).
- Lancez la scène  `UR3e haptic` pour le contrôle haptique.

#### 4. Scénarios de Test
- **Scénario 1 (Commandes de bases)** : Utilisez `robot_control` pour déplacer le robot avec le clavier (flèches pour vitesses x/z). Lancez la scène `URtoUnity` pour un suivi de l'effecteur du robot.
- **Scénario 2 (Connexion ROS2-Quest 2)** : Changez la couleur d'un objet dans Unity via le nœud `color_publisher`, avec le projet `Project_RosUnity`, pour une visualisation sur Quest 2.
- **Scénario 3 (communication de Unity vers ROS2)**: Lancez la scène `UnitytoUR` pour controler un objet avec les flèches du clavier. Avec le noeud `unity_control` pour controler le mouvement du robot UR selon le déplacement du cube.
- **Scénario 4 (Modèle URDF)** : Lancez la scène `UR3e_unity_to_robot` et manipulez les articulations avec le clavier, synchronisez avec le robot réel le noeud `joints_sub`.
- **Scénario 5 (Déplacement avec souris)** : Lancez la scène `UR3e`, puis déplacez le cube avec la souris. Lancez le noeud `unity_control_v2` pour que le robot suive la position du cube.
- **Scénario final** : Contrôlez le robot avec le bras haptique, synchronisez le modèle URDF dans Unity.

### Structure du Projet

#### Package ROS2 `unity_robotics_ira2`
- `src/haptic_control.cpp` : Nœud principal pour le contrôle haptique et robotique.
- `src/robot_control.cpp` : Contrôle manuel avec clavier.
- `src/unity_control_v2.cpp` : Contrôle depuis Unity en 2 dimensions.
- `src/unity_control_v2.cpp` : Contrôle depuis Unity en 3 dimensions.
- `src/joints_sub.cpp` : Souscription aux états articulaires pour évaluation.
- `src/color_publisher.cpp` : Changer la couleur d'un objet sur Unity.
- `msg/HapticInfo.msg` : Message personnalisé pour les données haptiques.

#### Projet Unity `project_ira2_ur`
##### Scripts :
- `Assets/Resources/Scripts/Ariculations_pub.cs` : publier la configuration articulaire du modèle URDF.
- `Assets/Resources/Scripts/Ariculations.cs` : reproduire la configuration articulaire du robot UR.
- `Assets/Resources/Scripts/MoveObject2D.cs` : déplacer un objet sur la scène selon 2 dimensions.
- `Assets/Resources/Scripts/MoveObject3D.cs` : déplacer un objet sur la scène selon 3 dimensions.
- `Assets/Resources/Scripts/Position_Publisher_with_limits.cs` : envoi de la position d'un objet de la scène avec un espace de travail.
`Assets/Resources/Scripts/Position_Publisher.cs` : envoi de la position d'un objet de la scène.
- `Assets/Resources/Scripts/Position_Subscriber.cs` : suivi de la position du robot avec objet sur la scène.
- `Assets/Resources/Scripts/RosPublisherExample.cs` : example de publisher pour ROS2 (utiliser avec le projet `Project_RosUnity`).
- `Assets/Resources/Scripts/RosSubscriberExample.cs` : example de subscriber pour ROS2 (utiliser avec le projet `Project_RosUnity`).
- `Assets/Resources/Scripts/stateButtons.cs` : suivi de l'état des boutons du dispositif haptique et publication de sa position.
- `Assets/Resources/Scripts/SubForce.cs` : retour de force appliquer sur le dispositif haptique.

##### Scènes : 
- `Assets/Scenes/URtoUnity` : pour la communication de ROS2 vers Unity, avec un suivi de la position de l'effecteur.
- `Assets/Scenes/UnitytoUR` : pour la communication de Unity vers ROS2, avec déplacement d'un cube sur la scène.
- `Assets/Scenes/UR3e_unity_to_ros` : modèle URDF de l'UR3e avec changement des articulations avec les flèches du clavier.
- `Assets/Scenes/UR3e` : scène avec URDF de l'UR3e et cube. Manipulation du cube avec la souris selon 3 dimensions, et suivi du robot réel.
- `Assets/Scenes/Ur3e haptic` : scène avec URDF de l'UR3e et du  3D Systems Touch. Suivi du mouvement de translation du dispositif haptique par le robot.


  ### Contact
- Nemanja Maksimovic : nemanja.maksimovic@etu.sorbonne-universite.fr
