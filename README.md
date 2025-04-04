# Projet Industriel : Manipulation à distance avec retour d'effort de sondes échographiques

# Présentation

Ce repository contient tous les packages ros et scripts permettant de faire fonctionner l'UR avec l'Haply par réseau.
- Les scripts haply.bash et ur.bash à la racine permettent d'effectuer les configurations et de lancer les noeuds ros nécessaires au fonctionnement de l'Haply et de l'UR
- Le dossier src/ contient tous les packages ros nécessaires, à savoir :
    omni_msgs pour créer les fichiers d'en-tête nécessaires pour pouvoir utiliser certains types de messages ros. Ces messages étaient nécessaires pour les topics du Phantom, mais nous les avons gardés pour l'Haply car ils permettent de créer des messages contenant à la fois les vitesses en translation et les quaternions pour les rotations. 

    ros2_haply_inverse3_python qui contient les noeuds ros permettant de faire fonctionner l'Haply, écrits en python

    ur_control qui contient les noeuds ros permettant de faire fonctionner l'UR, ainsi que les launchfiles utilisés par les scripts de lancement pour lancer différents noeuds simultanément

    ur_udp qui contient les noeuds ros permettant de faire fonctionner la partie réseau avec le protocole de communication UDP

    ur3e_moveit_config qui est un package custom de moveit, permettant entre autres de modifier la configuration de moveit

# Installation

- Prérequis : 
Ce projet a été réalisé pour des ordinateurs sous Linux, avec Ubuntu 22.04

- Télécharger et lancer le script d'installation install_all.sh pour télécharger toutes les dépendances nécessaires facilement 
```
./install_all.sh
```

# Lancements

- Réseau : 

Dans le cadre de ce projet, nous avons fait nos tests avec des ordinateurs connectés sur un même réseau via un routeur, il suffit donc de connecter les 2 ordinateurs à ce routeur par câbles ethernet, et de désactiver le wifi si celui ci est actif.

- Haply : 

(Intégrer la partie du readme de Samuel sur le démarrage de l'Haply)

Sur l'ordinateur gérant le contrôle de l'Haply, simplement lancer le script de lancement haply.bash
```
./haply.bash
```

- UR : 

Une fois l'UR allumé, appuyer sur le bouton rouge en bas à gauche de la tablette et suivre les instructions pour le démarrer. Puis charger le programme ros_control et appuyer sur le bouton play lorsque le reste des programmes seront lancés.
Connecter l'UR à l'ordinateur gérant le contrôle de l'UR par câble ethernet, puis simplement lancer le script de lancement ur.bash
```
./ur.bash
```

# Contrôle de l'UR

Une fois tout bien lancé, il suffit d'appuyer sur l'un des deux boutons du stylet pour pouvoir contrôler l'UR en déplaçant l'Haply

# Débuggage

En cas de problèmes, il y a certaines choses à vérifier : 

- Vérfier que les adresses IP des ordinateurs correspondent bien à celles indiquées dans les programmes réseau (l'adresse IP de l'ordinateur contrôlant l'UR doit correspondre à celle du programme src/ur_udp/phantom_udp.cpp)
- Vérifier dans les topics (présents sur le schéma de la partie réseau du rapport final fourni) que des messages sont bien envoyés
```
ros2 topic echo /topic_name
```
- Problèmes de connexion possibles entre l'ordinateur et l'Haply : relancer le script de l'Haply dans ce cas
- Problèmes possibles au niveau du calibrage du stylet, qu'il faut simplement refaire en le positionnant à plat
