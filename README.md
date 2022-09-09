# HIVE_ROBOT_SOFTWARE

RANE MK4 software developed by Hive Robotics.

##### **NOTE:**

***(03/09)* Techniquement, si l'on a tout moment la vitesse de mouvement du robot. On peux donc approximé ça position autant de fois que l'on veut par seconde. On peux donc reprojeter les obtacles de manière fine et ne plus être limiter au FPS de l'algorythme de positionnement pour augmenter la sécurité.**

##### **TODO:**

*(03/09)* Récuperer les informations importantes de status de la pixhawk, ex: local position, nombre de satelite, mode du robot(hold, lost, prés a voler).

*(04/09)* Gérer le programme 03 lorsqu'il n'arrive pas à ce connecté au server.

##### **FEATURE:**

*(26/08)* Ajouter un evenement qui permet de detecter quand le robot est soulever ou qu'il est renverser.

~~*(01/09)* Ajouter un timer de sécurité qui permet de couper le robot lorsqu'on est en mode manuel et qu'il y a une coupure de connection.~~

*(02/09)* Ajouter la fonctionnalité thread status qui permet d'avoir une idée précise des programmes qui tournent et ceux qui ne tournent pas.

*(02/09)* Ajouter dans 01 ou 03 un thread qui permet de lire l'ensemble des êvements généré par le robot et de les sauvegardés.

*(03/09)* Reset all redis value nécessaire au bon fonctionnement de 02 à chaque lancement.

*(03/09)* Que faire si on detect un port de connection au MCU, mais qu'il est quand même impossible de communiquer, ex: sur-bouqué ? Potentiellement mettre un watchdog chargé de reset le programme de manière plus bas niveau, ex: eteindre rallumer port.

~~*(03/09)* Ajouter lors du démarage du process 02 une configuration complete des ports de communication, ex: baudrate, bit de parité...~~

*(04/09)* Completer toute les conditions pour passer en mode auto dans la fonction auto_mode_available(), notament le faite que tout les programmes soit en marche. Pareil pour le mode manuel qui necessite un retour stream obligatoire.

*(04/09)* Ajouter une information pour le mode de parking effectué manuellement par un opérator, demander le nombre de road avant ou l'opérator doit prendre le relais.

*(04/09)* Ajouter un parametre qui permet de choisir si l'on souhaite que lidar ou autre rentre en jeu lors du pilotage manuel, ou bien que le robot se coupe automatique apres une certaine durée, ou qu'il roule à une vitesse max.

*(04/09)* Au (re)démarage du robot, choisir si l'on veut ouvrir les trappes à cargo ou non en fonction de la situation.

*(05/09)* Pour les events interne qui peuvent aussi mener à un message server, faire un publisher/subscriber.

*(05/09)* Ajouter au pilotage manuel, au niveau de la rotation sur place du robot, une différence de vitesse entre les roues centrals et les 4 roues diagonals.

*(05/09)* AJouter, "si le programme 04 ne tourne pas > envoyer stop motor". Puis tenter de le rallumer ?

*(08/09)* Récuperer des informations concernant l'état des roboclaws, l'état des encodeurs des moteurs, la température des roboclaw et le voltage de la batterie.

*(08/09)* Que faire si l'un des programmes plantes ?

*(09/09)* Ajouter les variables SERVER_CONNECTION_TIMESTAMP, SERVER_CONNECTION_DURATION, SERVER_LAST_MSG_TIMESTAMP et SERVER_LAST_MSG_DURATION.

*(09/09)* Gérer la callback du programme Server qui va lire les EVENTS.

**WIFI:** 8PfURsp!dvic
