# HIVE_ROBOT_SOFTWARE

RANE MK4 software developed by Hive Robotics.

**NOTE:**

***(02/09)* Il semble que la fonction is_open ne passe a false uniquement si on utilise la fonction Close(), en cas de débranchement, après un Open(), elle restera a jamais a True.**

**TODO:**

**FEATURE:**

*(26/08)* Ajouter un evenement qui permet de detecter quand le robot est soulever ou qu'il est renverser.

*(01/09)* Ajouter un timer de sécurité qui permet de couper le robot lorsqu'on est en mode manuel et qu'il y a une coupure de connection.

*(02/09)* Ajouter la fonctionnalité thread status qui permet d'avoir une idée précise des programmes qui tournent et ceux qui ne tournent pas.

*(02/09)* Ajouter dans 01 ou 03 un thread qui permet de lire l'ensemble des êvements généré par le robot et de les sauvegardés.

*(03/09)* Reset all redis value nécessaire au bon fonctionnement de 02 à chaque lancement.

*(03/09)* Que faire si on detect un port de connection au MCU, mais qu'il est quand même impossible de communiquer, ex: sur-bouqué ? Potentiellement mettre un watchdog chargé de reset le programme de manière plus bas niveau, ex: eteindre rallumer port.

**WIFI:** 8PfURsp!dvic
