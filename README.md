# HIVE_ROBOT_SOFTWARE

Version 0.1 du programme de controle et de navigation du robot.

* **01_system.cpp :** code de setup de redis et verification des status de chaque process.
* **02_hardware.cpp** : code de controle des Esp32 et pixhawk. (GPS - Encoder)
* **03_server.cpp :** code qui permet l'envoie et la réception des informations server.
* **04_navigation.cpp** : code qui va merge les informations des sensors et va prendre la descision pour naviguer en mode auto et manuel.

*Information code 04_navigation: il faut le lancer avec un flag quelconque.*

---



Les informations concernants la navigation du robot sont situé dans le dossier :

```
/data/robot_paramter.yaml
```

Les informations propre au robot sont situé dans les fichiers : 

```
/data/robot_id.yaml
```

```
/data/statistique_utilisation.txt
```

Les informations concernant la map sont situé dans les fichiers :

```
/data/HMD_TEST_VEZINET.txt
```


---

18 janvier 2023
