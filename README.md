TODO
====

Matériel :
* une caméra

Produit final :
* une interface graphique Qt+Openscenegraph permet d'interagir avec le moteur SLAM et de visualiser l'état et les nuages de points produits.
* produire un nuage de points dense.
* pouvoir switcher entre différents moteurs ?  Un graph-SLAM et un SLAM incrémental ?  Pouvoir combiner différents prédicteur (celui ici de la mécanique du solide rigide et celui basé sur le model based image alignment ?).
* pouvoir enregistrer le nuage de points produit et le visualiser dans un logiciel tel que meshlab.

Structures de données :
* une classe SLAM qui représente l'état du moteur (le nuage de points, etc).
* MainWindow.

Procédé numérique :
* prior : model based image alignment.
* posterior : keypoints retro projection.

Étapes imminentes :
1. écrire classe pour recevoir les images depuis la caméra.
2. implémenter le sparse model based image alignment.

