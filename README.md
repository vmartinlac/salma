TODO
====

Matériel :
* une caméra

Produit final :
* une interface graphique Qt+Openscenegraph permet d'interagir avec le moteur SLAM et de visualiser l'état et les nuages de points produits.
* produire un nuage de points dense.
* pouvoir switcher entre différents engines ? Un graph-SLAM et un SLAM incrémental ?
* pouvoir enregistrer le nuage de points produit et le visualiser dans un logiciel tel que meshlab.

Structures de données :
* une classe SLAM qui représente l'état du moteur (le nuage de points, etc).

Procédé numérique :
* prior : model based image alignment.
* posterior : keypoints retro projection.

Étapes imminentes :
1. écrire classe pour recevoir les images depuis la caméra.
2. implémenter le sparse model based image alignment.
3. a
