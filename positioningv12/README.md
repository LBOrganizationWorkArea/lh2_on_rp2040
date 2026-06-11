# positioningv12 mode d'emploi

v12 est le workflow "repere Lighthouse d'abord".

Le principe:

1. BS4 est fixe comme origine du repere Lighthouse.
2. Le wand waving sert a calibrer BS10 par rapport a BS4.
3. Quelques points connus servent ensuite a transformer le repere Lighthouse vers le repere de la piece.
4. En live, on estime la pose du drone par PnP/optimisation d'angles dans le repere Lighthouse, puis on convertit vers la piece.

Ce workflow est plus propre que de forcer directement les Lighthouse dans le repere piece au debut, parce que la calibration relative Lighthouse n'a pas besoin de connaitre tout de suite le sol, le centre de la piece, ou l'orientation globale.

## 0. Se placer dans v12

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv12
```

## 1. Verifier LH2A

```powershell
py .\tools\17_diagnose_lh2a_families.py --port COM3 --baudrate 115200 --duration 5 --cluster-deg 8
```

On veut:

- beaucoup de `LH2A`;
- les 16 canaux visibles;
- deux familles stables par canal;
- spread bas quand le drone/wand ne bouge pas.

## 2. Enregistrer le wand waving en repere Lighthouse

Bouger le drone/wand dans le volume utile. Il faut varier:

- gauche/droite;
- avant/arriere;
- hauteur;
- rotations douces.

Commande:

```powershell
py .\tools\21_record_lh2a_wave.py --port COM3 --baudrate 115200 --duration 60 --window 0.20 --period 0.25
```

Sortie:

```text
config/lh2a_wave_record.json
```

Ce fichier ne contient pas de positions connues. Il contient seulement les familles d'angles par frame.

## 3. Fit relatif Lighthouse

But:

- BS4 reste fixe a `(0,0,0)`;
- BS4 definit les axes du repere Lighthouse;
- BS10 est estime relativement a BS4;
- les poses du drone pendant la wave sont des variables internes;
- le solveur choisit les familles `LH2A` coherentes.

Commande prevue:

```powershell
py .\tools\22_fit_relative_lighthouse_frame.py --wave config\lh2a_wave_record.json
```

Sortie prevue:

```text
config/lighthouse_relative_geometry.json
```

Note: cette etape est le coeur de v12. Le script est pose comme point d'entree du workflow; l'optimisation bundle adjustment sera finalisee apres la premiere capture wave.

## 4. Capturer quelques points pour ancrer la piece

Une fois la geometrie relative Lighthouse plausible, capturer quelques points connus. Ces points ne servent pas a trouver BS10; ils servent a calculer:

```text
repere Lighthouse -> repere piece
```

Commande:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 1.0 --resume --only P00_bas_avant_gauche,P04_bas_centre,P08_bas_arriere_droite,P11_boite45_centre
```

Puis valider:

```powershell
py .\tools\20_validate_lh2a_family_capture.py --max-spread-deg 0.5
```

## 5. Calculer l'ancrage piece

Commande prevue:

```powershell
py .\tools\23_anchor_lighthouse_frame_to_room.py --geometry config\lighthouse_relative_geometry.json --points config\wand_calibration_poses_3d_lh2a_families.json
```

Sortie prevue:

```text
config/lighthouse_to_room_transform.json
```

Cette transformation convertit les positions calculees dans le repere Lighthouse vers le repere de la piece.

## 6. Pose live / PnP

En live:

1. lire les angles `LH2A`;
2. utiliser la geometrie relative Lighthouse;
3. resoudre la pose du drone avec les capteurs fixes du layout;
4. convertir la pose vers la piece avec `lighthouse_to_room_transform.json`.

Commande prevue:

```powershell
py .\tools\24_live_pnp_lighthouse_frame.py --port COM3 --baudrate 115200
```

## Pourquoi deux Lighthouse aident

Une Lighthouse donne surtout une direction angulaire. Deux Lighthouse donnent deux vues, donc une triangulation beaucoup plus forte.

Avec 4 capteurs fixes sur le drone:

```text
2 Lighthouse + 4 capteurs = pose 6D beaucoup plus robuste
```

BS4 sert de repere. BS10 apporte la deuxieme vue.

## Ordre recommande

1. Diagnostiquer `LH2A`.
2. Enregistrer une wave propre.
3. Fit relatif `BS4 -> BS10`.
4. Capturer 4 a 8 points propres.
5. Calculer l'ancrage vers la piece.
6. Tester PnP live.
7. Refaire une wave plus longue et raffiner.
