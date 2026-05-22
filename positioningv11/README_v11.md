# positioningv11 mode d'emploi

v11 est la version "positions Lighthouse connues".

L'idee est simple: au lieu de laisser le fit deviner completement ou sont les bases, on mesure leurs positions a la main, on les fixe dans le repere de la piece, puis le logiciel calibre surtout:

- l'orientation de chaque Lighthouse;
- le signe et l'axe des sweeps;
- les offsets d'angle;
- la bonne famille `LH2A` quand deux familles apparaissent.

Ce workflow n'a pas encore ete valide completement en pratique. Ce README est donc le mode d'emploi prevu pour demarrer proprement une calibration v11 depuis zero.

## 0. Se placer dans v11

```powershell
cd C:\Users\elkah\lh2_positioning\positioningv11
```

## 1. Choisir le repere de la piece

Choisir un repere simple et le garder partout:

- `x`: gauche/droite;
- `y`: avant/arriere, typiquement positif vers les Lighthouse;
- `z`: hauteur depuis le sol;
- unite: metres.

Les points connus du wand sont dans:

```text
config/wand_3d_points.json
```

Les positions des Lighthouse doivent etre exprimees dans le meme repere que ces points.

## 2. Entrer les positions mesurees des Lighthouse

Editer:

```text
config/lighthouse_positions.json
```

Exemple:

```json
{
  "description": "Measured Lighthouse positions in the room frame. Edit these values before fitting v11.",
  "unit": "meter",
  "basestations": [
    {"basestation": 4, "x_m": -0.70, "y_m": 1.85, "z_m": 0.70},
    {"basestation": 10, "x_m": 0.70, "y_m": 2.20, "z_m": 0.70}
  ]
}
```

Mesurer au minimum:

- hauteur des deux bases;
- distance entre les deux bases;
- position approximative de chaque base par rapport au centre des points de calibration.

Si une base est un peu plus loin que l'autre, mettre cette asymetrie dans le fichier. v11 depend fortement de ces positions.

## 3. Verifier que le firmware envoie bien les angles directs

```powershell
py .\tools\17_diagnose_lh2a_families.py --port COM3 --baudrate 115200 --duration 5 --cluster-deg 8
```

On veut voir:

- beaucoup de lignes `LH2A`;
- `channels=16/16` ou l'equivalent dans le diagnostic;
- deux familles stables sur la plupart des canaux;
- des spreads bas quand le wand ne bouge pas.

Si `LH2A=0`, verifier le firmware avant de continuer.

## 4. Capturer quelques points connus

Commencer avec 4 points qui couvrent sol + hauteur:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 2 --resume --only P00_bas_avant_gauche,P04_bas_centre,P08_bas_arriere_droite,P11_boite45_centre
```

Pendant chaque capture:

- placer le wand exactement au point demande;
- attendre 1 a 2 secondes;
- appuyer sur Entree;
- ne pas bouger pendant la capture.

Le fichier de sortie est:

```text
config/wand_calibration_poses_3d_lh2a_families.json
```

Le script sauvegarde apres chaque pose. Avec `--resume`, il reprend sans refaire les points deja captures.

## 5. Valider les captures

Validation simple:

```powershell
py .\tools\20_validate_lh2a_family_capture.py
```

Validation stricte conseillee:

```powershell
py .\tools\20_validate_lh2a_family_capture.py --max-spread-deg 0.5
```

Un bon point doit avoir:

- `channels=16/16`;
- `two-family=16/16`;
- `max_spread` idealement sous `0.5deg`.

Pour refaire seulement un point mauvais:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 1.0 --resume --recapture P00_bas_avant_gauche --only P00_bas_avant_gauche
```

Si un point reste instable, essayer `--cluster-deg 0.7` et bien stabiliser le wand avant d'appuyer sur Entree.

## 6. Premier fit avec positions Lighthouse fixes

Quand les 4 premiers points sont propres:

```powershell
py .\tools\20_fit_known_lighthouse_positions.py
```

Sortie:

```text
config/lighthouse_geometry_known_positions.json
```

Le RMSE affiche est une erreur d'angle en degres, pas une erreur en metres.

Si le RMSE est haut, tester les conventions:

```powershell
py .\tools\20_fit_known_lighthouse_positions.py --model-variants all
```

Si le RMSE reste haut:

- verifier `config/lighthouse_positions.json`;
- verifier que les points captures sont propres;
- verifier que les positions du fichier `wand_3d_points.json` correspondent bien aux vrais points physiques.

## 7. Ajouter plus de points

Quand le fit 4 points semble plausible, capturer plus de points:

```powershell
py .\tools\18_capture_lh2a_family_poses.py --port COM3 --baudrate 115200 --duration 3 --cluster-deg 2 --resume
```

Puis revalider:

```powershell
py .\tools\20_validate_lh2a_family_capture.py --max-spread-deg 0.5
```

Refaire les points trop larges avant de relancer le fit.

## 8. Verifier le resultat

Apres le fit, verifier:

- RMSE raisonnable;
- orientations stables;
- pas de convention de sweep absurde;
- position live stable si un script live est utilise ensuite.

Le fichier de geometrie v11 doit garder les positions Lighthouse mesurees, pas les remplacer par une position libre.

## 9. Etape suivante prevue: wand waving

Une fois les positions fixes + orientations initiales correctes, enregistrer un wand waving pour raffiner la calibration avec beaucoup plus d'observations:

```powershell
py .\tools\09_record_wand_wave.py --port COM3 --baudrate 115200 --duration 60
```

But du wand waving:

- renforcer les orientations;
- confirmer le choix des familles dans le temps;
- reduire les offsets residuels;
- tester le volume ou le drone devra vraiment voler.

Le workflow de refinement wand waving v11 reste a finaliser apres validation du fit positions fixes.
