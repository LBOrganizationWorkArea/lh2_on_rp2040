# Bitcrazy Calibration Procedure

## Calibration Steps

<img src="images/Untitled.jpg" alt="Imagem 1" style="width: 100%; height: auto; display: block;" />

<img src="images/Untitled (1).jpg" alt="Imagem 2" style="width: 100%; height: auto; display: block; margin-top: 16px;" />

<img src="images/Untitled (2).jpg" alt="Imagem 3" style="width: 100%; height: auto; display: block; margin-top: 16px;" />

## Class LighthouseGeometrySolver

Finds the poses of base stations and Crazyflie samples given a list of matched samples.
The solver is iterative and uses least squares fitting to minimize the distance from
the lighthouse sensors to each "ray" measured in the samples.

The equation system that is solved is defined as:

Columns are the estimated poses (what we solve for). Each pose is composed of 6 numbers (often referred to as
parameters in the code): rotation vector (3) and position (3).

Rows are representing one angle from one base station. The number of rows for each sample is given by the
number of bs in the sample * n_sensors * 2.

An examples matrix (X indicates non-zero coefficients):

| Row | bs0 | bs1 | bs2 | bs3 | cf1 | cf2 |
|-----|-----|-----|-----|-----|-----|-----|
| cf0/bs2/sens0/ang0 |  |  | X |  |  |  |
| cf0/bs2/sens0/ang1 |  |  | X |  |  |  |
| cf0/bs2/sens1/ang0 |  |  | X |  |  |  |
| cf0/bs2/sens1/ang1 |  |  | X |  |  |  |
| cf0/bs3/sens0/ang0 |  |  |  | X |  |  |
| cf0/bs3/sens0/ang1 |  |  |  | X |  |  |
| cf0/bs3/sens1/ang0 |  |  |  | X |  |  |
| cf0/bs3/sens1/ang1 |  |  |  | X |  |  |
| cf1/bs1/sens0/ang0 |  | X |  |  | X |  |
| cf1/bs1/sens0/ang1 |  | X |  |  | X |  |
| cf1/bs1/sens1/ang0 |  | X |  |  | X |  |
| cf1/bs1/sens1/ang1 |  | X |  |  | X |  |
| cf1/bs2/sens0/ang0 |  |  | X |  | X |  |
| cf1/bs2/sens0/ang1 |  |  | X |  | X |  |
| cf1/bs2/sens1/ang0 |  |  | X |  | X |  |
| cf1/bs2/sens1/ang1 |  |  | X |  | X |  |
| cf2/bs1/sens0/ang0 |  | X |  |  |  | X |
| cf2/bs1/sens0/ang1 |  | X |  |  |  | X |
| cf2/bs1/sens1/ang0 |  | X |  |  |  | X |
| cf2/bs1/sens1/ang1 |  | X |  |  |  | X |
| cf2/bs3/sens0/ang0 |  |  |  | X |  | X |
| cf2/bs3/sens0/ang1 |  |  |  | X |  | X |
| cf2/bs3/sens1/ang0 |  |  |  | X |  | X |
| cf2/bs3/sens1/ang1 |  |  |  | X |  | X |
